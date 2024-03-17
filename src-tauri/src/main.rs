#![cfg_attr(
    all(not(debug_assertions), target_os = "windows"),
    windows_subsystem = "windows"
)]
use axum::{
    body::Bytes,
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        State,
    },
    response::IntoResponse,
    routing::get,
    Router,
};
use tokio::net::TcpListener;

use futures::{sink::SinkExt, stream::StreamExt};
use http::Method;
use std::{
    collections::HashSet,
    sync::{Arc, Mutex},
    time::Duration,
};
use tokio::sync::broadcast;
use tower::ServiceBuilder;
use tower_http::{
    cors::{Any, CorsLayer},
    timeout::TimeoutLayer,
    trace::{DefaultMakeSpan, DefaultOnResponse, TraceLayer},
    LatencyUnit, ServiceBuilderExt,
};

mod components; // Import all the components
mod handlers;
mod setup;

#[derive(Clone, serde::Serialize)]
struct Payload {
    args: Vec<String>,
    cwd: String,
}
// Our shared state
struct AppState {
    // We require unique usernames. This tracks which usernames have been taken.
    user_set: Mutex<HashSet<String>>,
    // Channel used to send messages to all connected clients.
    tx: broadcast::Sender<String>,
}

fn check_username(state: &AppState, string: &mut String, name: &str) {
    let mut user_set = state.user_set.lock().unwrap();

    if !user_set.contains(name) {
        user_set.insert(name.to_owned());

        string.push_str(name);
    }
}

async fn websocket_handler(
    ws: WebSocketUpgrade,
    State(state): State<Arc<AppState>>,
) -> impl IntoResponse {
    ws.on_upgrade(|socket| websocket(socket, state))
}

// This function deals with a single websocket connection, i.e., a single
// connected client / user, for which we will spawn two independent tasks (for
// receiving / sending chat messages).
async fn websocket(stream: WebSocket, state: Arc<AppState>) {
    // By splitting, we can send and receive at the same time.
    let (mut sender, mut receiver) = stream.split();

    // Username gets set in the receive loop, if it's valid.
    let mut username = String::new();
    // Loop until a text message is found.
    while let Some(Ok(message)) = receiver.next().await {
        if let Message::Text(name) = message {
            // If username that is sent by client is not taken, fill username string.
            check_username(&state, &mut username, &name);

            // If not empty we want to quit the loop else we want to quit function.
            if !username.is_empty() {
                break;
            } else {
                // Only send our client that username is taken.
                let _ = sender
                    .send(Message::Text(
                        serde_json::json!({
                            "message": "Username is taken."
                        })
                        .to_string(),
                    ))
                    .await;

                return;
            }
        }
    }

    // We subscribe *before* sending the "joined" message, so that we will also
    // display it to our client.
    let mut rx = state.tx.subscribe();

    // Now send the "joined" message to all subscribers.
    let msg = serde_json::json!({
        "message": format!("{username} joined.")
    })
    .to_string();
    tracing::debug!("{msg}");
    let _ = state.tx.send(msg);

    // Spawn the first task that will receive broadcast messages and send text
    // messages over the websocket to our client.
    let mut send_task = tokio::spawn(async move {
        while let Ok(msg) = rx.recv().await {
            // In any websocket error, break loop.
            if sender.send(Message::Text(msg)).await.is_err() {
                break;
            }
        }
    });

    // Clone things we want to pass (move) to the receiving task.
    let tx = state.tx.clone();
    let name = username.clone();

    // Spawn a task that takes messages from the websocket, prepends the user
    // name, and sends them to all broadcast subscribers.
    let mut recv_task = tokio::spawn(async move {
        while let Some(Ok(Message::Text(text))) = receiver.next().await {
            // Add username before message.
            let _ = tx.send(
                serde_json::json!({
                    "name": name,
                    "message": text
                })
                .to_string(),
            );
        }
    });

    // If any one of the tasks run to completion, we abort the other.
    tokio::select! {
        _ = (&mut send_task) => recv_task.abort(),
        _ = (&mut recv_task) => send_task.abort(),
    };

    // Send "user left" message (similar to "joined" above).
    let msg = serde_json::json!({
        "message": format!("{username} left.")
    })
    .to_string();
    tracing::debug!("{msg}");
    let _ = state.tx.send(msg);

    // Remove username from map so new clients can take it again.
    state.user_set.lock().unwrap().remove(&username);
}

#[tokio::main]
async fn main() {
    let flag = Arc::new(std::sync::Mutex::new(false));

    // Set up application state for use with with_state().
    let user_set = Mutex::new(HashSet::new());
    let (tx, _rx) = broadcast::channel(100);

    let app_state = Arc::new(AppState { user_set, tx });

    tokio::spawn(async move {
        let cors = CorsLayer::new()
            .allow_methods([Method::GET, Method::POST])
            .allow_headers(Any)
            .allow_origin(Any);

        // Build our middleware stack
        let middleware = ServiceBuilder::new()
        // Add high level tracing/logging to all requests
        .layer(
            TraceLayer::new_for_http()
                .on_body_chunk(|chunk: &Bytes, latency: Duration, _: &tracing::Span| {
                    tracing::trace!(size_bytes = chunk.len(), latency = ?latency, "sending body chunk")
                })
                .make_span_with(DefaultMakeSpan::new().include_headers(true))
                .on_response(DefaultOnResponse::new().include_headers(true).latency_unit(LatencyUnit::Micros)),
        )
        // Set a timeout
        .layer(TimeoutLayer::new(Duration::from_secs(10)))
        .layer(cors)
        // Compress responses
        .compression();

        let app = Router::new()
            .route("/ws", get(websocket_handler))
            .layer(middleware)
            .with_state(app_state.clone());

        let server = axum::serve(
            TcpListener::bind("0.0.0.0:42068")
                .await
                .expect("bind error"),
            app.into_make_service(),
        );

        server.await.unwrap();
    });

    let app_builder = handlers::setup_handlers(); // Setup handlers

    app_builder
        .setup(move |app| {
            setup::setup_app(app, flag).expect("error while setting up the app");
            // println!("The app is being served on http://localhost:{}", port);
            Ok(())
        })
        .plugin(tauri_plugin_os::init())
        .plugin(tauri_plugin_shell::init())
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_fs::init())
        .plugin(tauri_plugin_autostart::init(
            tauri_plugin_autostart::MacosLauncher::LaunchAgent,
            Some(["--hidden", "-s"].to_vec()),
        ))
        .plugin(tauri_plugin_localhost::Builder::new(42069).build())
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
