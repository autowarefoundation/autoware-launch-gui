#![cfg_attr(
    all(not(debug_assertions), target_os = "windows"),
    windows_subsystem = "windows"
)]

use std::sync::{Arc, Mutex};
use tauri::Manager;

mod components; // Import all the components
mod handlers;
mod setup;

#[derive(Clone, serde::Serialize)]
struct Payload {
    args: Vec<String>,
    cwd: String,
}

#[tokio::main]
async fn main() {
    let flag = Arc::new(Mutex::new(false));

    let app_builder = handlers::setup_handlers(); // Setup handlers

    app_builder
        .setup(move |app| {
            setup::setup_app(app, flag).expect("error while setting up the app");
            Ok(())
        })
        .plugin(tauri_plugin_os::init())
        .plugin(tauri_plugin_shell::init())
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_fs::init())
        .plugin(tauri_plugin_autostart::init(
            tauri_plugin_autostart::MacosLauncher::LaunchAgent,
            Some(["--hidden"].to_vec()),
        ))
        .plugin(tauri_plugin_single_instance::init(|app, argv, cwd| {
            println!("You are trying to open a new instance with these args: {:?} and cwd: {:?}, but an instance is already running. Ignoring.", argv, cwd);

            app.emit("single-instance", Payload { args: argv, cwd })
                .unwrap();
        }))
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
