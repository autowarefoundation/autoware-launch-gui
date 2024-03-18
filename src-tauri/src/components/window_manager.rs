use std::sync::{Arc, Mutex};
use std::time::Duration;
use tauri::Manager;
use tokio::time::sleep;

pub async fn manage_splash_window(app: tauri::AppHandle, flag: Arc<Mutex<bool>>) {
    let splash_window = app
        .get_webview_window("splash")
        .expect("Failed to find splash window");

    // Assuming the splash window is initially visible, wait for the signal to close it
    loop {
        sleep(Duration::from_millis(100)).await;
        let flag_guard = flag.lock().unwrap();
        if *flag_guard {
            break;
        }
    }

    splash_window
        .close()
        .expect("Failed to close splash window");
}

pub async fn setup_and_show_main_window(app: tauri::AppHandle, flag: Arc<Mutex<bool>>) {
    let main_window = app
        .get_webview_window("main")
        .expect("Failed to find main window");
    main_window.hide().unwrap();

    sleep(Duration::from_secs(2)).await;

    // After setup, show the main window
    main_window.show().expect("Failed to show main window");
    main_window
        .set_focus()
        .expect("Failed to focus main window");

    main_window.center().expect("Failed to center main window");

    // Signal to close the splash window
    let mut flag_guard = flag.lock().unwrap();
    *flag_guard = true;
}

pub async fn setup_and_hide_main_window(app: tauri::AppHandle, flag: Arc<Mutex<bool>>) {
    let main_window = app
        .get_webview_window("main")
        .expect("Failed to find main window");
    main_window.hide().unwrap();

    sleep(Duration::from_secs(2)).await;

    // Signal to close the splash window
    let mut flag_guard = flag.lock().unwrap();
    *flag_guard = true;
}

pub fn setup_window_listener(app: &tauri::App) {
    // if user closes the main window, we prevent the app from closing and run the kill_autoware_process command before closing then close the app
    let main_window = app.get_webview_window("main").unwrap();
    let main_clone = main_window.clone();

    main_window.on_window_event(move |event| match event {
        tauri::WindowEvent::CloseRequested { api, .. } => {
            // TODO: To be updated after tauri-controls is updated
            println!("Closing the app...");
            main_clone
                .emit("close_requested", "close requested")
                .unwrap();
            api.prevent_close();
        }
        _ => {
            // do nothing
        }
    });
}
