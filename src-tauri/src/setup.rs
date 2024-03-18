use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use tauri::{App, Manager};

use crate::components;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(author,version,about,long_about=None)]
struct Args {
    /// Hide the main window on startup
    #[arg(long, action = clap::ArgAction::SetTrue)]
    hidden: bool,

    /// Launch autoware on startup
    #[arg(short,long, action = clap::ArgAction::SetTrue)]
    start_autoware: bool,
}

pub fn setup_app(app: &App, flag: Arc<Mutex<bool>>) -> Result<(), String> {
    let args = Args::parse();

    let app_for_async = app.app_handle().clone();
    let app_for_async2 = app.app_handle().clone();
    let flag_for_async = flag.clone();
    let window = app.app_handle().get_webview_window("main").unwrap();

    if args.hidden {
        tokio::spawn(async move {
            components::window_manager::setup_and_hide_main_window(
                app_for_async.clone(),
                flag_for_async.clone(),
            )
            .await;

            if args.start_autoware {
                println!("Starting Autoware in 3 seconds!");
                tokio::time::sleep(Duration::from_secs(3)).await;

                window
                    .clone()
                    .emit("start_autoware_on_app_start", args.start_autoware)
                    .unwrap();
            }
        });
        println!("Running app in hidden mode");
    } else {
        tokio::spawn(async move {
            components::window_manager::setup_and_show_main_window(
                app_for_async.clone(),
                flag_for_async.clone(),
            )
            .await;

            if args.start_autoware {
                println!("Starting Autoware in 3 seconds!");
                tokio::time::sleep(Duration::from_secs(3)).await;

                window
                    .clone()
                    .emit("start_autoware_on_app_start", args.start_autoware)
                    .unwrap();
            }
        });
        println!("Running app in gui mode");
    }

    let flag_for_thread = flag.clone();

    tokio::spawn(async move {
        components::window_manager::manage_splash_window(app_for_async2, flag_for_thread).await;
    });

    // setup the window listener
    components::window_manager::setup_window_listener(app);

    // setup the tray
    let _ = components::tray::setup_tray(app.app_handle());

    Ok(())
}
