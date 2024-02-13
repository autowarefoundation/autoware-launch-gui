use tauri::menu::MenuBuilder;
use tauri::Manager;

pub fn setup_tray(app: &tauri::AppHandle) -> Result<(), tauri::Error> {
    let icon = app.default_window_icon().unwrap();

    let menu = MenuBuilder::new(app)
        .text("toggle", "Show/Hide")
        .separator()
        .text("start_autoware", "Start Autoware")
        .separator()
        .text("kill_autoware", "Kill Autoware")
        .separator()
        .text("quit", "Quit")
        .build()?;

    // get access to the tray from app state
    let tray = app.tray();
    match tray {
        Some(tray) => Ok({
            let _ = tray.set_menu(Some(menu));
            let _ = tray.set_icon(Some(icon.clone()));

            tray.on_menu_event(move |app, event| match event.id().0.as_str() {
                "toggle" => {
                    let main_window = app.get_webview_window("main").unwrap();
                    if main_window.is_visible().unwrap() {
                        main_window.hide().unwrap();
                    } else {
                        main_window.show().unwrap();
                    }
                }
                "start_autoware" => {
                    let main_window = app.get_webview_window("main").unwrap();
                    main_window
                        .emit("start_autoware_on_app_start", true)
                        .unwrap();
                }

                "kill_autoware" => {
                    let main_window = app.get_webview_window("main").unwrap();
                    main_window
                        .emit("kill_autoware_through_tray", true)
                        .unwrap();
                }
                "quit" => {
                    let main_window = app.get_webview_window("main").unwrap();
                    main_window
                        .emit("close_requested", "close requested")
                        .unwrap();
                }
                _ => {
                    println!("Unknown menu item: {}", event.id().0);
                }
            });
        }),
        None => Ok({
            println!("No tray found");
        }),
    }
}
