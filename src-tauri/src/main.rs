#![cfg_attr(
    all(not(debug_assertions), target_os = "windows"),
    windows_subsystem = "windows"
)]

use std::sync::{Arc, Mutex};
use std::time::Duration;
use tauri::Manager;
use tauri_plugin_fs::FsExt;
use tokio::process::Command as TokioCommand;
use tokio::time::sleep;

mod components; // Assuming autoware_manager.rs is in the same directory as main.rs

use components::autoware_manager::{
    autoware_installed_packages, get_topics, kill_autoware_process, launch_autoware,
};
use components::json_profile::{load_profile, save_profile};
use components::ros2_cmd_manager::{
    kill_bag_record, kill_topic_echo, start_bag_record, start_topic_echo,
};
use components::rosbag_manager::{
    get_rosbag_info, play_rosbag, set_rosbag_playback_rate, stop_rosbag_play, toggle_pause_state,
};
use components::xml_parse::parse_and_send_xml;
use components::yaml_edit::{find_yaml_files, parse_yaml, save_edits_yaml};

#[derive(serde::Serialize)]
struct SystemInfo {
    cpus_usage: Vec<f32>, // New field for individual CPU usages
    cpu_usage: f32,
    top_processes: Vec<ProcessInfo>,
    memory_used_percentage: f32,
    memory_usage: String, // New field to store memory usage in "xGB/yGB" format
    autoware_processes: Vec<String>,
}

#[derive(serde::Serialize)]
struct ProcessInfo {
    name: String,
    cpu_usage: f32,
}

#[tauri::command(async)]
async fn get_system_info(autoware_path: Option<String>) -> Result<SystemInfo, String> {
    // Get top 5 CPU-consuming processes
    let top_processes_output = TokioCommand::new("ps")
        .args(&["-eo", "pid,pcpu,comm", "--sort=-pcpu"])
        .output()
        .await
        .map_err(|e| e.to_string())?;

    let top_processes_str =
        String::from_utf8(top_processes_output.stdout).map_err(|e| e.to_string())?;
    let lines: Vec<&str> = top_processes_str.lines().collect();

    let mut top_processes = Vec::new();
    // can we get only the processes with autoware in the name?
    for line in lines.iter().skip(1).take(5) {
        // skip the header and take top 5
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() == 3 {
            let cpu_usage: f32 = parts[1].parse().unwrap_or(0.0);
            top_processes.push(ProcessInfo {
                name: parts[2].to_string(),
                cpu_usage,
            });
        }
    }

    let mut autoware_processes = Vec::new();
    // Get all Autoware processes
    let autoware_processes_output = TokioCommand::new("ps")
        .args(&["-ef"])
        .output()
        .await
        .map_err(|e| e.to_string())?;

    let autoware_processes_str =
        String::from_utf8(autoware_processes_output.stdout).map_err(|e| e.to_string())?;
    let lines: Vec<&str> = autoware_processes_str.lines().collect();
    // we should filter out the processes that are not coming from "home/khalil/autoware"
    // if there is no autoware path, we should skip this for loop

    for line in lines.iter().skip(1) {
        // skip the header and take top 5
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() > 7 {
            let process_name = parts[7].to_string();
            if let Some(path) = autoware_path.clone() {
                if process_name.contains(&path) || process_name.contains("opt/ros") {
                    autoware_processes.push(process_name);
                }
            }
        }
    }

    // Get memory details using `free` command
    let memory_output = TokioCommand::new("free")
        .output()
        .await
        .map_err(|e| e.to_string())?;

    let memory_str = String::from_utf8(memory_output.stdout).map_err(|e| e.to_string())?;
    let memory_lines: Vec<&str> = memory_str.lines().collect();
    let memory_parts: Vec<&str> = memory_lines[1].split_whitespace().collect();
    let total_memory: f32 = memory_parts[1].parse().unwrap_or(1.0);
    let used_memory: f32 = memory_parts[2].parse().unwrap_or(0.0);
    let memory_used_percentage = (used_memory / total_memory) * 100.0;

    // Convert memory values from KB to GB
    let total_memory_gb = total_memory / 1_048_576.0; // 1 GB = 1_048_576 KB
    let used_memory_gb = used_memory / 1_048_576.0;

    // Format memory usage as "xGB/yGB"
    let memory_usage = format!("{:.2}GB/{:.2}GB", used_memory_gb, total_memory_gb);

    // For CPU usage, we can use `vmstat` or `mpstat`. Here, I'll use `vmstat`.
    let cpu_output = TokioCommand::new("vmstat")
        .args(&["1", "2"])
        .output()
        .await
        .map_err(|e| e.to_string())?;

    let cpu_str = String::from_utf8(cpu_output.stdout).map_err(|e| e.to_string())?;
    let cpu_lines: Vec<&str> = cpu_str.lines().collect();
    let cpu_parts: Vec<&str> = cpu_lines.last().unwrap().split_whitespace().collect();
    let idle_cpu: f32 = cpu_parts[14].parse().unwrap_or(100.0);
    let cpu_usage = 100.0 - idle_cpu;
    // Get individual CPU usages using `mpstat`
    // Get individual CPU usages using `mpstat`
    let cpu_cores_output = TokioCommand::new("mpstat")
        .args(&["-P", "ALL", "1", "1"])
        .output()
        .await
        .map_err(|e| e.to_string())?;

    let cpu_cores_str = String::from_utf8(cpu_cores_output.stdout).map_err(|e| e.to_string())?;
    let cpu_lines: Vec<&str> = cpu_cores_str.lines().collect();

    let mut cpus_usage = Vec::new();
    for line in cpu_lines.iter() {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() > 3 && parts[1].chars().all(char::is_numeric) {
            // Check if the second column is a number (CPU core number)
            let idle_cpu: f32 = parts[11].parse().unwrap_or(100.0);
            cpus_usage.push(100.0 - idle_cpu);
        }
    }

    Ok(SystemInfo {
        cpus_usage,
        cpu_usage,
        top_processes,
        memory_used_percentage,
        memory_usage, // Populate the new field
        autoware_processes,
    })
}

#[tauri::command]
fn files_in_dir(map_path: String) -> Vec<String> {
    let mut files = Vec::new();

    // expect the path to be a directory and not a file and that it might have $HOME in it
    // let's replace $HOME with the actual home directory during runtime
    let path = map_path.replace("$HOME", dirs::home_dir().unwrap().to_str().unwrap());

    let paths = std::fs::read_dir(path);

    // handle the error if the path is not a directory by returning an empty vector instead of crashing
    if let Ok(paths) = paths {
        for path in paths {
            if let Ok(path) = path {
                if let Ok(file_name) = path.file_name().into_string() {
                    files.push(file_name);
                }
            }
        }
    }

    return files;
}

// This is the main function

async fn show_main_window(app: tauri::AppHandle, flag: Arc<Mutex<bool>>) {
    // get the main window and show it

    let main_window = app.get_window("main").unwrap();
    main_window.hide().unwrap();

    sleep(Duration::from_secs(2)).await;

    main_window.show().unwrap();

    main_window.set_focus().unwrap();
    main_window.center().unwrap();
    main_window.emit("build_started", "Loading...").unwrap();

    // signal to close the splash window
    let mut flag = flag.lock().unwrap();
    *flag = true;
}

#[tokio::main]
async fn main() {
    let flag = Arc::new(Mutex::new(false));
    tauri::Builder::default()
        .invoke_handler(tauri::generate_handler![
            get_system_info,
            launch_autoware,
            kill_autoware_process,
            parse_and_send_xml,
            files_in_dir,
            autoware_installed_packages,
            save_edits_yaml,
            find_yaml_files,
            parse_yaml,
            save_profile,
            load_profile,
            play_rosbag,
            toggle_pause_state,
            stop_rosbag_play,
            set_rosbag_playback_rate,
            get_rosbag_info,
            get_topics,
            start_topic_echo,
            kill_topic_echo,
            start_bag_record,
            kill_bag_record,
        ])
        .setup(move |app| {
            let app_for_async = app.app_handle().clone();
            // app.fs_scope().allow_directory("$HOME/**/*", true).unwrap();
            // print out the fs scope
            println!(
                "{:?}",
                app.fs_scope().allowed_patterns().iter().collect::<Vec<_>>()
            );
            let flag_for_async = flag.clone();
            tokio::spawn(async move {
                show_main_window(app_for_async, flag_for_async).await;
            });

            let splash_window = app.get_window("splash").unwrap();
            let flag_for_thread = flag.clone();

            // periodically check the flag to close the splash window
            tokio::spawn(async move {
                loop {
                    tokio::time::sleep(Duration::from_millis(100)).await;
                    let flag = flag_for_thread.lock().unwrap();
                    if *flag {
                        splash_window.close().unwrap();
                        break;
                    }
                }
            });

            // if user closes the main window, we prevent the app from closing and run the kill_autoware_process command before closing then close the app
            let main_window = app.get_window("main").unwrap();
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
                tauri::WindowEvent::Resized(_) => {
                    // do nothing
                }
                tauri::WindowEvent::Moved(_) => {
                    // do nothing
                }
                tauri::WindowEvent::Destroyed => {
                    // do nothing
                }
                tauri::WindowEvent::Focused(_) => {
                    // do nothing
                }
                tauri::WindowEvent::ScaleFactorChanged { .. } => {
                    // do nothing
                }
                tauri::WindowEvent::FileDrop(_) => {
                    // do nothing
                }
                tauri::WindowEvent::ThemeChanged(_) => {
                    // do nothing
                }
                _ => {
                    // do nothing
                }
            });

            Ok(())
        })
        .plugin(tauri_plugin_app::init())
        .plugin(tauri_plugin_os::init())
        .plugin(tauri_plugin_shell::init())
        .plugin(tauri_plugin_window::init())
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_fs::init())
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
