use nix::unistd::Pid;
use once_cell::sync::Lazy;
use serde::Deserialize;
use std::process::Stdio;
use std::sync::{Arc, Mutex};
use tauri::Wry;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command;

// Global reference to the Autoware process
pub static AUTOWARE_PROCESS: Lazy<Arc<Mutex<Option<tokio::process::Child>>>> =
    Lazy::new(|| Arc::new(Mutex::new(None)));
pub static AUTOWARE_PIDS: Lazy<Arc<Mutex<Vec<i32>>>> =
    Lazy::new(|| Arc::new(Mutex::new(Vec::new())));

#[derive(Debug, Deserialize)]
pub struct ArgsToLaunch {
    arg: String,
    value: String,
}

#[tauri::command]
pub async fn launch_autoware(
    window: tauri::Window<Wry>,
    path: String,
    launch_file: String,
    args_to_launch: Vec<ArgsToLaunch>,
) {
    let autoware_process = Arc::clone(&AUTOWARE_PROCESS);
    let window_clone = window.clone();
    let window_clone_err = window.clone();

    // Construct the command string dynamically based on the provided arguments
    let cmd_args = args_to_launch
        .iter()
        .map(|arg| format!("{}:={}", arg.arg, arg.value))
        .collect::<Vec<String>>()
        .join(" ");

    // Construct the full command string
    let cmd_str = format!(
        "source /opt/ros/humble/setup.bash && \
         source {}/install/setup.bash && \
         ros2 launch autoware_launch {} {}",
        path, launch_file, cmd_args
    );

    // Execute the command with stdout piped
    let mut child = Command::new("bash")
        .arg("-c")
        .arg(&cmd_str)
        .stdout(Stdio::piped())
        .spawn()
        .expect("Failed to start the Autoware process");

    let stdout = child
        .stdout
        .take()
        .expect("child did not have a handle to stdout");

    window.emit("autoware-started", "Autoware started").unwrap();

    // Handle the stdout in an asynchronous task
    tokio::spawn(async move {
        let pids = Arc::clone(&AUTOWARE_PIDS);
        let reader = BufReader::new(stdout);
        let mut lines = reader.lines();

        while let Ok(Some(line)) = lines.next_line().await {
            println!("Autoware: {}", line);
            window_clone.emit("autoware-output", line.clone()).unwrap();

            if line.contains("[ERROR]") {
                println!("Autoware ERROR: {}", line);
                if line.contains("Caught exception") {
                    let mut pids_lock = pids.lock().unwrap();
                    pids_lock.clear();
                    window_clone_err
                        .emit("pids-cleared", "pid list cleared")
                        .unwrap();
                    window_clone_err
                        .emit("package-not-found", "package not found")
                        .unwrap();
                }
            } else if line.contains("process started with pid") {
                if let Some(start) = line.rfind('[') {
                    if let Some(end) = line.rfind(']') {
                        let pid_str = &line[start + 1..end];
                        if let Ok(pid) = pid_str.parse::<i32>() {
                            let mut pids_lock = pids.lock().unwrap();
                            pids_lock.push(pid);
                        }
                    }
                }
            } else if line.contains("All log files can be found below") {
                if let Some(start) = line.rfind('-') {
                    let pid_str = &line[start + 1..];
                    if let Ok(pid) = pid_str.parse::<i32>() {
                        let mut pids_lock = pids.lock().unwrap();
                        pids_lock.push(pid);
                    }
                }
            }

            let pids_lock = pids.lock().unwrap();
            let pids_len = pids_lock.len();
            window_clone.emit("pids_len", pids_len).unwrap();
        }

        window_clone
            .emit("autoware-ended", "Autoware ended")
            .unwrap();
    });

    // Store the child process reference
    let mut autoware_handle = autoware_process.lock().unwrap();
    *autoware_handle = Some(child);
    println!(
        "Autoware process started, PID: {}",
        autoware_handle.as_ref().unwrap().id().unwrap()
    );
}

#[tauri::command]
pub async fn kill_autoware_process(window: tauri::Window<Wry>) -> Result<(), String> {
    let child_opt = {
        let mut autoware_handle = match AUTOWARE_PROCESS.lock() {
            Ok(handle) => handle,
            Err(poisoned) => poisoned.into_inner(),
        };
        autoware_handle.take()
    };

    if let Some(mut child) = child_opt {
        println!("Killing the Autoware process... {}", child.id().unwrap());

        // Kill the main bash process
        match child.kill().await {
            Ok(_) => println!("Successfully killed the main bash process"),
            Err(e) => eprintln!("Failed to kill the main bash process: {}", e),
        }

        // Wait for the child process to finish
        let _ = child.wait().await;

        // Kill the other Autoware processes
        let pids = {
            let pids_lock = match AUTOWARE_PIDS.lock() {
                Ok(lock) => lock,
                Err(poisoned) => poisoned.into_inner(),
            };
            pids_lock.clone()
        };

        for &pid in &pids {
            println!("Killing process with PID {}", pid);
            if let Err(e) =
                nix::sys::signal::kill(Pid::from_raw(pid), nix::sys::signal::Signal::SIGKILL)
            {
                eprintln!("Failed to kill process with PID {}: {}", pid, e);
            }
        }

        // Clear the list of PIDs
        {
            let mut pids_lock = match AUTOWARE_PIDS.lock() {
                Ok(lock) => lock,
                Err(poisoned) => poisoned.into_inner(),
            };
            pids_lock.clear();
        }

        // Emit an event to the frontend to enable the button
        window.emit("pids-cleared", "pid list cleared").unwrap();
    } else {
        eprintln!("No Autoware process found");
    }

    Ok(())
}

#[tauri::command]
pub async fn autoware_installed_packages(autoware_path: String) -> Result<Vec<String>, String> {
    // Get the list of installed packages by parsing the names of the folders in the install folder
    let packages = std::fs::read_dir(format!("{}/install", autoware_path))
        .map_err(|e| e.to_string())?
        .filter_map(|entry| {
            let entry = entry.ok()?;
            if entry.file_type().ok()?.is_dir() {
                entry.file_name().into_string().ok()
            } else {
                None
            }
        })
        .collect::<Vec<String>>();

    Ok(packages)
}

#[tauri::command]
pub async fn get_topics() -> Result<Vec<String>, String> {
    let mut topics = Vec::new();

    let cmd_str = format!(
        "source /opt/ros/humble/setup.bash && \
         ros2 topic list -t"
    );

    let output = Command::new("bash")
        .arg("-c")
        .arg(&cmd_str)
        .output()
        .await
        .map_err(|e| e.to_string())?;

    let stdout = String::from_utf8(output.stdout).unwrap();
    let lines = stdout.split("\n");
    for line in lines {
        if line != "" {
            let topic = line.split(" ").collect::<Vec<&str>>()[0];
            let type_ = line.split(" ").collect::<Vec<&str>>()[1];
            let type_ = type_.trim_start_matches('[').trim_end_matches(']');

            topics.push(format!("{} {}", topic, type_));
        }
    }
    Ok(topics)
}

#[tauri::command]
pub async fn get_services() -> Result<Vec<String>, String> {
    let mut services = Vec::new();

    let cmd_str = format!(
        "source /opt/ros/humble/setup.bash && \
         ros2 service list -t"
    );

    let output = Command::new("bash")
        .arg("-c")
        .arg(&cmd_str)
        .output()
        .await
        .map_err(|e| e.to_string())?;

    let stdout = String::from_utf8(output.stdout).unwrap();
    let lines = stdout.split("\n");
    for line in lines {
        if line != "" {
            let service = line.split(" ").collect::<Vec<&str>>()[0];
            let type_ = line.split(" ").collect::<Vec<&str>>()[1];
            let type_ = type_.trim_start_matches('[').trim_end_matches(']');

            services.push(format!("{} {}", service, type_));
        }
    }
    Ok(services)
}
