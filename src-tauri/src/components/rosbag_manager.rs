use nix::sys::signal::{kill, Signal};
use nix::unistd::Pid;
use once_cell::sync::Lazy;
use serde::Deserialize;
use std::process::Stdio;
use std::sync::{Arc, Mutex};
use tauri::Wry;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command;

// Global reference to the Rosbag process
pub static ROSBAG_PROCESS: Lazy<Arc<Mutex<Option<tokio::process::Child>>>> =
    Lazy::new(|| Arc::new(Mutex::new(None)));
pub static ROSBAG_COMMAND_TOBE_KILLED: Lazy<Arc<Mutex<Option<String>>>> =
    Lazy::new(|| Arc::new(Mutex::new(None)));

// Define the Flag and Value enums
#[derive(Debug, Deserialize)]
pub struct Flag {
    arg: String,
    value: Value,
}

#[derive(Debug, Deserialize)]
#[serde(untagged)]
enum Value {
    String(String),
    Bool(bool),
}

#[tauri::command]
pub async fn play_rosbag(
    window: tauri::Window<Wry>,
    path: String,
    autoware_path: String,
    extra_workspaces: Vec<String>,
    flags: Vec<Flag>,
) {
    let window_clone = window.clone();

    // Construct the command string dynamically based on the provided flags
    let mut cmd_flags = String::new();
    for flag in &flags {
        match &flag.value {
            Value::Bool(b) => {
                if *b {
                    cmd_flags.push_str(&format!(" {} ", flag.arg));
                }
            }
            Value::String(s) => {
                if s.is_empty() {
                    continue;
                }
                cmd_flags.push_str(&format!(" {} {} ", flag.arg, s));
            }
        }
    }

    let mut source_extra_workspaces_str = String::new();
    for workspace in extra_workspaces {
        source_extra_workspaces_str
            .push_str(&format!("source {}/install/setup.bash && ", workspace));
    }

    // Construct the full command string
    let cmd_str = format!(
        "source /opt/ros/humble/setup.bash && \
         source {}/install/setup.bash && \
         {} \
         ros2 bag play {} {}",
        autoware_path, source_extra_workspaces_str, path, cmd_flags
    );

    println!("Rosbag command: {}", cmd_str.clone());

    // Execute the command with stdin piped
    let mut child = Command::new("bash")
        .arg("-c")
        .arg(&cmd_str)
        .stderr(Stdio::piped())
        .spawn()
        .expect("Failed to start rosbag play");

    let stdout = child
        .stderr
        .take()
        .expect("child did not have a handle to stdout");

    window.emit("rosbag-started", "Rosbag started").unwrap();

    {
        // Store the child process reference
        let mut rosbag_handle = ROSBAG_PROCESS.lock().unwrap();
        *rosbag_handle = Some(child);
    }

    {
        // set the command to be killed
        let mut roscmdtobekilledhandle = ROSBAG_COMMAND_TOBE_KILLED.lock().unwrap();
        let str_tobe_saved = format!("ros2 bag play {} {}", path, cmd_flags);
        *roscmdtobekilledhandle = Some(str_tobe_saved);
    }

    // // Spawn a new thread to read the stdout
    let reader = BufReader::new(stdout);
    let mut lines = reader.lines();

    while let Ok(Some(line)) = lines.next_line().await {
        println!("{}", line);

        let status = if line.contains("Resuming") {
            "Playing"
        } else if line.contains("Pausing") {
            "Pausing"
        } else {
            "unknown"
        };

        window
            .emit("rosbag-status", status)
            .unwrap_or_else(|err| eprintln!("Failed to emit to frontend: {}", err));

        if line.contains("[ERROR]") {
            println!("Rosbag ERROR: {}", line);
        }
    }

    println!("Rosbag play ended");
    window_clone.emit("rosbag-ended", "Rosbag ended").unwrap();
}

#[tauri::command]
pub async fn toggle_pause_state() -> Result<(), String> {
    println!("Sending toggle pause/play to rosbag");
    Command::new("bash")
        .arg("-c")
        .arg(
            "source /opt/ros/humble/setup.bash && \
        ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused",
        )
        .status()
        .await
        .expect("Failed to toggle pause/play rosbag");

    Ok(())
}

// Function to stop rosbag play
#[tauri::command]
pub async fn stop_rosbag_play(window: tauri::Window<Wry>) -> String {
    println!("Checking child process...");
    let child_option = ROSBAG_PROCESS.lock().unwrap().take();
    let roscmdtobekilled_option = ROSBAG_COMMAND_TOBE_KILLED.lock().unwrap().take();

    if let Some(cmd_str) = roscmdtobekilled_option {
        // run pkill -f cmd_str
        Command::new("bash")
            .arg("-c")
            .arg(format!("pkill -f \"{}\"", cmd_str))
            .status()
            .await
            .expect("Failed to stop rosbag play");
    } else {
        println!("No rosbag command found.");
    }

    if let Some(mut child) = child_option {
        // Check if the child process is still running
        if let Ok(Some(_)) = child.try_wait() {
            println!("Child process has already exited.");
        } else {
            // Try to gracefully terminate the process
            let _ = child.kill().await;

            // Safely attempt to get the process ID
            if let Some(pid) = child.id() {
                let _ = kill(Pid::from_raw(pid as i32), Signal::SIGKILL);
            } else {
                println!("Failed to get child process ID");
            }
        }

        window.emit("rosbag-stopped", "Rosbag stopped").unwrap();
        println!("Rosbag play stopped");
        return "process_stopped".to_string();
    } else {
        println!("No child process found.");
        return "no_process_found".to_string();
    }
}

#[tauri::command]
pub async fn set_rosbag_playback_rate(rate: f64) -> Result<(), String> {
    // Construct the command string to set the playback rate
    let cmd_str = format!(
        "source /opt/ros/humble/setup.bash && \
         ros2 service call /rosbag2_player/set_rate rosbag2_interfaces/srv/SetRate \"rate: {}\"",
        rate
    );

    println!("Setting rosbag rate: {}", cmd_str);

    // Execute the command
    let status = Command::new("bash").arg("-c").arg(&cmd_str).status().await;

    match status {
        Ok(_) => Ok(()),
        Err(e) => Err(format!("Failed to set rosbag playback rate: {}", e)),
    }
}

fn to_snake_case(s: &str) -> String {
    s.to_lowercase().replace(" ", "_")
}

#[tauri::command]
pub async fn get_rosbag_info(
    path: String,
    autoware_path: String,
    extra_workspaces: Vec<String>,
) -> Result<serde_json::Value, String> {
    // Construct the command string to set the playback rate
    let mut source_extra_workspaces_str = String::new();
    for workspace in extra_workspaces {
        source_extra_workspaces_str
            .push_str(&format!("source {}/install/setup.bash && ", workspace));
    }
    let cmd_str = format!(
        "source /opt/ros/humble/setup.bash && \
         source {}/install/setup.bash && \
         {} \
         ros2 bag info {}",
        autoware_path, source_extra_workspaces_str, path
    );

    // println!("Getting rosbag info: {}", cmd_str);

    // Execute the command
    let output = Command::new("bash")
        .arg("-c")
        .arg(&cmd_str)
        .output()
        .await
        .expect("Failed to get rosbag info");

    let stdout = String::from_utf8(output.stdout).unwrap();

    let mut info_object = serde_json::Map::new();
    let mut lines = stdout.lines().skip(2).peekable(); // Convert to peekable iterator

    while let Some(line) = lines.next() {
        if line.is_empty() || line.contains("closing.") {
            continue;
        }

        let mut arg_value = line.splitn(2, ":");
        let arg = arg_value.next().unwrap().trim();
        let value = arg_value.next().unwrap_or("").trim();

        let snake_case_arg = to_snake_case(arg); // Convert the key to snake_case

        if arg == "Topic information" {
            let mut topics = Vec::new();
            while lines.peek().is_some()
                && !lines.peek().unwrap().is_empty()
                && !lines.peek().unwrap().contains("closing.")
            {
                let topic_line = lines.next().unwrap();
                let topic_details: serde_json::Map<String, serde_json::Value> = topic_line
                    .split("|")
                    .filter_map(|s| {
                        let mut kv = s.splitn(2, ":");
                        let k = to_snake_case(kv.next()?.trim()); // Convert the key inside Topic information to snake_case
                        let v = kv.next().unwrap_or("").trim().to_string();
                        Some((k, serde_json::Value::String(v)))
                    })
                    .collect();
                topics.push(serde_json::Value::Object(topic_details));
            }
            info_object.insert(snake_case_arg, serde_json::Value::Array(topics));
        } else {
            info_object.insert(snake_case_arg, serde_json::Value::String(value.to_string()));
        }
    }

    Ok(serde_json::Value::Object(info_object))
}
