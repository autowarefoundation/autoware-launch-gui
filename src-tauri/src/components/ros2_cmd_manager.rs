use once_cell::sync::Lazy;
use serde::Deserialize;
use std::process::Stdio;
use std::sync::{Arc, Mutex};
use tauri::Wry;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command;

// Global reference to the ros2 topic echo process
pub static ROS2_ECHO_PROCESS: Lazy<Arc<Mutex<Option<tokio::process::Child>>>> =
    Lazy::new(|| Arc::new(Mutex::new(None)));
pub static RO2_RECORD_PROCESS: Lazy<Arc<Mutex<Option<tokio::process::Child>>>> =
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
pub async fn start_topic_echo(
    topic: String,
    topic_type: String,
    autoware_path: String,
    window: tauri::Window<Wry>,
) -> Result<(), String> {
    let window_clone = window.clone();

    tokio::task::spawn(async move {
        let cmd_str = format!(
            "source /opt/ros/humble/setup.bash && \
             source {}/install/setup.bash && \
             ros2 topic echo {} {}",
            autoware_path, topic, topic_type
        );

        println!("Running command: {}", cmd_str);
        let output = Command::new("bash")
            .arg("-c")
            .arg(&cmd_str)
            .stdout(Stdio::piped())
            .spawn()
            .map_err(|e| e.to_string());

        let mut child = match output {
            Ok(child) => child,
            Err(err) => return eprintln!("Failed to start topic echo: {}", err),
        };

        let stdout = child
            .stdout
            .take()
            .expect("child did not have a handle to stdout");

        {
            let mut process = ROS2_ECHO_PROCESS.lock().unwrap();
            *process = Some(child);
        }

        let reader = BufReader::new(stdout);
        let mut lines = reader.lines();

        while let Ok(line) = lines.next_line().await {
            if let Some(line_content) = line {
                println!("Topic echo: {}", line_content);
                window_clone
                    .emit("ros2-topic-echo-output", line_content)
                    .unwrap_or_else(|err| eprintln!("Failed to emit to frontend: {}", err));
            }
        }
    });

    Ok(())
}

// either we give a list of topics to record or we record all topics if the list is empty
// and of course we also give a name to the bag file as well as flags to be added to the command
#[tauri::command]
pub async fn start_bag_record(
    topics: Vec<String>,
    autoware_path: String,
    flags: Vec<Flag>,
    window: tauri::Window<Wry>,
) -> Result<(), String> {
    let window_clone = window.clone();
    // Check if --all flag is set to true
    let all_flag = flags
        .iter()
        .any(|flag| flag.arg == "--all" && matches!(flag.value, Value::Bool(true)));

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

    let mut cmd_str = format!(
        "source /opt/ros/humble/setup.bash && \
         source {}/install/setup.bash && \
         cd ~ && \
         ros2 bag record",
        autoware_path
    );

    if !all_flag && !topics.is_empty() {
        cmd_str.push_str(" ");
        cmd_str.push_str(&topics.join(" "));
    }

    cmd_str.push_str(&cmd_flags);

    println!("Running command: {}", cmd_str);

    let output = Command::new("bash")
        .arg("-c")
        .arg(&cmd_str)
        .stdout(Stdio::piped())
        .spawn()
        .map_err(|e| e.to_string());

    let mut child = match output {
        Ok(child) => child,
        Err(err) => {
            eprintln!("Failed to start bag record: {}", err);
            return Err(err.to_string());
        }
    };

    let stdout = child
        .stdout
        .take()
        .expect("child did not have a handle to stdout");

    {
        let mut process = RO2_RECORD_PROCESS.lock().unwrap();
        *process = Some(child);
    }

    let reader = BufReader::new(stdout);

    let mut lines = reader.lines();

    while let Ok(Some(line)) = lines.next_line().await {
        println!("Bag record: {}", line);
        if line == "" {
            continue;
        }
        window_clone
            .emit("ros2-bag-record-output", line)
            .unwrap_or_else(|err| eprintln!("Failed to emit to frontend: {}", err));
    }

    Ok(())
}

#[tauri::command]
pub async fn kill_topic_echo(window: tauri::Window<Wry>) -> Result<(), String> {
    let child_opt = {
        let mut process_handle = match ROS2_ECHO_PROCESS.lock() {
            Ok(handle) => handle,
            Err(poisoned) => poisoned.into_inner(),
        };
        process_handle.take()
    };

    if let Some(mut child) = child_opt {
        println!("Killing the topic echo process... {}", child.id().unwrap());

        // Kill the main process
        match child.kill().await {
            Ok(_) => println!("Successfully killed the topic echo process"),
            Err(e) => eprintln!("Failed to kill the topic echo process: {}", e),
        }

        // Wait for the child process to finish
        let _ = child.wait().await;

        // Emit an event to the frontend
        window
            .emit("topic-echo-killed", "Topic echo process killed")
            .unwrap();
    } else {
        eprintln!("No topic echo process found");
    }

    Ok(())
}

#[tauri::command]
pub async fn kill_bag_record(window: tauri::Window<Wry>) -> Result<(), String> {
    let child_opt = {
        let mut process_handle = match RO2_RECORD_PROCESS.lock() {
            Ok(handle) => handle,
            Err(poisoned) => poisoned.into_inner(),
        };
        process_handle.take()
    };

    if let Some(mut child) = child_opt {
        println!("Killing the bag record process... {}", child.id().unwrap());

        // Kill the main process
        match child.kill().await {
            Ok(_) => println!("Successfully killed the bag record process"),
            Err(e) => eprintln!("Failed to kill the bag record process: {}", e),
        }

        // Wait for the child process to finish
        let _ = child.wait().await;

        // Emit an event to the frontend
        window
            .emit("bag-record-killed", "Bag record process killed")
            .unwrap();
    } else {
        eprintln!("No bag record process found");
    }

    Ok(())
}
