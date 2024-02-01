use nix::unistd::Pid;
use once_cell::sync::Lazy;
use serde::Deserialize;
use serde_json::json;
use std::collections::HashMap;
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

pub static CALIBRATION_PROCESS: Lazy<Arc<Mutex<HashMap<String, tokio::process::Child>>>> =
    Lazy::new(|| Arc::new(Mutex::new(HashMap::new())));
pub static CALIBRATION_PIDS: Lazy<Arc<Mutex<HashMap<String, Vec<i32>>>>> =
    Lazy::new(|| Arc::new(Mutex::new(HashMap::new())));
pub static TOPIC_PUBLISH_PROCESS: Lazy<Arc<Mutex<Option<tokio::process::Child>>>> =
    Lazy::new(|| Arc::new(Mutex::new(None)));
pub static SERVICE_CALL_PROCESS: Lazy<Arc<Mutex<Option<tokio::process::Child>>>> =
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
    extra_workspaces: Vec<String>,
    window: tauri::Window<Wry>,
) -> Result<(), String> {
    let window_clone = window.clone();

    tokio::task::spawn(async move {
        let mut source_extra_workspaces_str = String::new();
        for workspace in extra_workspaces {
            source_extra_workspaces_str
                .push_str(&format!("source {}/install/setup.bash && ", workspace));
        }
        let cmd_str = format!(
            "source /opt/ros/humble/setup.bash && \
             source {}/install/setup.bash && \
             {} \
             ros2 topic echo {} {}",
            autoware_path, source_extra_workspaces_str, topic, topic_type
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
    extra_workspaces: Vec<String>,
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

    let mut source_extra_workspaces_str = String::new();
    for workspace in extra_workspaces {
        source_extra_workspaces_str
            .push_str(&format!("source {}/install/setup.bash && ", workspace));
    }

    let mut cmd_str = format!(
        "source /opt/ros/humble/setup.bash && \
         source {}/install/setup.bash && \
         {} \
         cd ~ && \
         ros2 bag record",
        autoware_path, source_extra_workspaces_str
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

// Calibration tools

#[tauri::command]
pub async fn launch_tool(
    tool: String,
    command: String,
    autoware_path: String,
    extra_workspaces: Vec<String>,
    window: tauri::Window<Wry>,
) -> Result<(), String> {
    let window_clone = window.clone();

    tokio::task::spawn(async move {
        let pids = Arc::clone(&CALIBRATION_PIDS);

        let mut source_extra_workspaces_str = String::new();
        for workspace in extra_workspaces {
            source_extra_workspaces_str
                .push_str(&format!("source {}/install/setup.bash && ", workspace));
        }

        let cmd_str = format!(
            "source /opt/ros/humble/setup.bash && \
             source {}/install/setup.bash && \
             {} \
             {}",
            autoware_path, source_extra_workspaces_str, command
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
            Err(err) => return eprintln!("Failed to start calibration tool: {}", err),
        };

        let stdout = child
            .stdout
            .take()
            .expect("child did not have a handle to stdout");

        {
            let mut process = CALIBRATION_PROCESS.lock().unwrap();
            process.insert(tool.clone(), child);
        }

        let reader = BufReader::new(stdout);
        let mut lines = reader.lines();

        while let Ok(line) = lines.next_line().await {
            if let Some(line_content) = line {
                let line_content_clone = line_content.clone();
                println!("Calibration tool: {}", line_content);
                let payload = json!({
                    "tool": tool.clone(),
                    "output": line_content
                });
                // let str_payload = match serde_json::to_string(&payload) {
                //     Ok(str) => str,
                //     Err(err) => {
                //         eprintln!("Failed to serialize payload: {}", err);
                //         continue;
                //     }
                // };
                window_clone
                    .emit("calibration-tool-output", payload)
                    .unwrap_or_else(|err| eprintln!("Failed to emit to frontend: {}", err));

                if line_content_clone.contains("process started with pid") {
                    if let Some(start) = line_content_clone.rfind('[') {
                        if let Some(end) = line_content_clone.rfind(']') {
                            let pid_str = &line_content_clone[start + 1..end];
                            if let Ok(pid) = pid_str.parse::<i32>() {
                                let mut pids_lock = pids.lock().unwrap();
                                if let Some(pids) = pids_lock.get_mut(&tool) {
                                    pids.push(pid);
                                } else {
                                    pids_lock.insert(tool.clone(), vec![pid]);
                                }
                            }
                        }
                    }
                } else if line_content_clone.contains("All log files can be found below") {
                    if let Some(start) = line_content_clone.rfind('-') {
                        let pid_str = &line_content_clone[start + 1..];
                        if let Ok(pid) = pid_str.parse::<i32>() {
                            let mut pids_lock = pids.lock().unwrap();
                            if let Some(pids) = pids_lock.get_mut(&tool) {
                                pids.push(pid);
                            } else {
                                pids_lock.insert(tool.clone(), vec![pid]);
                            }
                        }
                    }
                }
            }
        }
    });

    Ok(())
}

#[tauri::command]
pub async fn kill_calibration_tool(
    tool: String, // The identifier of the tool to kill
    window: tauri::Window<Wry>,
) -> Result<(), String> {
    // Retrieve and remove the child process from the map
    let child = {
        let mut process_map = CALIBRATION_PROCESS.lock().unwrap();
        process_map.remove(&tool)
    };

    // Kill the main process
    if let Some(mut child_process) = child {
        println!(
            "Killing the calibration tool process... {}",
            child_process.id().unwrap()
        );

        match child_process.kill().await {
            Ok(_) => println!("Successfully killed the calibration tool process"),
            Err(e) => eprintln!("Failed to kill the calibration tool process: {}", e),
        }

        let _ = child_process.wait().await;
    } else {
        eprintln!("No calibration tool process found for tool: {}", tool);
    }

    // Retrieve and remove the PIDs associated with the tool
    let pids = {
        let mut pids_map = CALIBRATION_PIDS.lock().unwrap();
        pids_map.remove(&tool)
    };

    // Kill any additional PIDs
    if let Some(pid_list) = pids {
        for &pid in &pid_list {
            println!("Killing process with PID {}", pid);
            if let Err(e) =
                nix::sys::signal::kill(Pid::from_raw(pid), nix::sys::signal::Signal::SIGKILL)
            {
                eprintln!("Failed to kill process with PID {}: {}", pid, e);
            }
        }
    } else {
        eprintln!("No additional PIDs found for tool: {}", tool);
    }

    window
        .emit(
            "calibration-tool-killed",
            format!("Calibration tool process killed for tool: {}", tool),
        )
        .unwrap();

    Ok(())
}

#[tauri::command]
pub async fn get_message_interface(
    message_type: String,
    autoware_path: String,
    extra_workspaces: Vec<String>,
) -> Result<String, String> {
    let mut source_extra_workspaces_str = String::new();
    for workspace in extra_workspaces {
        source_extra_workspaces_str
            .push_str(&format!("source {}/install/setup.bash && ", workspace));
    }

    // Execute the `ros2 interface proto x` command
    let cmd_str = format!(
        "source /opt/ros/humble/setup.bash && \
         source {}/install/setup.bash && \
         {} \
         ros2 interface proto {} --no-quotes",
        autoware_path, source_extra_workspaces_str, message_type
    );

    let output = Command::new("bash")
        .arg("-c")
        .arg(&cmd_str)
        .output()
        .await
        .map_err(|e| e.to_string())?;

    if !output.status.success() {
        return Err(String::from_utf8_lossy(&output.stderr).to_string());
    }

    let output_str = String::from_utf8_lossy(&output.stdout).to_string();

    Ok(output_str)
}

#[tauri::command]
pub async fn publish_message(
    topic: String,
    message_type: String,
    message: String,
    flags: Vec<Flag>,
    autoware_path: String,
    extra_workspaces: Vec<String>,
    window: tauri::Window<Wry>,
) -> Result<(), String> {
    let publish_process = Arc::clone(&TOPIC_PUBLISH_PROCESS);

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

    let mut cmd_str = format!(
        "source /opt/ros/humble/setup.bash && \
         source {}/install/setup.bash && \
         {} \
         ros2 topic pub {} {} {}",
        autoware_path, source_extra_workspaces_str, topic, message_type, message
    );

    cmd_str.push_str(&cmd_flags);

    println!("Running command: {}", cmd_str);

    let mut output = Command::new("bash")
        .arg("-c")
        .arg(&cmd_str)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("Failed to start the publish process");

    let stdout: tokio::process::ChildStdout = output
        .stdout
        .take()
        .expect("child did not have a handle to stdout");

    let stderr: tokio::process::ChildStderr = output
        .stderr
        .take()
        .expect("child did not have a handle to stderr");

    // emit the output to the frontend
    tokio::spawn(async move {
        let reader = BufReader::new(stdout);
        let mut lines = reader.lines();

        while let Ok(Some(line)) = lines.next_line().await {
            window.emit("ros2-topic-pub-output", line).unwrap();
        }

        let err_reader = BufReader::new(stderr);
        let mut err_lines = err_reader.lines();

        while let Ok(Some(line)) = err_lines.next_line().await {
            window.emit("ros2-topic-pub-output", line).unwrap();
        }
    });

    // Store the child process reference
    let mut process_handle = publish_process.lock().unwrap();
    *process_handle = Some(output);
    println!(
        "Process handle id for topic publish: {}",
        process_handle.as_ref().unwrap().id().unwrap()
    );

    Ok(())
}

#[tauri::command]
pub async fn call_service(
    service_name: String,
    service_type: String,
    request: String,
    flag: Flag,
    autoware_path: String,
    extra_workspaces: Vec<String>,
    window: tauri::Window<Wry>,
) -> Result<(), String> {
    let service_process = Arc::clone(&SERVICE_CALL_PROCESS);

    let mut cmd_flag = String::new();

    match &flag.value {
        Value::String(s) => {
            if !s.is_empty() {
                cmd_flag.push_str(&format!(" {} {} ", flag.arg, s));
            }
        }
        _ => {}
    }

    let mut source_extra_workspaces_str = String::new();
    for workspace in extra_workspaces {
        source_extra_workspaces_str
            .push_str(&format!("source {}/install/setup.bash && ", workspace));
    }

    let mut cmd_str = format!(
        "source /opt/ros/humble/setup.bash && \
         source {}/install/setup.bash && \
         {} \
         ros2 service call {} {} {}",
        autoware_path, source_extra_workspaces_str, service_name, service_type, request
    );

    cmd_str.push_str(&cmd_flag);

    println!("Running command: {}", cmd_str);

    let mut output = Command::new("bash")
        .arg("-c")
        .arg(&cmd_str)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("Failed to start the service call process");

    let stdout: tokio::process::ChildStdout = output
        .stdout
        .take()
        .expect("child did not have a handle to stdout");

    let stderr: tokio::process::ChildStderr = output
        .stderr
        .take()
        .expect("child did not have a handle to stderr");

    // emit the output to the frontend
    tokio::spawn(async move {
        let reader = BufReader::new(stdout);
        let mut lines = reader.lines();

        while let Ok(Some(line)) = lines.next_line().await {
            window.emit("ros2-service-call-output", line).unwrap();
        }

        let err_reader = BufReader::new(stderr);
        let mut err_lines = err_reader.lines();

        while let Ok(Some(line)) = err_lines.next_line().await {
            window.emit("ros2-service-call-output", line).unwrap();
        }
    });

    // Store the child process reference
    let mut process_handle = service_process.lock().unwrap();
    *process_handle = Some(output);
    println!(
        "Process handle id for topic publish: {}",
        process_handle.as_ref().unwrap().id().unwrap()
    );

    Ok(())
}

#[tauri::command]
pub async fn kill_topic_pub() -> Result<(), String> {
    let child_opt = {
        let mut pub_process_handle = match TOPIC_PUBLISH_PROCESS.lock() {
            Ok(handle) => handle,
            Err(poisoned) => poisoned.into_inner(),
        };
        pub_process_handle.take()
    };

    if let Some(mut child) = child_opt {
        println!("Killing the publish process... {}", child.id().unwrap());

        // Kill the main bash process
        match child.kill().await {
            Ok(_) => println!("Successfully killed the main bash process"),
            Err(e) => eprintln!("Failed to kill the main bash process: {}", e),
        }

        // Wait for the child process to finish
        let _ = child.wait().await;
    } else {
        eprintln!("No publish process found");
    }

    Ok(())
}

#[tauri::command]
pub async fn kill_service_call() -> Result<(), String> {
    let child_opt = {
        let mut call_process_handle = match SERVICE_CALL_PROCESS.lock() {
            Ok(handle) => handle,
            Err(poisoned) => poisoned.into_inner(),
        };
        call_process_handle.take()
    };

    if let Some(mut child) = child_opt {
        println!(
            "Killing the service call process... {}",
            child.id().unwrap()
        );

        // Kill the main bash process
        match child.kill().await {
            Ok(_) => println!("Successfully killed the main bash process"),
            Err(e) => eprintln!("Failed to kill the main bash process: {}", e),
        }

        // Wait for the child process to finish
        let _ = child.wait().await;
    } else {
        eprintln!("No service call process found");
    }

    Ok(())
}

#[tauri::command]
pub async fn find_all_ros_message_types(
    autoware_path: String,
    extra_workspaces: Vec<String>,
) -> Result<String, String> {
    let mut source_extra_workspaces_str = String::new();
    for workspace in extra_workspaces {
        source_extra_workspaces_str
            .push_str(&format!("source {}/install/setup.bash && ", workspace));
    }
    let cmd_str = format!(
        "source /opt/ros/humble/setup.bash && \
         source {}/install/setup.bash && \
         {} \
         ros2 interface list -m",
        autoware_path, source_extra_workspaces_str
    );

    let output = Command::new("bash")
        .arg("-c")
        .arg(&cmd_str)
        .output()
        .await
        .map_err(|e| e.to_string())?;

    if !output.status.success() {
        return Err(String::from_utf8_lossy(&output.stderr).to_string());
    }

    let output_str = String::from_utf8_lossy(&output.stdout).to_string();

    // remove Messages: from the string and trim the whitespace
    let output_str = output_str.replace("Messages:", "").trim().to_string();

    Ok(output_str)
}
