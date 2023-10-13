use nix::unistd::Pid;
use once_cell::sync::Lazy;
use serde::Deserialize;
use std::io::{BufRead, BufReader};
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};
use tauri::Wry;

// Global reference to the Autoware process
pub static AUTOWARE_PROCESS: Lazy<Arc<Mutex<Option<std::process::Child>>>> =
    Lazy::new(|| Arc::new(Mutex::new(None)));
pub static AUTOWARE_PIDS: Lazy<Arc<Mutex<Vec<i32>>>> =
    Lazy::new(|| Arc::new(Mutex::new(Vec::new())));

#[derive(Deserialize)]
pub struct ArgsToLaunch {
    arg: String,
    value: String,
}

#[tauri::command]
pub fn launch_autoware(
    window: tauri::Window<Wry>,
    path: String,
    launch_file: String,
    args_to_launch: Vec<ArgsToLaunch>,
) {
    let autoware_process = Arc::clone(&AUTOWARE_PROCESS);

    let window_clone = window.clone();
    let window_clone_err = window.clone();

    // Construct the command string
    let cmd_str = format!(
        "source /opt/ros/humble/setup.bash && \
         source {}/install/setup.bash && \
         ros2 launch autoware_launch {} {}",
        path,
        launch_file,
        args_to_launch
            .iter()
            .map(|arg| format!("{}:={}", arg.arg, arg.value))
            .collect::<Vec<String>>()
            .join(" ")
    );

    std::thread::spawn(move || {
        // Execute the command
        let mut child = Command::new("bash")
            .arg("-c")
            .arg(&cmd_str)
            .stdout(Stdio::piped())
            .spawn()
            .map_err(|e| e.to_string())
            .unwrap();

        // Add the main command's PID to the list
        let main_pid = child.id() as i32;
        {
            let mut pids_lock = AUTOWARE_PIDS.lock().unwrap();
            pids_lock.push(main_pid);
        }

        let stdout = child
            .stdout
            .take()
            .expect("child did not have a handle to stdout");

        // Spawn a new thread to read the stdout
        std::thread::spawn(move || {
            let pids = Arc::clone(&AUTOWARE_PIDS);
            let reader = BufReader::new(stdout);

            // "package" something "not found" to be matched in the stdout in this way
            // since the package name is not known
            for line in reader.lines() {
                match line {
                    Ok(line) => {
                        window_clone.emit("autoware-output", line.clone()).unwrap();
                        if line.contains("[ERROR]") {
                            println!("Autoware ERROR: {}", line);
                            // i believe we should emit that since it errors out, the process is not running anymore
                            // clear out the pids list
                            // and enable the button
                            if line.contains("package") && line.contains("not found") {
                                // clear the pids list
                                let mut pids_lock = pids.lock().unwrap();
                                pids_lock.clear();
                                // emit an event to the frontend to enable the button
                                window_clone_err
                                    .emit("pids-cleared", "pid list cleared")
                                    .unwrap();
                                // emit an event to the frontend to show the error
                                window_clone_err
                                    .emit("package-not-found", "package not found")
                                    .unwrap();
                            }
                        } else if line.contains("process started with pid") {
                            println!("Autoware OUPUT: {}", line);
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
                            println!("Autoware OUTPUT: {}", line);
                            if let Some(start) = line.rfind('-') {
                                let pid_str = &line[start + 1..];
                                if let Ok(pid) = pid_str.parse::<i32>() {
                                    let mut pids_lock = pids.lock().unwrap();
                                    pids_lock.push(pid);
                                }
                            }
                        }
                    }
                    Err(err) => {
                        eprintln!("Error reading Autoware stdout: {}", err);
                    }
                }

                // lets send to the frontend the length of the pids vector to be able to disable the button when the process is running
                let pids_lock = pids.lock().unwrap();
                let pids_len = pids_lock.len();
                window_clone.emit("pids_len", pids_len).unwrap();
            }
        });

        // Store the child process reference
        let mut autoware_handle = autoware_process.lock().unwrap();
        *autoware_handle = Some(child);
    });
}

#[tauri::command]
pub fn kill_autoware_process(window: tauri::Window<Wry>) -> Result<(), String> {
    std::thread::spawn(move || {
        let mut autoware_handle = AUTOWARE_PROCESS.lock().unwrap();

        if let Some(child) = autoware_handle.as_mut() {
            println!("Killing the Autoware process...");

            // Kill the main bash process
            match child.kill() {
                Ok(_) => println!("Successfully killed the main bash process"),
                Err(e) => eprintln!("Failed to kill the main bash process: {}", e),
            }

            // Kill the other Autoware processes
            let mut pids_lock = AUTOWARE_PIDS.lock().unwrap();

            for &pid in pids_lock.iter() {
                nix::sys::signal::kill(Pid::from_raw(pid), nix::sys::signal::Signal::SIGKILL)
                    .expect("Failed to kill process");
            }

            // Clear the list of PIDs
            pids_lock.clear();

            // Emit an event to the frontend to enable the button
            window.emit("pids-cleared", "pid list cleared").unwrap();

            // kill the child process before setting it to None and then kill the main process
            nix::sys::signal::kill(
                Pid::from_raw(child.id() as i32),
                nix::sys::signal::Signal::SIGKILL,
            )
            .expect("Failed to kill process");

            // use a different way to kill the child process with the pid without using the nix library or the child.kill() method
            let pid = child.id() as i32;

            // extra check to make sure the pid is not 0
            if pid == 0 {
                eprintln!("There is no Autoware process running");
                return;
            } else {
                println!("The pid is {}", pid);
                Command::new("kill")
                    .arg("-9")
                    .arg(pid.to_string())
                    .output()
                    .expect("failed to execute process");
                println!(
                    "Successfully killed the Autoware process with PID {}",
                    child.id()
                );
            }
            // Set the child process reference to None
            *autoware_handle = None;
        } else {
            eprintln!("No Autoware process found");
        }
    });

    Ok(())
}

#[tauri::command]
pub fn autoware_installed_packages(autoware_path: String) -> Result<Vec<String>, String> {
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
