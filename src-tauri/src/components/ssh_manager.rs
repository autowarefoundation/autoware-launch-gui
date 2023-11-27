use once_cell::sync::Lazy;
use ssh2::{Channel, Session};
use std::io::prelude::*;
use std::net::TcpStream;
use std::sync::{Arc, Mutex};
use strip_ansi_escapes::strip_str;
use tauri::command;

static SSH_SESSION: Lazy<Arc<Mutex<Option<Session>>>> = Lazy::new(|| Arc::new(Mutex::new(None)));
static SHELL_CHANNEL: Lazy<Arc<Mutex<Option<Channel>>>> = Lazy::new(|| Arc::new(Mutex::new(None)));

#[command]
pub async fn connect_ssh(host: &str, username: &str, password: &str) -> Result<String, String> {
    // Establish a TCP connection to the remote host
    let tcp = TcpStream::connect(format!("{}:22", host)).map_err(|e| e.to_string())?;
    let mut session = Session::new().unwrap();
    session.set_tcp_stream(tcp);

    session.set_allow_sigpipe(true);

    // Perform the SSH handshake
    session.handshake().map_err(|e| e.to_string())?;

    // Try to authenticate with the given username and password
    session
        .userauth_password(username, password)
        .map_err(|e| e.to_string())?;

    // Check if authentication was successful
    if !session.authenticated() {
        return Err("Authentication failed".to_string());
    }

    // Save the session in the global state
    let mut ssh_session = SSH_SESSION.lock().unwrap();
    *ssh_session = Some(session);

    Ok("SSH connection established".to_string())
}

#[command]
pub async fn start_shell_session() -> Result<String, String> {
    let mut ssh_session = SSH_SESSION.lock().unwrap();
    if let Some(session) = ssh_session.as_mut() {
        let mut channel = session.channel_session().map_err(|e| e.to_string())?;
        channel
            .request_pty("xterm", None, None)
            .map_err(|e| e.to_string())?;
        channel.shell().map_err(|e| e.to_string())?;
        let channel = unsafe { std::mem::transmute::<ssh2::Channel, Channel>(channel) };
        *SHELL_CHANNEL.lock().unwrap() = Some(channel);
        Ok("Shell session started".to_string())
    } else {
        Err("SSH session not established".to_string())
    }
}

#[command]
pub async fn execute_command_in_shell(command: String, user: String) -> Result<String, String> {
    // if command is "exit" then kill the ssh connection
    if command == "exit" {
        kill_ssh_connection().await?;
        return Ok("SSH connection killed".to_string());
    }

    let mut shell_channel = SHELL_CHANNEL.lock().unwrap();
    if let Some(channel) = shell_channel.as_mut() {
        // Send the command
        channel
            .write_all((command.clone() + "\n").as_bytes())
            .map_err(|e| e.to_string())?;
        channel.flush().map_err(|e| e.to_string())?;

        // Read the output
        let mut output = String::new();
        let mut buffer = [0; 4096]; // Use a larger buffer for efficiency
        let mut end_of_command = false;

        while !end_of_command {
            match channel.read(&mut buffer) {
                Ok(0) => break, // No more data to read
                Ok(n) => {
                    let chunk = std::str::from_utf8(&buffer[..n]).map_err(|e| e.to_string())?;
                    // Check if the chunk contains the shell prompt, indicating the end of command output
                    // we check lines because the prompt is usually on a line by itself
                    // and we push all the lines except the one with the prompt
                    let lines = chunk.lines().collect::<Vec<&str>>();
                    // we push line by line except the one that contains the user
                    for line in lines {
                        if !line.contains(format!("{}@", user).as_str()) {
                            println!("print: {} ", line);
                            output.push_str(format!("{} \n", line).as_str());
                        } else {
                            end_of_command = true;
                        }
                    }
                }
                Err(e) => {
                    // Handle the error, possibly a timeout or other issue
                    return Err(format!("Error reading from channel: {}", e));
                }
            }
        }

        // Here you might want to process 'output' to remove any control characters or prompts
        let clean_output = strip_str(&output);

        // Remove the command from the start of the output, if it's there
        let clean_output = if clean_output.starts_with(&command) {
            clean_output.trim_start_matches(&command).trim_start()
        } else {
            &clean_output
        };

        println!("output: {}", clean_output);

        Ok(clean_output.to_string())
        // Ok(output.to_string())
    } else {
        Err("Shell session not started".to_string())
    }
}

// #[command]
// pub async fn end_shell_session() -> Result<String, String> {
//     let mut shell_channel = SHELL_CHANNEL.lock().unwrap();
//     if shell_channel.is_some() {
//         *shell_channel = None;
//         Ok("Shell session ended".to_string())
//     } else {
//         Err("Shell session not started".to_string())
//     }
// }

#[tauri::command]
pub async fn kill_ssh_connection() -> Result<(), String> {
    let mut ssh_session = SSH_SESSION.lock().unwrap();
    if let Some(session) = ssh_session.as_mut() {
        let _ = session.disconnect(None, "", None);
    }
    let mut shell_channel = SHELL_CHANNEL.lock().unwrap();
    if shell_channel.is_some() {
        *shell_channel = None;
    }
    Ok(())
}
