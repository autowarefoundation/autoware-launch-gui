use serde_json::Value;
use std::fs::{read_to_string, File};
use std::io::Write;
use tauri::command;

#[command]
pub fn save_profile(profile_path: String, profile_data: Value) -> Result<(), String> {
    let json_string = serde_json::to_string(&profile_data).map_err(|e| e.to_string())?;

    let mut file = File::create(profile_path).map_err(|e| e.to_string())?;
    file.write_all(json_string.as_bytes())
        .map_err(|e| e.to_string())?;

    Ok(())
}

#[command]
pub fn load_profile(path: String) -> Result<Value, String> {
    // Read the file
    let file_content = read_to_string(&path).map_err(|e| e.to_string())?;

    // Parse the JSON content
    let json_content: Value = serde_json::from_str(&file_content).map_err(|e| e.to_string())?;

    Ok(json_content)
}
