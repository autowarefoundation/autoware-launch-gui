#[tauri::command]
pub fn files_in_dir(map_path: String) -> Vec<String> {
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
