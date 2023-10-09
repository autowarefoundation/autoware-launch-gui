use regex::Regex;
use serde_yaml::Value;
use std::fs;

#[derive(serde::Deserialize)]
pub struct YamlArg {
    arg: String,
    value: Value,
}

#[tauri::command]
pub fn find_yaml_files(path: String) -> Result<Vec<String>, String> {
    let mut yaml_files: Vec<String> = Vec::new();

    // Recursively search for yaml files
    for entry in walkdir::WalkDir::new(path)
        .into_iter()
        .filter_map(|e| e.ok())
    {
        if entry.path().is_file() {
            if let Some(ext) = entry.path().extension() {
                if ext == "yaml" || ext == "yml" {
                    yaml_files.push(entry.path().to_str().unwrap().to_string());
                }
            }
        }
    }

    Ok(yaml_files)
}

#[tauri::command]
pub fn parse_yaml(path: String) -> Result<Value, String> {
    let yaml_file = fs::read_to_string(path).map_err(|e| e.to_string())?;
    let yaml_value: Value = serde_yaml::from_str(&yaml_file).map_err(|e| e.to_string())?;

    Ok(yaml_value)
}

// #[tauri::command]
// pub fn save_edits_yaml(path: String, new_yaml_args: Vec<YamlArg>) -> Result<(), String> {
//     let path_clone = path.clone();
//     let path_clone1 = path.clone();
//     let yaml_file = fs::read_to_string(&path).map_err(|e| e.to_string())?;
//     // get the new yaml args and parse them into a Value so we can update the yaml file
//     let mut yaml_value: Value = serde_yaml::from_str(&yaml_file).map_err(|e| e.to_string())?;

//     // create a .old file using the path
//     // save the old yaml file to the .old file
//     // save the new yaml file to the original file

//     let old_file_path = path_clone + ".old";
//     fs::write(old_file_path, yaml_file).map_err(|e| e.to_string())?;

//     // update the yaml file with the new values
//     for yaml_arg in new_yaml_args {
//         yaml_value[yaml_arg.arg] = yaml_arg.value;
//     }

//     // write the new yaml file
//     fs::write(
//         path_clone1,
//         serde_yaml::to_string(&yaml_value).map_err(|e| e.to_string())?,
//     )
//     .map_err(|e| e.to_string())?;

//     Ok(())
// }

#[tauri::command]
pub fn save_edits_yaml(path: String, new_yaml_args: Vec<YamlArg>) -> Result<(), String> {
    let path_clone = path.clone();
    let yaml_file = fs::read_to_string(&path).map_err(|e| e.to_string())?;
    let mut yaml_value: Value = serde_yaml::from_str(&yaml_file).map_err(|e| e.to_string())?;

    // create a .old file using the path
    let old_file_path = path_clone + ".old";
    fs::write(old_file_path, yaml_file).map_err(|e| e.to_string())?;

    // update the yaml file with the new values
    for yaml_arg in new_yaml_args {
        yaml_value[yaml_arg.arg] = yaml_arg.value;
    }

    // Serialize the new YAML value to a string
    let mut serialized = serde_yaml::to_string(&yaml_value).map_err(|e| e.to_string())?;

    // Use regex to convert block arrays to inline arrays
    let re = Regex::new(r"(?m)^(.+):\n((?:\s+- .+\n)+)").unwrap();
    serialized = re
        .replace_all(&serialized, |caps: &regex::Captures| {
            let key = &caps[1];
            let items: Vec<_> = caps[2]
                .lines()
                .map(|line| line.trim())
                // remove the "- " from the beginning of each line
                .map(|line| line.trim_start_matches("- "))
                .collect();
            format!("{}: [{}]\n", key, items.join(", "))
        })
        .to_string();

    // Write the new YAML string to the file
    fs::write(path, serialized).map_err(|e| e.to_string())?;

    Ok(())
}
