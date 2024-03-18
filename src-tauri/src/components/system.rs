use tokio::process::Command as TokioCommand;

#[derive(serde::Serialize)]
pub struct SystemInfo {
    cpus_usage: Vec<f32>, // New field for individual CPU usages
    cpu_usage: f32,
    top_processes: Vec<ProcessInfo>,
    memory_used_percentage: f32,
    memory_usage: String, // New field to store memory usage in "xGB/yGB" format
    autoware_processes: Vec<String>,
}

#[derive(serde::Serialize)]
pub struct ProcessInfo {
    name: String,
    cpu_usage: f32,
}

#[tauri::command(async)]
pub async fn get_system_info(autoware_path: Option<String>) -> Result<SystemInfo, String> {
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
