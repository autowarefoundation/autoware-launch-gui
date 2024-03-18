use crate::components;
use tauri::{Builder, Wry};
pub fn setup_handlers() -> Builder<Wry> {
    let app_builder = tauri::Builder::default();
    app_builder.invoke_handler(tauri::generate_handler![
        components::system::get_system_info,
        components::files::files_in_dir,
        components::autoware_manager::launch_autoware,
        components::autoware_manager::kill_autoware_process,
        components::autoware_manager::autoware_installed_packages,
        components::autoware_manager::get_topics,
        components::autoware_manager::get_services,
        components::json_profile::save_profile,
        components::json_profile::load_profile,
        components::xml_parse::parse_and_send_xml,
        components::yaml_edit::save_edits_yaml,
        components::yaml_edit::find_yaml_files,
        components::yaml_edit::parse_yaml,
        components::rosbag_manager::play_rosbag,
        components::rosbag_manager::toggle_pause_state,
        components::rosbag_manager::stop_rosbag_play,
        components::rosbag_manager::set_rosbag_playback_rate,
        components::rosbag_manager::get_rosbag_info,
        components::ros2_cmd_manager::start_topic_echo,
        components::ros2_cmd_manager::kill_topic_echo,
        components::ros2_cmd_manager::start_bag_record,
        components::ros2_cmd_manager::kill_bag_record,
        components::ros2_cmd_manager::launch_tool,
        components::ros2_cmd_manager::kill_calibration_tool,
        components::ros2_cmd_manager::get_message_interface,
        components::ros2_cmd_manager::publish_message,
        components::ros2_cmd_manager::call_service,
        components::ros2_cmd_manager::kill_service_call,
        components::ros2_cmd_manager::kill_topic_pub,
        components::ros2_cmd_manager::find_all_ros_message_types,
        components::ssh_manager::execute_command_in_shell,
        components::ssh_manager::start_shell_session,
        components::ssh_manager::kill_ssh_connection,
        components::ssh_manager::connect_ssh
    ])
}
