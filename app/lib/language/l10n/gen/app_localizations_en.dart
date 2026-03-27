// ignore: unused_import
import 'package:intl/intl.dart' as intl;
import 'app_localizations.dart';

// ignore_for_file: type=lint

/// The translations for English (`en`).
class AppLocalizationsEn extends AppLocalizations {
  AppLocalizationsEn([String locale = 'en']) : super(locale);

  @override
  String get auto => 'Auto';

  @override
  String get light => 'Light';

  @override
  String get dark => 'Dark';

  @override
  String get connect_robot => 'Connect To Robot';

  @override
  String get connect_robot_subtitle =>
      'Enter ROS Bridge IP to connect to ROS robot system';

  @override
  String get ip_address => 'IP Address';

  @override
  String get port => 'Port';

  @override
  String get http_server_port => 'HTTP Server Port';

  @override
  String get robot_type => 'Robot Type';

  @override
  String get default_config_template => 'Default Config Template';

  @override
  String get max_speed => 'Max Speed';

  @override
  String get max_y_speed => 'Max Y Speed';

  @override
  String get max_angular_speed => 'Max Angular Speed';

  @override
  String get map_frame => 'Map Frame';

  @override
  String get odom_frame => 'Odom Frame';

  @override
  String get base_frame => 'Base Frame';

  @override
  String get laser_frame => 'Laser Frame';

  @override
  String get map_topic => 'Map Topic';

  @override
  String get laser_topic => 'Laser Topic';

  @override
  String get global_path_topic => 'Global Path Topic';

  @override
  String get local_path_topic => 'Local Path Topic';

  @override
  String get trace_path_topic => 'Trace Path Topic';

  @override
  String get reloc_topic => 'Reloc Topic';

  @override
  String get nav_goal_topic => 'Nav Goal Topic';

  @override
  String get odometry_topic => 'Odometry Topic';

  @override
  String get speed_ctrl_topic => 'Speed Ctrl Topic';

  @override
  String get battery_topic => 'Battery Topic';

  @override
  String get save => 'Save';

  @override
  String get cancel => 'Cancel';

  @override
  String get image_topic => 'Image Topic';

  @override
  String get screen_orientation => 'Screen Orientation';

  @override
  String get connect_error => 'Connect ROS failed, please check IP and port';

  @override
  String get edit => 'Edit';

  @override
  String get config_saved => 'Config saved, restart app to take effect';

  @override
  String get ok => 'OK';

  @override
  String get image_port => 'Image Port';

  @override
  String get image_width => 'Image Width';

  @override
  String get image_height => 'Image Height';

  @override
  String get confirm_change => 'Confirm Change';

  @override
  String get switch_template_will_reset_all_settings =>
      'Switch template will reset all settings';

  @override
  String get portrait => 'Portrait';

  @override
  String get landscape => 'Landscape';

  @override
  String get language => 'Language';

  @override
  String get switch_language => 'Switch Language';

  @override
  String get zh => 'Chinese';

  @override
  String get en => 'English';

  @override
  String get setting => 'Setting';

  @override
  String get basic_setting => 'Basic Setting';

  @override
  String get topic_setting => 'Topic Setting';

  @override
  String get app_setting => 'APP Setting';

  @override
  String get robot_footprint_topic => 'Robot Footprint Topic';

  @override
  String get local_cost_map_topic => 'Local Cost Map Topic';

  @override
  String get pointcloud2_topic => 'Point Cloud Topic';

  @override
  String get global_costmap_topic => 'Global Cost Map Topic';

  @override
  String get robot_size => 'Robot Icon Size';

  @override
  String get not_allow_send_nav_goal =>
      'Current mode is not allow to send navigation goal, please checkout to normal mode!';

  @override
  String get gamepad_mapping => 'Gamepad Mapping';

  @override
  String get remap => 'Remap';

  @override
  String get start_mapping_message =>
      'Please push the joystick or button to this position to start mapping';

  @override
  String get mapping_reset => 'Mapping has been reset to default settings';

  @override
  String get left_stick_x => 'Left Stick X';

  @override
  String get left_stick_y => 'Left Stick Y';

  @override
  String get right_stick_x => 'Right Stick X';

  @override
  String get right_stick_y => 'Right Stick Y';

  @override
  String get button_a => 'Button A';

  @override
  String get button_b => 'Button B';

  @override
  String get button_x => 'Button X';

  @override
  String get button_y => 'Button Y';

  @override
  String get camera_fixed_no_layer =>
      'Cannot adjust layers when camera view is fixed!';

  @override
  String get switch_to_normal_mode =>
      'Please switch to normal mode first, then click navigation point';

  @override
  String get emergency_stop_triggered => 'Emergency stop triggered!';

  @override
  String get stop_nav => 'Stop Navigation';

  @override
  String get stop => 'Stop';

  @override
  String get layer_grid => 'Grid Layer';

  @override
  String get layer_global_costmap => 'Global Cost Map';

  @override
  String get layer_local_costmap => 'Local Cost Map';

  @override
  String get layer_laser => 'Laser Data';

  @override
  String get layer_pointcloud => 'Point Cloud Data';

  @override
  String get layer_global_path => 'Global Path';

  @override
  String get layer_local_path => 'Local Path';

  @override
  String get layer_topology => 'Topology Map';

  @override
  String get map_edit => 'Map Edit';

  @override
  String get zoom_in => 'Zoom In';

  @override
  String get zoom_out => 'Zoom Out';

  @override
  String get center_on_robot => 'Center on Robot';

  @override
  String get exit => 'Exit';

  @override
  String get layers => 'Layers';

  @override
  String get layer_color => 'Color';

  @override
  String get layer_dot_size => 'Dot size';

  @override
  String get reloc => 'Relocalization';

  @override
  String get camera => 'Camera';

  @override
  String get manual_control => 'Manual Control';

  @override
  String get emergency_stop => 'Emergency Stop';

  @override
  String get stop_navigation => 'Stop Navigation';

  @override
  String get open_file => 'Open File';

  @override
  String get save_file => 'Save File';

  @override
  String get undo => 'Undo';

  @override
  String get redo => 'Redo';

  @override
  String get exit_edit_mode => 'Exit Edit Mode';

  @override
  String get add_nav_point => 'Add Navigation Point';

  @override
  String get draw_obstacle => 'Draw Obstacle';

  @override
  String get erase_obstacle => 'Erase Obstacle';

  @override
  String get add_current_position => 'Add Current Position';

  @override
  String get map_management => 'Map Management';

  @override
  String get save_success => 'Save Success';

  @override
  String get save_success_desc => 'Topology map and occupancy map published';

  @override
  String get save_failed => 'Save Failed';

  @override
  String get save_as => 'Save As';

  @override
  String get map_name => 'Map Name';

  @override
  String get save_as_success => 'Save As Success';

  @override
  String save_as_desc(Object name) {
    return 'Saved as: $name';
  }

  @override
  String get save_as_failed => 'Save As Failed';

  @override
  String get tool_move => 'Move';

  @override
  String get tool_point => 'Point';

  @override
  String get tool_route => 'Route';

  @override
  String get tool_brush => 'Brush';

  @override
  String get tool_eraser => 'Eraser';

  @override
  String route_start_selected(Object name) {
    return 'Start selected: $name';
  }

  @override
  String route_created(Object from, Object to) {
    return 'Route created: $from -> $to';
  }

  @override
  String get route_properties => 'Route Properties';

  @override
  String direction(Object from, Object to) {
    return 'Direction: $from -> $to';
  }

  @override
  String get controller => 'Controller';

  @override
  String get controller_readonly => 'Controller (Read Only)';

  @override
  String get close => 'Close';

  @override
  String get delete_route => 'Delete This Direction';

  @override
  String get point_properties => 'Point Properties';

  @override
  String get name => 'Name';

  @override
  String get delete_point => 'Delete This Point';

  @override
  String get retry => 'Retry';

  @override
  String get no_map => 'No Map';

  @override
  String get current_in_use => 'In Use';

  @override
  String get switch_map => 'Switch Map';

  @override
  String get delete_map_tooltip_current => 'In use, cannot delete';

  @override
  String get delete => 'Delete';

  @override
  String get confirm_delete => 'Confirm Delete';

  @override
  String confirm_delete_map(Object name) {
    return 'Delete map \"$name\"? This cannot be undone.';
  }

  @override
  String map_deleted(Object name) {
    return 'Deleted: $name';
  }

  @override
  String delete_failed(Object e) {
    return 'Delete failed: $e';
  }

  @override
  String get position_label => 'Position';

  @override
  String position_format(Object x, Object y) {
    return 'Position: ($x, $y)';
  }

  @override
  String init_error(Object error) {
    return 'Error: $error';
  }

  @override
  String get diagnostic_warning => 'Warning';

  @override
  String get diagnostic_error => 'Error';

  @override
  String get diagnostic_stale => 'Stale';

  @override
  String get diagnostic_normal => 'Normal';

  @override
  String diagnostic_health(Object level, Object component) {
    return 'Diagnostic: [$level] $component';
  }

  @override
  String diagnostic_hardware(Object id, Object msg) {
    return 'Hardware ID: $id\nMessage: $msg';
  }

  @override
  String error_count(Object count) {
    return 'Errors: $count';
  }

  @override
  String warn_count(Object count) {
    return 'Warnings: $count';
  }

  @override
  String get nav_point_info => 'Nav Point Info';

  @override
  String get position_coords => 'Position Coords';

  @override
  String get coord_x => 'X';

  @override
  String get coord_y => 'Y';

  @override
  String get heading => 'Heading';

  @override
  String get stop_manual_first => 'Please stop manual control first';

  @override
  String nav_goal_sent(Object name) {
    return 'Nav goal sent to $name';
  }

  @override
  String get send_nav_goal => 'Send Nav Goal';

  @override
  String get emergency_stopped => 'Emergency Stop Triggered';

  @override
  String get nav_stopped => 'Navigation Stopped';

  @override
  String get legend_free => 'Free';

  @override
  String get legend_occupied => 'Occupied';

  @override
  String get legend_unknown => 'Unknown';

  @override
  String get nav_goal => 'Nav Goal';

  @override
  String get charge_station => 'Charge Station';

  @override
  String get layer_trace => 'Trace Path';

  @override
  String get layer_robot_footprint => 'Robot Footprint';

  @override
  String get camera_image => 'Camera Image';

  @override
  String get no_map_available =>
      'No map available, please select or create one first';

  @override
  String get invalid_json => 'Import failed: invalid JSON format';

  @override
  String get data_stale => 'No data update for over 5s';

  @override
  String get unknown_hardware => 'Unknown hardware';

  @override
  String get status => 'Status';

  @override
  String get update_time => 'Update time';

  @override
  String get no_detail => 'No details available';

  @override
  String get last_update => 'Last update';

  @override
  String get no_diagnostic_data => 'No diagnostic data';

  @override
  String get no_matching_diagnostic => 'No matching diagnostic data found';

  @override
  String get all => 'All';

  @override
  String get clear_filter => 'Clear filter';

  @override
  String show_hardware_count(Object count) {
    return 'Showing $count hardware groups';
  }

  @override
  String get clear_all_filter => 'Clear all filters';

  @override
  String component_count(Object count) {
    return 'Components: $count';
  }

  @override
  String get system_diagnostic => 'System Diagnostic';

  @override
  String get refresh => 'Refresh';

  @override
  String get search_component_hint => 'Search component...';

  @override
  String get status_filter => 'Status filter';

  @override
  String get table_key => 'Key';

  @override
  String get table_value => 'Value';

  @override
  String get detail_info => 'Detail';

  @override
  String get diagnostic_overview => 'Diagnostic Overview';

  @override
  String get hardware_id => 'Hardware ID';
}
