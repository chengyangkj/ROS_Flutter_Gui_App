// ignore: unused_import
import 'package:intl/intl.dart' as intl;
import 'app_localizations.dart';

// ignore_for_file: type=lint

/// The translations for Chinese (`zh`).
class AppLocalizationsZh extends AppLocalizations {
  AppLocalizationsZh([String locale = 'zh']) : super(locale);

  @override
  String get auto => '自动';

  @override
  String get light => '浅色';

  @override
  String get dark => '暗色';

  @override
  String get connect_robot => '连接机器人';

  @override
  String get connect_robot_subtitle => '输入ROS Bridge IP 以连接 ROS 机器人系统';

  @override
  String get ip_address => 'IP地址';

  @override
  String get port => '端口';

  @override
  String get tile_server_port => '瓦片服务端口';

  @override
  String get robot_type => '机器人类型';

  @override
  String get default_config_template => '默认配置模版';

  @override
  String get max_speed => '最大前进速度';

  @override
  String get max_y_speed => '最大横向速度';

  @override
  String get max_angular_speed => '最大角速度';

  @override
  String get map_frame => '地图坐标系';

  @override
  String get odom_frame => '里程计坐标系';

  @override
  String get base_frame => '机器人坐标系';

  @override
  String get laser_frame => '激光雷达坐标系';

  @override
  String get map_topic => '地图话题';

  @override
  String get laser_topic => '激光雷达话题';

  @override
  String get global_path_topic => '全局路径话题';

  @override
  String get local_path_topic => '局部路径话题';

  @override
  String get trace_path_topic => '轨迹路径话题';

  @override
  String get reloc_topic => '重定位话题';

  @override
  String get nav_goal_topic => '导航目标话题';

  @override
  String get odometry_topic => '里程计话题';

  @override
  String get speed_ctrl_topic => '速度控制话题';

  @override
  String get battery_topic => '电池状态话题';

  @override
  String get save => '保存';

  @override
  String get cancel => '取消';

  @override
  String get image_topic => '图像话题';

  @override
  String get screen_orientation => '屏幕方向';

  @override
  String get connect_error => '连接ROS失败，请检查IP和端口是否正确';

  @override
  String get edit => '编辑';

  @override
  String get config_saved => '设置已保存，重启应用后生效';

  @override
  String get ok => '确定';

  @override
  String get image_port => '图像端口';

  @override
  String get image_width => '图像宽度';

  @override
  String get image_height => '图像高度';

  @override
  String get confirm_change => '确认切换';

  @override
  String get switch_template_will_reset_all_settings => '切换模板将重置所有设置';

  @override
  String get portrait => '竖屏';

  @override
  String get landscape => '横屏';

  @override
  String get language => '语言';

  @override
  String get switch_language => '切换语言';

  @override
  String get zh => '中文';

  @override
  String get en => '英文';

  @override
  String get setting => '设置';

  @override
  String get basic_setting => '基础设置';

  @override
  String get topic_setting => '话题设置';

  @override
  String get app_setting => 'APP设置';

  @override
  String get robot_footprint_topic => '机器人尺寸话题';

  @override
  String get local_cost_map_topic => '局部代价地图话题';

  @override
  String get pointcloud2_topic => '点云话题';

  @override
  String get global_costmap_topic => '全局代价地图话题';

  @override
  String get robot_size => '机器人图标尺寸';

  @override
  String get not_allow_send_nav_goal => '当前模式不允许下发导航模式，请切换至普通模式!';

  @override
  String get gamepad_mapping => '手柄按键映射';

  @override
  String get remap => '重新映射';

  @override
  String get start_mapping_message => '请推动摇杆或按键至该位置，开始映射';

  @override
  String get mapping_reset => '映射已恢复默认设置';

  @override
  String get left_stick_x => '左摇杆 X';

  @override
  String get left_stick_y => '左摇杆 Y';

  @override
  String get right_stick_x => '右摇杆 X';

  @override
  String get right_stick_y => '右摇杆 Y';

  @override
  String get button_a => '按钮 A';

  @override
  String get button_b => '按钮 B';

  @override
  String get button_x => '按钮 X';

  @override
  String get button_y => '按钮 Y';

  @override
  String get camera_fixed_no_layer => '相机视角固定时不可调整图层！';

  @override
  String get switch_to_normal_mode => '请先切换到正常模式,再点击导航点';

  @override
  String get emergency_stop_triggered => '已触发急停！';

  @override
  String get stop_nav => '停止导航';

  @override
  String get stop => '停止';

  @override
  String get layer_grid => '网格图层';

  @override
  String get layer_global_costmap => '全局代价地图';

  @override
  String get layer_local_costmap => '局部代价地图';

  @override
  String get layer_laser => '激光雷达数据';

  @override
  String get layer_pointcloud => '点云数据';

  @override
  String get layer_global_path => '全局路径';

  @override
  String get layer_local_path => '局部路径';

  @override
  String get layer_topology => '拓扑地图';

  @override
  String get map_edit => '地图编辑';

  @override
  String get zoom_in => '放大';

  @override
  String get zoom_out => '缩小';

  @override
  String get center_on_robot => '定位到机器人';

  @override
  String get exit => '退出';

  @override
  String get layers => '图层';

  @override
  String get reloc => '重定位';

  @override
  String get camera => '相机';

  @override
  String get manual_control => '手动控制';

  @override
  String get emergency_stop => '急停';

  @override
  String get stop_navigation => '停止导航';

  @override
  String get open_file => '打开文件';

  @override
  String get save_file => '保存文件';

  @override
  String get undo => '撤销';

  @override
  String get redo => '重做';

  @override
  String get exit_edit_mode => '退出编辑模式';

  @override
  String get add_nav_point => '添加导航点';

  @override
  String get draw_obstacle => '绘制障碍物';

  @override
  String get erase_obstacle => '擦除障碍物';

  @override
  String get add_current_position => '添加当前位置';

  @override
  String get map_management => '地图管理';

  @override
  String get save_success => '保存成功';

  @override
  String get save_success_desc => '已发布拓扑地图与栅格地图';

  @override
  String get save_failed => '保存失败';

  @override
  String get save_as => '另存为';

  @override
  String get map_name => '地图名称';

  @override
  String get save_as_success => '另存为成功';

  @override
  String save_as_desc(Object name) {
    return '已保存为: $name';
  }

  @override
  String get save_as_failed => '另存为失败';

  @override
  String get tool_move => '移动';

  @override
  String get tool_point => '点位';

  @override
  String get tool_route => '拓扑线';

  @override
  String get tool_brush => '画笔';

  @override
  String get tool_eraser => '橡皮';

  @override
  String route_start_selected(Object name) {
    return '已选择起点: $name';
  }

  @override
  String route_created(Object from, Object to) {
    return '已创建连线: $from -> $to';
  }

  @override
  String get route_properties => '拓扑线属性';

  @override
  String direction(Object from, Object to) {
    return '方向: $from -> $to';
  }

  @override
  String get controller => '控制器';

  @override
  String get controller_readonly => '控制器（只读）';

  @override
  String get close => '关闭';

  @override
  String get delete_route => '删除该方向';

  @override
  String get point_properties => '点位属性';

  @override
  String get name => '名称';

  @override
  String get delete_point => '删除该点位';

  @override
  String get retry => '重试';

  @override
  String get no_map => '暂无地图';

  @override
  String get current_in_use => '当前使用';

  @override
  String get switch_map => '切换地图';

  @override
  String get delete_map_tooltip_current => '当前使用中，不可删除';

  @override
  String get delete => '删除';

  @override
  String get confirm_delete => '确认删除';

  @override
  String confirm_delete_map(Object name) {
    return '确定要删除地图「$name」吗？此操作不可恢复。';
  }

  @override
  String map_deleted(Object name) {
    return '已删除: $name';
  }

  @override
  String delete_failed(Object e) {
    return '删除失败: $e';
  }

  @override
  String get position_label => '位置';

  @override
  String position_format(Object x, Object y) {
    return '位置: ($x, $y)';
  }

  @override
  String init_error(Object error) {
    return '发生错误：$error';
  }

  @override
  String get diagnostic_warning => '警告';

  @override
  String get diagnostic_error => '错误';

  @override
  String get diagnostic_stale => '失活';

  @override
  String get diagnostic_normal => '正常';

  @override
  String diagnostic_health(Object level, Object component) {
    return '健康诊断:[$level] $component';
  }

  @override
  String diagnostic_hardware(Object id, Object msg) {
    return '硬件ID: $id\n消息: $msg';
  }

  @override
  String error_count(Object count) {
    return '错误: $count';
  }

  @override
  String warn_count(Object count) {
    return '警告: $count';
  }

  @override
  String get nav_point_info => '导航点信息';

  @override
  String get position_coords => '位置坐标';

  @override
  String get coord_x => 'X坐标';

  @override
  String get coord_y => 'Y坐标';

  @override
  String get heading => '方向';

  @override
  String get stop_manual_first => '请先停止手动控制';

  @override
  String nav_goal_sent(Object name) {
    return '已发送导航目标到 $name';
  }

  @override
  String get send_nav_goal => '发送导航目标';

  @override
  String get emergency_stopped => '急停已触发';

  @override
  String get nav_stopped => '导航已停止';

  @override
  String get legend_free => '自由';

  @override
  String get legend_occupied => '障碍';

  @override
  String get legend_unknown => '未知';

  @override
  String get nav_goal => '导航目标';

  @override
  String get charge_station => '充电站';

  @override
  String get layer_trace => '轨迹路径';

  @override
  String get layer_robot_footprint => '机器人轮廓';

  @override
  String get camera_image => '相机图像';

  @override
  String get no_map_available => '当前无地图可用，请先选择或创建地图';

  @override
  String get invalid_json => '导入失败：无效的JSON格式';

  @override
  String get data_stale => '超过5s未更新数据';

  @override
  String get unknown_hardware => '未知硬件';

  @override
  String get status => '状态';

  @override
  String get update_time => '更新时间';

  @override
  String get no_detail => '暂无详细信息';

  @override
  String get last_update => '最后更新';

  @override
  String get no_diagnostic_data => '暂无诊断数据';

  @override
  String get no_matching_diagnostic => '没有找到匹配的诊断数据';

  @override
  String get all => '全部';

  @override
  String get clear_filter => '清除筛选';

  @override
  String show_hardware_count(Object count) {
    return '显示 $count 个硬件组';
  }

  @override
  String get clear_all_filter => '清除所有筛选';

  @override
  String component_count(Object count) {
    return '组件数: $count';
  }

  @override
  String get system_diagnostic => '系统诊断';

  @override
  String get refresh => '刷新';

  @override
  String get search_component_hint => '搜索组件名称...';

  @override
  String get status_filter => '状态筛选';

  @override
  String get table_key => '键';

  @override
  String get table_value => '值';

  @override
  String get detail_info => '详细信息';

  @override
  String get diagnostic_overview => '诊断状态总览';

  @override
  String get hardware_id => '硬件ID';
}
