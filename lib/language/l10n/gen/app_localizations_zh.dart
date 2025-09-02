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
  String get ip_address => 'IP地址';

  @override
  String get port => '端口';

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
}
