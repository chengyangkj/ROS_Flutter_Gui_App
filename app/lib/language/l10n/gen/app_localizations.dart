import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:intl/intl.dart' as intl;

import 'app_localizations_en.dart';
import 'app_localizations_zh.dart';

// ignore_for_file: type=lint

/// Callers can lookup localized strings with an instance of AppLocalizations
/// returned by `AppLocalizations.of(context)`.
///
/// Applications need to include `AppLocalizations.delegate()` in their app's
/// `localizationDelegates` list, and the locales they support in the app's
/// `supportedLocales` list. For example:
///
/// ```dart
/// import 'gen/app_localizations.dart';
///
/// return MaterialApp(
///   localizationsDelegates: AppLocalizations.localizationsDelegates,
///   supportedLocales: AppLocalizations.supportedLocales,
///   home: MyApplicationHome(),
/// );
/// ```
///
/// ## Update pubspec.yaml
///
/// Please make sure to update your pubspec.yaml to include the following
/// packages:
///
/// ```yaml
/// dependencies:
///   # Internationalization support.
///   flutter_localizations:
///     sdk: flutter
///   intl: any # Use the pinned version from flutter_localizations
///
///   # Rest of dependencies
/// ```
///
/// ## iOS Applications
///
/// iOS applications define key application metadata, including supported
/// locales, in an Info.plist file that is built into the application bundle.
/// To configure the locales supported by your app, you’ll need to edit this
/// file.
///
/// First, open your project’s ios/Runner.xcworkspace Xcode workspace file.
/// Then, in the Project Navigator, open the Info.plist file under the Runner
/// project’s Runner folder.
///
/// Next, select the Information Property List item, select Add Item from the
/// Editor menu, then select Localizations from the pop-up menu.
///
/// Select and expand the newly-created Localizations item then, for each
/// locale your application supports, add a new item and select the locale
/// you wish to add from the pop-up menu in the Value field. This list should
/// be consistent with the languages listed in the AppLocalizations.supportedLocales
/// property.
abstract class AppLocalizations {
  AppLocalizations(String locale)
    : localeName = intl.Intl.canonicalizedLocale(locale.toString());

  final String localeName;

  static AppLocalizations? of(BuildContext context) {
    return Localizations.of<AppLocalizations>(context, AppLocalizations);
  }

  static const LocalizationsDelegate<AppLocalizations> delegate =
      _AppLocalizationsDelegate();

  /// A list of this localizations delegate along with the default localizations
  /// delegates.
  ///
  /// Returns a list of localizations delegates containing this delegate along with
  /// GlobalMaterialLocalizations.delegate, GlobalCupertinoLocalizations.delegate,
  /// and GlobalWidgetsLocalizations.delegate.
  ///
  /// Additional delegates can be added by appending to this list in
  /// MaterialApp. This list does not have to be used at all if a custom list
  /// of delegates is preferred or required.
  static const List<LocalizationsDelegate<dynamic>> localizationsDelegates =
      <LocalizationsDelegate<dynamic>>[
        delegate,
        GlobalMaterialLocalizations.delegate,
        GlobalCupertinoLocalizations.delegate,
        GlobalWidgetsLocalizations.delegate,
      ];

  /// A list of this localizations delegate's supported locales.
  static const List<Locale> supportedLocales = <Locale>[
    Locale('en'),
    Locale('zh'),
  ];

  /// No description provided for @auto.
  ///
  /// In zh, this message translates to:
  /// **'自动'**
  String get auto;

  /// No description provided for @light.
  ///
  /// In zh, this message translates to:
  /// **'浅色'**
  String get light;

  /// No description provided for @dark.
  ///
  /// In zh, this message translates to:
  /// **'暗色'**
  String get dark;

  /// No description provided for @connect_robot.
  ///
  /// In zh, this message translates to:
  /// **'连接机器人'**
  String get connect_robot;

  /// No description provided for @connect_robot_subtitle.
  ///
  /// In zh, this message translates to:
  /// **'输入ROS Bridge IP 以连接 ROS 机器人系统'**
  String get connect_robot_subtitle;

  /// No description provided for @ip_address.
  ///
  /// In zh, this message translates to:
  /// **'IP地址'**
  String get ip_address;

  /// No description provided for @port.
  ///
  /// In zh, this message translates to:
  /// **'端口'**
  String get port;

  /// No description provided for @http_server_port.
  ///
  /// In zh, this message translates to:
  /// **'HTTP 服务端口'**
  String get http_server_port;

  /// No description provided for @robot_type.
  ///
  /// In zh, this message translates to:
  /// **'机器人类型'**
  String get robot_type;

  /// No description provided for @default_config_template.
  ///
  /// In zh, this message translates to:
  /// **'默认配置模版'**
  String get default_config_template;

  /// No description provided for @max_speed.
  ///
  /// In zh, this message translates to:
  /// **'最大前进速度'**
  String get max_speed;

  /// No description provided for @max_y_speed.
  ///
  /// In zh, this message translates to:
  /// **'最大横向速度'**
  String get max_y_speed;

  /// No description provided for @max_angular_speed.
  ///
  /// In zh, this message translates to:
  /// **'最大角速度'**
  String get max_angular_speed;

  /// No description provided for @map_frame.
  ///
  /// In zh, this message translates to:
  /// **'地图坐标系'**
  String get map_frame;

  /// No description provided for @odom_frame.
  ///
  /// In zh, this message translates to:
  /// **'里程计坐标系'**
  String get odom_frame;

  /// No description provided for @base_frame.
  ///
  /// In zh, this message translates to:
  /// **'机器人坐标系'**
  String get base_frame;

  /// No description provided for @laser_frame.
  ///
  /// In zh, this message translates to:
  /// **'激光雷达坐标系'**
  String get laser_frame;

  /// No description provided for @map_topic.
  ///
  /// In zh, this message translates to:
  /// **'地图话题'**
  String get map_topic;

  /// No description provided for @laser_topic.
  ///
  /// In zh, this message translates to:
  /// **'激光雷达话题'**
  String get laser_topic;

  /// No description provided for @global_path_topic.
  ///
  /// In zh, this message translates to:
  /// **'全局路径话题'**
  String get global_path_topic;

  /// No description provided for @local_path_topic.
  ///
  /// In zh, this message translates to:
  /// **'局部路径话题'**
  String get local_path_topic;

  /// No description provided for @trace_path_topic.
  ///
  /// In zh, this message translates to:
  /// **'轨迹路径话题'**
  String get trace_path_topic;

  /// No description provided for @reloc_topic.
  ///
  /// In zh, this message translates to:
  /// **'重定位话题'**
  String get reloc_topic;

  /// No description provided for @nav_goal_topic.
  ///
  /// In zh, this message translates to:
  /// **'导航目标话题'**
  String get nav_goal_topic;

  /// No description provided for @odometry_topic.
  ///
  /// In zh, this message translates to:
  /// **'里程计话题'**
  String get odometry_topic;

  /// No description provided for @speed_ctrl_topic.
  ///
  /// In zh, this message translates to:
  /// **'速度控制话题'**
  String get speed_ctrl_topic;

  /// No description provided for @battery_topic.
  ///
  /// In zh, this message translates to:
  /// **'电池状态话题'**
  String get battery_topic;

  /// No description provided for @save.
  ///
  /// In zh, this message translates to:
  /// **'保存'**
  String get save;

  /// No description provided for @cancel.
  ///
  /// In zh, this message translates to:
  /// **'取消'**
  String get cancel;

  /// No description provided for @image_topic.
  ///
  /// In zh, this message translates to:
  /// **'图像话题'**
  String get image_topic;

  /// No description provided for @screen_orientation.
  ///
  /// In zh, this message translates to:
  /// **'屏幕方向'**
  String get screen_orientation;

  /// No description provided for @connect_error.
  ///
  /// In zh, this message translates to:
  /// **'连接ROS失败，请检查IP和端口是否正确'**
  String get connect_error;

  /// No description provided for @edit.
  ///
  /// In zh, this message translates to:
  /// **'编辑'**
  String get edit;

  /// No description provided for @config_saved.
  ///
  /// In zh, this message translates to:
  /// **'设置已保存，重启应用后生效'**
  String get config_saved;

  /// No description provided for @ok.
  ///
  /// In zh, this message translates to:
  /// **'确定'**
  String get ok;

  /// No description provided for @image_port.
  ///
  /// In zh, this message translates to:
  /// **'图像端口'**
  String get image_port;

  /// No description provided for @image_width.
  ///
  /// In zh, this message translates to:
  /// **'图像宽度'**
  String get image_width;

  /// No description provided for @image_height.
  ///
  /// In zh, this message translates to:
  /// **'图像高度'**
  String get image_height;

  /// No description provided for @confirm_change.
  ///
  /// In zh, this message translates to:
  /// **'确认切换'**
  String get confirm_change;

  /// No description provided for @switch_template_will_reset_all_settings.
  ///
  /// In zh, this message translates to:
  /// **'切换模板将重置所有设置'**
  String get switch_template_will_reset_all_settings;

  /// No description provided for @portrait.
  ///
  /// In zh, this message translates to:
  /// **'竖屏'**
  String get portrait;

  /// No description provided for @landscape.
  ///
  /// In zh, this message translates to:
  /// **'横屏'**
  String get landscape;

  /// No description provided for @language.
  ///
  /// In zh, this message translates to:
  /// **'语言'**
  String get language;

  /// No description provided for @switch_language.
  ///
  /// In zh, this message translates to:
  /// **'切换语言'**
  String get switch_language;

  /// No description provided for @zh.
  ///
  /// In zh, this message translates to:
  /// **'中文'**
  String get zh;

  /// No description provided for @en.
  ///
  /// In zh, this message translates to:
  /// **'英文'**
  String get en;

  /// No description provided for @setting.
  ///
  /// In zh, this message translates to:
  /// **'设置'**
  String get setting;

  /// No description provided for @basic_setting.
  ///
  /// In zh, this message translates to:
  /// **'基础设置'**
  String get basic_setting;

  /// No description provided for @topic_setting.
  ///
  /// In zh, this message translates to:
  /// **'话题设置'**
  String get topic_setting;

  /// No description provided for @app_setting.
  ///
  /// In zh, this message translates to:
  /// **'APP设置'**
  String get app_setting;

  /// No description provided for @robot_footprint_topic.
  ///
  /// In zh, this message translates to:
  /// **'机器人尺寸话题'**
  String get robot_footprint_topic;

  /// No description provided for @local_cost_map_topic.
  ///
  /// In zh, this message translates to:
  /// **'局部代价地图话题'**
  String get local_cost_map_topic;

  /// No description provided for @pointcloud2_topic.
  ///
  /// In zh, this message translates to:
  /// **'点云话题'**
  String get pointcloud2_topic;

  /// No description provided for @global_costmap_topic.
  ///
  /// In zh, this message translates to:
  /// **'全局代价地图话题'**
  String get global_costmap_topic;

  /// No description provided for @robot_size.
  ///
  /// In zh, this message translates to:
  /// **'机器人图标尺寸'**
  String get robot_size;

  /// No description provided for @not_allow_send_nav_goal.
  ///
  /// In zh, this message translates to:
  /// **'当前模式不允许下发导航模式，请切换至普通模式!'**
  String get not_allow_send_nav_goal;

  /// No description provided for @gamepad_mapping.
  ///
  /// In zh, this message translates to:
  /// **'手柄按键映射'**
  String get gamepad_mapping;

  /// No description provided for @remap.
  ///
  /// In zh, this message translates to:
  /// **'重新映射'**
  String get remap;

  /// No description provided for @start_mapping_message.
  ///
  /// In zh, this message translates to:
  /// **'请推动摇杆或按键至该位置，开始映射'**
  String get start_mapping_message;

  /// No description provided for @mapping_reset.
  ///
  /// In zh, this message translates to:
  /// **'映射已恢复默认设置'**
  String get mapping_reset;

  /// No description provided for @left_stick_x.
  ///
  /// In zh, this message translates to:
  /// **'左摇杆 X'**
  String get left_stick_x;

  /// No description provided for @left_stick_y.
  ///
  /// In zh, this message translates to:
  /// **'左摇杆 Y'**
  String get left_stick_y;

  /// No description provided for @right_stick_x.
  ///
  /// In zh, this message translates to:
  /// **'右摇杆 X'**
  String get right_stick_x;

  /// No description provided for @right_stick_y.
  ///
  /// In zh, this message translates to:
  /// **'右摇杆 Y'**
  String get right_stick_y;

  /// No description provided for @button_a.
  ///
  /// In zh, this message translates to:
  /// **'按钮 A'**
  String get button_a;

  /// No description provided for @button_b.
  ///
  /// In zh, this message translates to:
  /// **'按钮 B'**
  String get button_b;

  /// No description provided for @button_x.
  ///
  /// In zh, this message translates to:
  /// **'按钮 X'**
  String get button_x;

  /// No description provided for @button_y.
  ///
  /// In zh, this message translates to:
  /// **'按钮 Y'**
  String get button_y;

  /// No description provided for @camera_fixed_no_layer.
  ///
  /// In zh, this message translates to:
  /// **'相机视角固定时不可调整图层！'**
  String get camera_fixed_no_layer;

  /// No description provided for @switch_to_normal_mode.
  ///
  /// In zh, this message translates to:
  /// **'请先切换到正常模式,再点击导航点'**
  String get switch_to_normal_mode;

  /// No description provided for @emergency_stop_triggered.
  ///
  /// In zh, this message translates to:
  /// **'已触发急停！'**
  String get emergency_stop_triggered;

  /// No description provided for @stop_nav.
  ///
  /// In zh, this message translates to:
  /// **'停止导航'**
  String get stop_nav;

  /// No description provided for @stop.
  ///
  /// In zh, this message translates to:
  /// **'停止'**
  String get stop;

  /// No description provided for @layer_grid.
  ///
  /// In zh, this message translates to:
  /// **'网格图层'**
  String get layer_grid;

  /// No description provided for @layer_global_costmap.
  ///
  /// In zh, this message translates to:
  /// **'全局代价地图'**
  String get layer_global_costmap;

  /// No description provided for @layer_local_costmap.
  ///
  /// In zh, this message translates to:
  /// **'局部代价地图'**
  String get layer_local_costmap;

  /// No description provided for @layer_laser.
  ///
  /// In zh, this message translates to:
  /// **'激光雷达数据'**
  String get layer_laser;

  /// No description provided for @layer_pointcloud.
  ///
  /// In zh, this message translates to:
  /// **'点云数据'**
  String get layer_pointcloud;

  /// No description provided for @layer_global_path.
  ///
  /// In zh, this message translates to:
  /// **'全局路径'**
  String get layer_global_path;

  /// No description provided for @layer_local_path.
  ///
  /// In zh, this message translates to:
  /// **'局部路径'**
  String get layer_local_path;

  /// No description provided for @layer_topology.
  ///
  /// In zh, this message translates to:
  /// **'拓扑地图'**
  String get layer_topology;

  /// No description provided for @map_edit.
  ///
  /// In zh, this message translates to:
  /// **'地图编辑'**
  String get map_edit;

  /// No description provided for @zoom_in.
  ///
  /// In zh, this message translates to:
  /// **'放大'**
  String get zoom_in;

  /// No description provided for @zoom_out.
  ///
  /// In zh, this message translates to:
  /// **'缩小'**
  String get zoom_out;

  /// No description provided for @center_on_robot.
  ///
  /// In zh, this message translates to:
  /// **'定位到机器人'**
  String get center_on_robot;

  /// No description provided for @exit.
  ///
  /// In zh, this message translates to:
  /// **'退出'**
  String get exit;

  /// No description provided for @layers.
  ///
  /// In zh, this message translates to:
  /// **'图层'**
  String get layers;

  /// No description provided for @layer_color.
  ///
  /// In zh, this message translates to:
  /// **'显示颜色'**
  String get layer_color;

  /// No description provided for @layer_dot_size.
  ///
  /// In zh, this message translates to:
  /// **'点大小'**
  String get layer_dot_size;

  /// No description provided for @reloc.
  ///
  /// In zh, this message translates to:
  /// **'重定位'**
  String get reloc;

  /// No description provided for @camera.
  ///
  /// In zh, this message translates to:
  /// **'相机'**
  String get camera;

  /// No description provided for @manual_control.
  ///
  /// In zh, this message translates to:
  /// **'手动控制'**
  String get manual_control;

  /// No description provided for @emergency_stop.
  ///
  /// In zh, this message translates to:
  /// **'急停'**
  String get emergency_stop;

  /// No description provided for @stop_navigation.
  ///
  /// In zh, this message translates to:
  /// **'停止导航'**
  String get stop_navigation;

  /// No description provided for @open_file.
  ///
  /// In zh, this message translates to:
  /// **'打开文件'**
  String get open_file;

  /// No description provided for @save_file.
  ///
  /// In zh, this message translates to:
  /// **'保存文件'**
  String get save_file;

  /// No description provided for @undo.
  ///
  /// In zh, this message translates to:
  /// **'撤销'**
  String get undo;

  /// No description provided for @redo.
  ///
  /// In zh, this message translates to:
  /// **'重做'**
  String get redo;

  /// No description provided for @exit_edit_mode.
  ///
  /// In zh, this message translates to:
  /// **'退出编辑模式'**
  String get exit_edit_mode;

  /// No description provided for @add_nav_point.
  ///
  /// In zh, this message translates to:
  /// **'添加导航点'**
  String get add_nav_point;

  /// No description provided for @draw_obstacle.
  ///
  /// In zh, this message translates to:
  /// **'绘制障碍物'**
  String get draw_obstacle;

  /// No description provided for @erase_obstacle.
  ///
  /// In zh, this message translates to:
  /// **'擦除障碍物'**
  String get erase_obstacle;

  /// No description provided for @add_current_position.
  ///
  /// In zh, this message translates to:
  /// **'添加当前位置'**
  String get add_current_position;

  /// No description provided for @map_management.
  ///
  /// In zh, this message translates to:
  /// **'地图管理'**
  String get map_management;

  /// No description provided for @save_success.
  ///
  /// In zh, this message translates to:
  /// **'保存成功'**
  String get save_success;

  /// No description provided for @save_success_desc.
  ///
  /// In zh, this message translates to:
  /// **'已发布拓扑地图与栅格地图'**
  String get save_success_desc;

  /// No description provided for @save_failed.
  ///
  /// In zh, this message translates to:
  /// **'保存失败'**
  String get save_failed;

  /// No description provided for @save_as.
  ///
  /// In zh, this message translates to:
  /// **'另存为'**
  String get save_as;

  /// No description provided for @map_name.
  ///
  /// In zh, this message translates to:
  /// **'地图名称'**
  String get map_name;

  /// No description provided for @save_as_success.
  ///
  /// In zh, this message translates to:
  /// **'另存为成功'**
  String get save_as_success;

  /// No description provided for @save_as_desc.
  ///
  /// In zh, this message translates to:
  /// **'已保存为: {name}'**
  String save_as_desc(Object name);

  /// No description provided for @save_as_failed.
  ///
  /// In zh, this message translates to:
  /// **'另存为失败'**
  String get save_as_failed;

  /// No description provided for @tool_move.
  ///
  /// In zh, this message translates to:
  /// **'移动'**
  String get tool_move;

  /// No description provided for @tool_point.
  ///
  /// In zh, this message translates to:
  /// **'点位'**
  String get tool_point;

  /// No description provided for @tool_route.
  ///
  /// In zh, this message translates to:
  /// **'拓扑线'**
  String get tool_route;

  /// No description provided for @tool_brush.
  ///
  /// In zh, this message translates to:
  /// **'画笔'**
  String get tool_brush;

  /// No description provided for @tool_eraser.
  ///
  /// In zh, this message translates to:
  /// **'橡皮'**
  String get tool_eraser;

  /// No description provided for @route_start_selected.
  ///
  /// In zh, this message translates to:
  /// **'已选择起点: {name}'**
  String route_start_selected(Object name);

  /// No description provided for @route_created.
  ///
  /// In zh, this message translates to:
  /// **'已创建连线: {from} -> {to}'**
  String route_created(Object from, Object to);

  /// No description provided for @route_properties.
  ///
  /// In zh, this message translates to:
  /// **'拓扑线属性'**
  String get route_properties;

  /// No description provided for @direction.
  ///
  /// In zh, this message translates to:
  /// **'方向: {from} -> {to}'**
  String direction(Object from, Object to);

  /// No description provided for @controller.
  ///
  /// In zh, this message translates to:
  /// **'控制器'**
  String get controller;

  /// No description provided for @controller_readonly.
  ///
  /// In zh, this message translates to:
  /// **'控制器（只读）'**
  String get controller_readonly;

  /// No description provided for @close.
  ///
  /// In zh, this message translates to:
  /// **'关闭'**
  String get close;

  /// No description provided for @delete_route.
  ///
  /// In zh, this message translates to:
  /// **'删除该方向'**
  String get delete_route;

  /// No description provided for @point_properties.
  ///
  /// In zh, this message translates to:
  /// **'点位属性'**
  String get point_properties;

  /// No description provided for @name.
  ///
  /// In zh, this message translates to:
  /// **'名称'**
  String get name;

  /// No description provided for @delete_point.
  ///
  /// In zh, this message translates to:
  /// **'删除该点位'**
  String get delete_point;

  /// No description provided for @retry.
  ///
  /// In zh, this message translates to:
  /// **'重试'**
  String get retry;

  /// No description provided for @no_map.
  ///
  /// In zh, this message translates to:
  /// **'暂无地图'**
  String get no_map;

  /// No description provided for @current_in_use.
  ///
  /// In zh, this message translates to:
  /// **'当前使用'**
  String get current_in_use;

  /// No description provided for @switch_map.
  ///
  /// In zh, this message translates to:
  /// **'切换地图'**
  String get switch_map;

  /// No description provided for @delete_map_tooltip_current.
  ///
  /// In zh, this message translates to:
  /// **'当前使用中，不可删除'**
  String get delete_map_tooltip_current;

  /// No description provided for @delete.
  ///
  /// In zh, this message translates to:
  /// **'删除'**
  String get delete;

  /// No description provided for @confirm_delete.
  ///
  /// In zh, this message translates to:
  /// **'确认删除'**
  String get confirm_delete;

  /// No description provided for @confirm_delete_map.
  ///
  /// In zh, this message translates to:
  /// **'确定要删除地图「{name}」吗？此操作不可恢复。'**
  String confirm_delete_map(Object name);

  /// No description provided for @map_deleted.
  ///
  /// In zh, this message translates to:
  /// **'已删除: {name}'**
  String map_deleted(Object name);

  /// No description provided for @delete_failed.
  ///
  /// In zh, this message translates to:
  /// **'删除失败: {e}'**
  String delete_failed(Object e);

  /// No description provided for @position_label.
  ///
  /// In zh, this message translates to:
  /// **'位置'**
  String get position_label;

  /// No description provided for @position_format.
  ///
  /// In zh, this message translates to:
  /// **'位置: ({x}, {y})'**
  String position_format(Object x, Object y);

  /// No description provided for @init_error.
  ///
  /// In zh, this message translates to:
  /// **'发生错误：{error}'**
  String init_error(Object error);

  /// No description provided for @diagnostic_warning.
  ///
  /// In zh, this message translates to:
  /// **'警告'**
  String get diagnostic_warning;

  /// No description provided for @diagnostic_error.
  ///
  /// In zh, this message translates to:
  /// **'错误'**
  String get diagnostic_error;

  /// No description provided for @diagnostic_stale.
  ///
  /// In zh, this message translates to:
  /// **'失活'**
  String get diagnostic_stale;

  /// No description provided for @diagnostic_normal.
  ///
  /// In zh, this message translates to:
  /// **'正常'**
  String get diagnostic_normal;

  /// No description provided for @diagnostic_health.
  ///
  /// In zh, this message translates to:
  /// **'健康诊断:[{level}] {component}'**
  String diagnostic_health(Object level, Object component);

  /// No description provided for @diagnostic_hardware.
  ///
  /// In zh, this message translates to:
  /// **'硬件ID: {id}\n消息: {msg}'**
  String diagnostic_hardware(Object id, Object msg);

  /// No description provided for @error_count.
  ///
  /// In zh, this message translates to:
  /// **'错误: {count}'**
  String error_count(Object count);

  /// No description provided for @warn_count.
  ///
  /// In zh, this message translates to:
  /// **'警告: {count}'**
  String warn_count(Object count);

  /// No description provided for @nav_point_info.
  ///
  /// In zh, this message translates to:
  /// **'导航点信息'**
  String get nav_point_info;

  /// No description provided for @position_coords.
  ///
  /// In zh, this message translates to:
  /// **'位置坐标'**
  String get position_coords;

  /// No description provided for @coord_x.
  ///
  /// In zh, this message translates to:
  /// **'X坐标'**
  String get coord_x;

  /// No description provided for @coord_y.
  ///
  /// In zh, this message translates to:
  /// **'Y坐标'**
  String get coord_y;

  /// No description provided for @heading.
  ///
  /// In zh, this message translates to:
  /// **'方向'**
  String get heading;

  /// No description provided for @stop_manual_first.
  ///
  /// In zh, this message translates to:
  /// **'请先停止手动控制'**
  String get stop_manual_first;

  /// No description provided for @nav_goal_sent.
  ///
  /// In zh, this message translates to:
  /// **'已发送导航目标到 {name}'**
  String nav_goal_sent(Object name);

  /// No description provided for @send_nav_goal.
  ///
  /// In zh, this message translates to:
  /// **'发送导航目标'**
  String get send_nav_goal;

  /// No description provided for @emergency_stopped.
  ///
  /// In zh, this message translates to:
  /// **'急停已触发'**
  String get emergency_stopped;

  /// No description provided for @nav_stopped.
  ///
  /// In zh, this message translates to:
  /// **'导航已停止'**
  String get nav_stopped;

  /// No description provided for @legend_free.
  ///
  /// In zh, this message translates to:
  /// **'自由'**
  String get legend_free;

  /// No description provided for @legend_occupied.
  ///
  /// In zh, this message translates to:
  /// **'障碍'**
  String get legend_occupied;

  /// No description provided for @legend_unknown.
  ///
  /// In zh, this message translates to:
  /// **'未知'**
  String get legend_unknown;

  /// No description provided for @nav_goal.
  ///
  /// In zh, this message translates to:
  /// **'导航目标'**
  String get nav_goal;

  /// No description provided for @charge_station.
  ///
  /// In zh, this message translates to:
  /// **'充电站'**
  String get charge_station;

  /// No description provided for @layer_trace.
  ///
  /// In zh, this message translates to:
  /// **'轨迹路径'**
  String get layer_trace;

  /// No description provided for @layer_robot_footprint.
  ///
  /// In zh, this message translates to:
  /// **'机器人轮廓'**
  String get layer_robot_footprint;

  /// No description provided for @camera_image.
  ///
  /// In zh, this message translates to:
  /// **'相机图像'**
  String get camera_image;

  /// No description provided for @no_map_available.
  ///
  /// In zh, this message translates to:
  /// **'当前无地图可用，请先选择或创建地图'**
  String get no_map_available;

  /// No description provided for @invalid_json.
  ///
  /// In zh, this message translates to:
  /// **'导入失败：无效的JSON格式'**
  String get invalid_json;

  /// No description provided for @data_stale.
  ///
  /// In zh, this message translates to:
  /// **'超过5s未更新数据'**
  String get data_stale;

  /// No description provided for @unknown_hardware.
  ///
  /// In zh, this message translates to:
  /// **'未知硬件'**
  String get unknown_hardware;

  /// No description provided for @status.
  ///
  /// In zh, this message translates to:
  /// **'状态'**
  String get status;

  /// No description provided for @update_time.
  ///
  /// In zh, this message translates to:
  /// **'更新时间'**
  String get update_time;

  /// No description provided for @no_detail.
  ///
  /// In zh, this message translates to:
  /// **'暂无详细信息'**
  String get no_detail;

  /// No description provided for @last_update.
  ///
  /// In zh, this message translates to:
  /// **'最后更新'**
  String get last_update;

  /// No description provided for @no_diagnostic_data.
  ///
  /// In zh, this message translates to:
  /// **'暂无诊断数据'**
  String get no_diagnostic_data;

  /// No description provided for @no_matching_diagnostic.
  ///
  /// In zh, this message translates to:
  /// **'没有找到匹配的诊断数据'**
  String get no_matching_diagnostic;

  /// No description provided for @all.
  ///
  /// In zh, this message translates to:
  /// **'全部'**
  String get all;

  /// No description provided for @clear_filter.
  ///
  /// In zh, this message translates to:
  /// **'清除筛选'**
  String get clear_filter;

  /// No description provided for @show_hardware_count.
  ///
  /// In zh, this message translates to:
  /// **'显示 {count} 个硬件组'**
  String show_hardware_count(Object count);

  /// No description provided for @clear_all_filter.
  ///
  /// In zh, this message translates to:
  /// **'清除所有筛选'**
  String get clear_all_filter;

  /// No description provided for @component_count.
  ///
  /// In zh, this message translates to:
  /// **'组件数: {count}'**
  String component_count(Object count);

  /// No description provided for @system_diagnostic.
  ///
  /// In zh, this message translates to:
  /// **'系统诊断'**
  String get system_diagnostic;

  /// No description provided for @refresh.
  ///
  /// In zh, this message translates to:
  /// **'刷新'**
  String get refresh;

  /// No description provided for @search_component_hint.
  ///
  /// In zh, this message translates to:
  /// **'搜索组件名称...'**
  String get search_component_hint;

  /// No description provided for @status_filter.
  ///
  /// In zh, this message translates to:
  /// **'状态筛选'**
  String get status_filter;

  /// No description provided for @table_key.
  ///
  /// In zh, this message translates to:
  /// **'键'**
  String get table_key;

  /// No description provided for @table_value.
  ///
  /// In zh, this message translates to:
  /// **'值'**
  String get table_value;

  /// No description provided for @detail_info.
  ///
  /// In zh, this message translates to:
  /// **'详细信息'**
  String get detail_info;

  /// No description provided for @diagnostic_overview.
  ///
  /// In zh, this message translates to:
  /// **'诊断状态总览'**
  String get diagnostic_overview;

  /// No description provided for @hardware_id.
  ///
  /// In zh, this message translates to:
  /// **'硬件ID'**
  String get hardware_id;
}

class _AppLocalizationsDelegate
    extends LocalizationsDelegate<AppLocalizations> {
  const _AppLocalizationsDelegate();

  @override
  Future<AppLocalizations> load(Locale locale) {
    return SynchronousFuture<AppLocalizations>(lookupAppLocalizations(locale));
  }

  @override
  bool isSupported(Locale locale) =>
      <String>['en', 'zh'].contains(locale.languageCode);

  @override
  bool shouldReload(_AppLocalizationsDelegate old) => false;
}

AppLocalizations lookupAppLocalizations(Locale locale) {
  // Lookup logic when only language code is specified.
  switch (locale.languageCode) {
    case 'en':
      return AppLocalizationsEn();
    case 'zh':
      return AppLocalizationsZh();
  }

  throw FlutterError(
    'AppLocalizations.delegate failed to load unsupported locale "$locale". This is likely '
    'an issue with the localizations generation tool. Please file an issue '
    'on GitHub with a reproducible sample app and the gen-l10n configuration '
    'that was used.',
  );
}
