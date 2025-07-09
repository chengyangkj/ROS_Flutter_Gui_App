import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/main.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:package_info_plus/package_info_plus.dart';
import 'gamepad_mapping_page.dart';
import 'package:flutter/services.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class SettingsPage extends StatefulWidget {
  const SettingsPage({super.key});

  @override
  _SettingsPageState createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  final Map<String, String> _settings = {};
  late TempConfigType _selectedTempConfigType;
  String _version = '';
  Orientation _selectedOrientation = Orientation.landscape;

  @override
  void initState() {
    super.initState();
    _loadSettings();
    _loadVersion();
    _loadOrientation();
    _selectedTempConfigType = globalSetting.tempConfig;
  }

  Future<void> _loadVersion() async {
    final packageInfo = await PackageInfo.fromPlatform();
    setState(() {
      _version = packageInfo.version;
    });
  }

  void _loadSettings() async {
    final prefs = await SharedPreferences.getInstance();
    final keys = prefs.getKeys();
    final Map<String, String> newSettings = {};

    for (String key in keys) {
      final value = prefs.get(key);
      newSettings[key] = value?.toString() ?? '';
    }

    setState(() {
      _settings.addAll(newSettings);
    });
  }

  Future<void> _showEditDialog(String key, String currentValue) async {
    final controller = TextEditingController(text: currentValue);
    return showDialog(
      context: context,
      barrierColor: Colors.black54,
      builder: (context) => Theme(
        data: Theme.of(context).copyWith(
          dialogBackgroundColor:
              Theme.of(context).dialogBackgroundColor.withOpacity(1.0),
        ),
        child: AlertDialog(
          title: Text('Edit ${_getDisplayName(key)}'),
          content: TextField(
            controller: controller,
            decoration: InputDecoration(
              hintText: 'Please input ${_getDisplayName(key)}',
              border: const OutlineInputBorder(),
            ),
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.pop(context),
              child: Text(AppLocalizations.of(context)!.cancel),
            ),
            TextButton(
              onPressed: () async {
                await _saveSettings(key, controller.text);
                Navigator.pop(context);
                if (context.mounted) {
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: Text(AppLocalizations.of(context)!.config_saved),
                      duration: Duration(seconds: 2),
                    ),
                  );
                }
              },
              child: Text(AppLocalizations.of(context)!.ok),
            ),
          ],
        ),
      ),
    );
  }

  String _getDisplayName(String key) {
    final Map<String, String> displayNames = {
      'robotIp': AppLocalizations.of(context)!.ip_address,
      'robotPort': AppLocalizations.of(context)!.port,
      'mapTopic': AppLocalizations.of(context)!.map_topic,
      'laserTopic': AppLocalizations.of(context)!.laser_topic,
      'globalPathTopic': AppLocalizations.of(context)!.global_path_topic,
      'localPathTopic': AppLocalizations.of(context)!.local_path_topic,
      'relocTopic': AppLocalizations.of(context)!.reloc_topic,
      'navGoalTopic': AppLocalizations.of(context)!.nav_goal_topic,
      'OdometryTopic': AppLocalizations.of(context)!.odometry_topic,
      'SpeedCtrlTopic': AppLocalizations.of(context)!.speed_ctrl_topic,
      'BatteryTopic': AppLocalizations.of(context)!.battery_topic,
      'MaxVx': AppLocalizations.of(context)!.max_speed,
      'MaxVy': AppLocalizations.of(context)!.max_y_speed,
      'MaxVw': AppLocalizations.of(context)!.max_angular_speed,
      'mapFrameName': AppLocalizations.of(context)!.map_frame,
      'baseLinkFrameName': AppLocalizations.of(context)!.base_frame,
      'imagePort': AppLocalizations.of(context)!.image_port,
      'imageTopic': AppLocalizations.of(context)!.image_topic,
      'imageWidth': AppLocalizations.of(context)!.image_width,
      'imageHeight': AppLocalizations.of(context)!.image_height,
      'robotFootprintTopic': AppLocalizations.of(context)!.robot_footprint_topic,
      'localCostmapTopic': AppLocalizations.of(context)!.local_cost_map_topic,
      'pointCloud2Topic': AppLocalizations.of(context)!.pointcloud2_topic,
      'globalCostmapTopic': AppLocalizations.of(context)!.global_costmap_topic,
    };
    return displayNames[key] ?? key;
  }

  Future<void> _saveSettings(String key, String value) async {
    final prefs = await SharedPreferences.getInstance();

    await prefs.setString(key, value);
    switch (key) {
      case 'robotIp':
        globalSetting.setRobotIp(value);
        break;
      case 'robotPort':
        globalSetting.setRobotPort(value);
        break;
      case 'mapTopic':
        globalSetting.setMapTopic(value);
        break;
      case 'laserTopic':
        globalSetting.setLaserTopic(value);
        break;
      case 'globalPathTopic':
        globalSetting.setGlobalPathTopic(value);
        break;
      case 'localPathTopic':
        globalSetting.setLocalPathTopic(value);
        break;
      case 'relocTopic':
        globalSetting.setRelocTopic(value);
        break;
      case 'navGoalTopic':
        globalSetting.setNavGoalTopic(value);
        break;
      case 'OdometryTopic':
        globalSetting.setOdomTopic(value);
        break;
      case 'SpeedCtrlTopic':
        globalSetting.setSpeedCtrlTopic(value);
        break;
      case 'BatteryTopic':
        globalSetting.setBatteryTopic(value);
        break;
      case 'MaxVx':
        globalSetting.setMaxVx(value);
        break;
      case 'MaxVy':
        globalSetting.setMaxVy(value);
        break;
      case 'MaxVw':
        globalSetting.setMaxVw(value);
        break;
      case 'mapFrameName':
        globalSetting.setMapFrameName(value);
        break;
      case 'baseLinkFrameName':
        globalSetting.setBaseLinkFrameName(value);
        break;
      case 'imagePort':
        globalSetting.setImagePort(value);
        break;
      case 'imageTopic':
        globalSetting.setImageTopic(value);
        break;
      case 'imageWidth':
        globalSetting.setImageWidth(double.parse(value));
        break;
      case 'imageHeight':
        globalSetting.setImageHeight(double.parse(value));
        break;
      case 'robotFootprintTopic':
        globalSetting.setRobotFootprintTopic(value);
        break;
      case 'localCostmapTopic':
        globalSetting.setLocalCostmapTopic(value);
        break;
      case 'globalCostmapTopic':
        globalSetting.setGlobalCostmapTopic(value);
        break;
      case 'pointCloud2Topic':
        globalSetting.setPointCloud2Topic(value);
        break;
    }

    _loadSettings();
  }

  Future<void> _applyTemplate(TempConfigType type) async {
    switch (type) {
      case TempConfigType.ROS2Default:
        globalSetting.setDefaultCfgRos2();
        break;
      case TempConfigType.ROS1:
        globalSetting.setDefaultCfgRos1();
        break;
      case TempConfigType.TurtleBot3:
        globalSetting.setDefaultCfgRos2TB3();
        break;
      case TempConfigType.TurtleBot4:
        globalSetting.setDefaultCfgRos2TB4();
        break;
      case TempConfigType.Jackal:
        globalSetting.setDefaultCfgRos2Jackal();
        break;
    }
    setState(() {
      _selectedTempConfigType = type;
    });
  }

  List<Widget> _buildSettingGroups() {
    return [
      _buildLanguageSection(),
      _buildTempConfigTypeSection(),
      _buildBasicSection(),
      _buildTopicSection(),
      _buildOrientationSection(),
      // ... 其他设置组
    ];
  }

  Widget _buildLanguageSection() {
    return _buildSection(
      AppLocalizations.of(context)!.language,
      [
        ListTile(
          title: Text(AppLocalizations.of(context)!.language),
          trailing: DropdownButton<String>(
            value: _settings['language'] == 'zh'
                ? AppLocalizations.of(context)!.zh
                : AppLocalizations.of(context)!.en,
            dropdownColor: Theme.of(context).brightness == Brightness.dark
                ? Colors.grey[800]
                : Colors.white,
            onChanged: (String? newValue) {
              if (newValue != null) {
                setState(() {
                  if (newValue == AppLocalizations.of(context)!.en) {
                    _settings['language'] = 'en';
                    _saveLanguage('en');
                    globalSetting.setLanguage(Locale('en'));
                  } else if (newValue == AppLocalizations.of(context)!.zh) {
                    _settings['language'] = 'zh';
                    _saveLanguage('zh');
                    globalSetting.setLanguage(Locale('zh'));
                  }
                });
              }
            },
            items: [
              AppLocalizations.of(context)!.en,
              AppLocalizations.of(context)!.zh
            ].map<DropdownMenuItem<String>>((String language) {
              return DropdownMenuItem<String>(
                value: language,
                child: Text(language),
              );
            }).toList(),
          ),
        ),
      ],
    );
  }

  Future<void> _saveLanguage(String language) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('language', language);
  }

  Widget _buildTempConfigTypeSection() {
    return _buildSection(
      AppLocalizations.of(context)!.robot_type,
      [
        ListTile(
          title: Text(AppLocalizations.of(context)!.default_config_template),
          trailing: Container(
            decoration: BoxDecoration(
              color: Theme.of(context).brightness == Brightness.dark
                  ? Colors.grey[800]
                  : Colors.white,
              border: Border.all(
                color: Colors.grey.withOpacity(0.2),
              ),
              borderRadius: BorderRadius.circular(8),
            ),
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 0),
            child: DropdownButtonHideUnderline(
              child: DropdownButton<TempConfigType>(
                value: _selectedTempConfigType,
                dropdownColor: Theme.of(context).brightness == Brightness.dark
                    ? Colors.grey[800]
                    : Colors.white,
                borderRadius: BorderRadius.circular(8),
                elevation: 8,
                isDense: true,
                icon: const Icon(Icons.arrow_drop_down),
                onChanged: (TempConfigType? newValue) {
                  if (newValue != null) {
                    showDialog(
                      context: context,
                      builder: (context) => AlertDialog(
                        title:
                            Text(AppLocalizations.of(context)!.confirm_change),
                        content: Text(AppLocalizations.of(context)!
                            .switch_template_will_reset_all_settings),
                        actions: [
                          TextButton(
                            onPressed: () => Navigator.pop(context),
                            child: Text(AppLocalizations.of(context)!.cancel),
                          ),
                          TextButton(
                            onPressed: () async {
                              Navigator.pop(context);
                              await _applyTemplate(newValue);
                              _loadSettings();
                              setState(() {});
                            },
                            child: Text(AppLocalizations.of(context)!.ok),
                          ),
                        ],
                      ),
                    );
                  }
                },
                items: TempConfigType.values
                    .map<DropdownMenuItem<TempConfigType>>(
                        (TempConfigType type) {
                  return DropdownMenuItem<TempConfigType>(
                    value: type,
                    child: Text(
                      tempConfigTypeToString(type),
                      style: TextStyle(
                        color: Theme.of(context).brightness == Brightness.dark
                            ? Colors.white
                            : Colors.black87,
                      ),
                    ),
                  );
                }).toList(),
              ),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildSection(String title, List<Widget> children) {
    final theme = Theme.of(context);
    final isDarkMode = theme.brightness == Brightness.dark;

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Padding(
          padding: const EdgeInsets.fromLTRB(16, 16, 16, 8),
          child: Text(
            title,
            style: TextStyle(
              color: isDarkMode ? Colors.grey[300] : Colors.grey,
              fontSize: 13,
              fontWeight: FontWeight.w500,
            ),
          ),
        ),
        Container(
          margin: const EdgeInsets.symmetric(horizontal: 16),
          decoration: BoxDecoration(
            color: isDarkMode ? Colors.grey[800] : Colors.white,
            borderRadius: BorderRadius.circular(10),
            border: Border.all(color: Colors.grey.withOpacity(0.2)),
          ),
          child: Column(children: children),
        ),
      ],
    );
  }

  Widget _buildSettingItem(String key, String value) {
    final theme = Theme.of(context);
    final isDarkMode = theme.brightness == Brightness.dark;

    return Material(
      color: Colors.transparent,
      child: InkWell(
        onTap: () => _showEditDialog(key, value),
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
          decoration: BoxDecoration(
            border: Border(
              bottom: BorderSide(
                color: Colors.grey.withOpacity(0.2),
                width: 0.5,
              ),
            ),
          ),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Expanded(
                child: Text(
                  _getDisplayName(key),
                  style: TextStyle(
                    fontSize: 16,
                    color: isDarkMode ? Colors.white : Colors.black,
                  ),
                ),
              ),
              Flexible(
                child: Text(
                  value,
                  style: TextStyle(
                    color: isDarkMode ? Colors.grey[400] : Colors.grey,
                    fontSize: 16,
                  ),
                  overflow: TextOverflow.ellipsis,
                  maxLines: 1,
                  textAlign: TextAlign.right,
                ),
              ),
              const SizedBox(width: 8),
              Icon(
                Icons.chevron_right,
                color: isDarkMode ? Colors.grey[400] : Colors.grey,
                size: 20,
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildBasicSection() {
    return _buildSection(
      AppLocalizations.of(context)!.basic_setting,
      [
        _buildSettingItem('robotIp', _settings['robotIp'] ?? '127.0.0.1'),
        _buildSettingItem('robotPort', _settings['robotPort'] ?? '9090'),
        _buildSettingItem('MaxVx', _settings['MaxVx'] ?? '0.1'),
        _buildSettingItem('MaxVy', _settings['MaxVy'] ?? '0.1'),
        _buildSettingItem('MaxVw', _settings['MaxVw'] ?? '0.3'),
        _buildSettingItem('mapFrameName', _settings['mapFrameName'] ?? 'map'),
        _buildSettingItem(
            'baseLinkFrameName', _settings['baseLinkFrameName'] ?? 'base_link'),
        _buildSettingItem('imagePort', _settings['imagePort'] ?? '8080'),
        _buildSettingItem(
            'imageWidth', _settings['imageWidth']?.toString() ?? '640'),
        _buildSettingItem(
            'imageHeight', _settings['imageHeight']?.toString() ?? '480'),
      ],
    );
  }

  Widget _buildTopicSection() {
    return _buildSection(
      AppLocalizations.of(context)!.topic_setting,
      [
        _buildSettingItem('mapTopic', _settings['mapTopic'] ?? 'map'),
        _buildSettingItem('laserTopic', _settings['laserTopic'] ?? 'scan'),
        _buildSettingItem(
            'globalPathTopic', _settings['globalPathTopic'] ?? '/plan'),
        _buildSettingItem(
            'localPathTopic', _settings['localPathTopic'] ?? '/local_plan'),
        _buildSettingItem(
            'relocTopic', _settings['relocTopic'] ?? '/initialpose'),
        _buildSettingItem(
            'navGoalTopic', _settings['navGoalTopic'] ?? '/goal_pose'),
        _buildSettingItem(
            'OdometryTopic', _settings['OdometryTopic'] ?? '/wheel/odometry'),
        _buildSettingItem(
            'SpeedCtrlTopic', _settings['SpeedCtrlTopic'] ?? '/cmd_vel'),
        _buildSettingItem(
            'BatteryTopic', _settings['BatteryTopic'] ?? '/battery_status'),
        _buildSettingItem(
            'imageTopic', _settings['imageTopic'] ?? '/camera/image_raw'),
        _buildSettingItem(
            'robotFootprintTopic', _settings['robotFootprintTopic'] ?? '/local_costmap/published_footprint'),
        _buildSettingItem(
            'localCostmapTopic', _settings['localCostmapTopic'] ?? '/local_costmap/costmap'),
        _buildSettingItem(
            'globalCostmapTopic', _settings['globalCostmapTopic'] ?? '/global_costmap/costmap'),
        _buildSettingItem(
            'pointCloud2Topic', _settings['pointCloud2Topic'] ?? '/pointcloud2'),
      ],
    );
  }

  Widget _buildOrientationSection() {
    return _buildSection(
      AppLocalizations.of(context)!.app_setting,
      [
        ListTile(
          title: Text(AppLocalizations.of(context)!.screen_orientation),
          trailing: DropdownButton<Orientation>(
            value: _selectedOrientation,
            dropdownColor: Theme.of(context).brightness == Brightness.dark
                ? Colors.grey[800]
                : Colors.white,
            onChanged: (Orientation? newValue) {
              if (newValue != null) {
                setState(() {
                  _selectedOrientation = newValue;
                  _setOrientation(newValue);
                  _saveOrientation(newValue);
                });
              }
            },
            items: Orientation.values
                .map<DropdownMenuItem<Orientation>>((Orientation orientation) {
              return DropdownMenuItem<Orientation>(
                value: orientation,
                child: Text(orientation == Orientation.portrait
                    ? AppLocalizations.of(context)!.portrait
                    : AppLocalizations.of(context)!.landscape),
              );
            }).toList(),
          ),
        ),
      ],
    );
  }

  Future<void> _loadOrientation() async {
    final prefs = await SharedPreferences.getInstance();
    final orientationValue =
        prefs.getString('screenOrientation') ?? 'landscape';
    _selectedOrientation = orientationValue == 'portrait'
        ? Orientation.portrait
        : Orientation.landscape;
    _setOrientation(_selectedOrientation);
  }

  void _setOrientation(Orientation orientation) {
    if (orientation == Orientation.portrait) {
      SystemChrome.setPreferredOrientations([
        DeviceOrientation.portraitUp,
        DeviceOrientation.portraitDown,
      ]);
    } else {
      SystemChrome.setPreferredOrientations([
        DeviceOrientation.landscapeLeft,
        DeviceOrientation.landscapeRight,
      ]);
    }
  }

  Future<void> _saveOrientation(Orientation orientation) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('screenOrientation',
        orientation == Orientation.portrait ? 'portrait' : 'landscape');
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final isDarkMode = theme.brightness == Brightness.dark;

    return Scaffold(
      backgroundColor: isDarkMode ? Colors.grey[900] : Colors.grey[100],
      appBar: AppBar(
        title: Text(AppLocalizations.of(context)!.setting),
        elevation: 0,
        actions: [
          Center(
            child: Padding(
              padding: const EdgeInsets.only(right: 16.0),
              child: Text(
                'v$_version',
                style: const TextStyle(fontSize: 14),
              ),
            ),
          ),
        ],
      ),
      body: ListView(
        children: _buildSettingGroups(),
      ),
    );
  }
}
