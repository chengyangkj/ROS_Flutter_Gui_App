import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:package_info_plus/package_info_plus.dart';

class SettingsPage extends StatefulWidget {
  const SettingsPage({super.key});

  @override
  _SettingsPageState createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  final Map<String, String> _settings = {};
  late RobotType _selectedRobotType;
  String _version = '';

  @override
  void initState() {
    super.initState();
    _loadSettings();
    _loadVersion();
    String initValue = globalSetting.getConfig('init');
    _selectedRobotType = RobotType.values.firstWhere(
      (type) => type.value == initValue,
      orElse: () => RobotType.ROS2Default,
    );
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
          title: Text('编辑 ${_getDisplayName(key)}'),
          content: TextField(
            controller: controller,
            decoration: InputDecoration(
              hintText: '请输入${_getDisplayName(key)}',
              border: const OutlineInputBorder(),
            ),
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.pop(context),
              child: const Text('取消'),
            ),
            TextButton(
              onPressed: () async {
                await _saveSettings(key, controller.text);
                Navigator.pop(context);
                if (context.mounted) {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(
                      content: Text('设置已保存，重启应用后生效'),
                      duration: Duration(seconds: 2),
                    ),
                  );
                }
              },
              child: const Text('确定'),
            ),
          ],
        ),
      ),
    );
  }

  String _getDisplayName(String key) {
    final Map<String, String> displayNames = {
      'robotIp': 'IP地址',
      'robotPort': '端口',
      'mapTopic': '地图话题',
      'laserTopic': '激光雷达话题',
      'globalPathTopic': '全局路径话题',
      'localPathTopic': '局部路径话题',
      'relocTopic': '重定位话题',
      'navGoalTopic': '导航目标话题',
      'OdometryTopic': '里程计话题',
      'SpeedCtrlTopic': '速度控制话题',
      'BatteryTopic': '电池状态话题',
      'MaxVx': '最大前进速度',
      'MaxVy': '最大横向速度',
      'MaxVw': '最大旋转速度',
      'mapFrameName': '地图坐标系',
      'baseLinkFrameName': '机器人坐标系',
      'imagePort': '图像端口',
      'imageTopic': '图像话题',
      'imageWidth': '图像宽度',
      'imageHeight': '图像高度',
    };
    return displayNames[key] ?? key;
  }

  Future<void> _saveSettings(String key, String value) async {
    final prefs = await SharedPreferences.getInstance();

    if (key == "tempConfig" && value.isNotEmpty) {
      await prefs.setString(key, value);
      await initGlobalSetting();
    } else {
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
      }
    }

    _loadSettings();
  }

  Future<void> _applyTemplate(RobotType type) async {
    switch (type) {
      case RobotType.ROS2Default:
        globalSetting.setDefaultCfgRos2();
        break;
      case RobotType.ROS1:
        globalSetting.setDefaultCfgRos1();
        break;
      case RobotType.TurtleBot3:
        globalSetting.setDefaultCfgRos2TB3();
        break;
      case RobotType.TurtleBot4:
        globalSetting.setDefaultCfgRos2TB4();
        break;
      case RobotType.Jackal:
        globalSetting.setDefaultCfgRos2Jackal();
        break;
    }
    setState(() {
      _selectedRobotType = type;
    });
  }

  List<Widget> _buildSettingGroups() {
    return [
      _buildRobotTypeSection(),
      _buildBasicSection(),
      _buildTopicSection(),
      // ... 其他设置组
    ];
  }

  Widget _buildRobotTypeSection() {
    return _buildSection(
      "机器人类型",
      [
        ListTile(
          title: const Text("默认配置模板"),
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
              child: DropdownButton<RobotType>(
                value: _selectedRobotType,
                dropdownColor: Theme.of(context).brightness == Brightness.dark
                    ? Colors.grey[800]
                    : Colors.white,
                borderRadius: BorderRadius.circular(8),
                elevation: 8,
                isDense: true,
                icon: const Icon(Icons.arrow_drop_down),
                onChanged: (RobotType? newValue) {
                  if (newValue != null) {
                    showDialog(
                      context: context,
                      builder: (context) => AlertDialog(
                        title: const Text("确认更改"),
                        content: const Text("切换配置模版板将会重置所有配置，是否继续？"),
                        actions: [
                          TextButton(
                            onPressed: () => Navigator.pop(context),
                            child: const Text("取消"),
                          ),
                          TextButton(
                            onPressed: () async {
                              Navigator.pop(context);
                              await _applyTemplate(newValue);
                              _loadSettings();
                              setState(() {});
                            },
                            child: const Text("确定"),
                          ),
                        ],
                      ),
                    );
                  }
                },
                items: RobotType.values
                    .map<DropdownMenuItem<RobotType>>((RobotType type) {
                  return DropdownMenuItem<RobotType>(
                    value: type,
                    child: Text(
                      type.displayName,
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
              Text(
                _getDisplayName(key),
                style: TextStyle(
                  fontSize: 16,
                  color: isDarkMode ? Colors.white : Colors.black,
                ),
              ),
              Row(
                children: [
                  Text(
                    value,
                    style: TextStyle(
                      color: isDarkMode ? Colors.grey[400] : Colors.grey,
                      fontSize: 16,
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
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildBasicSection() {
    return _buildSection(
      "基础设置",
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
      "话题设置",
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
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final isDarkMode = theme.brightness == Brightness.dark;

    return Scaffold(
      backgroundColor: isDarkMode ? Colors.grey[900] : Colors.grey[100],
      appBar: AppBar(
        title: const Text('设置'),
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
