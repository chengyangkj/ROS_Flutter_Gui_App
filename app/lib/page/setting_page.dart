import 'package:flutter/material.dart';

import 'package:shared_preferences/shared_preferences.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/http_channel.dart';
import 'package:package_info_plus/package_info_plus.dart';

import 'package:flutter/services.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';
import 'package:ros_flutter_gui_app/page/layer_settings_panel.dart';
import 'package:ros_flutter_gui_app/page/ssh_widgets.dart';

const String kSettingsRouteArgLayers = 'layers';

class SettingsPage extends StatefulWidget {
  const SettingsPage({super.key, this.scrollToLayerSection = false});

  final bool scrollToLayerSection;

  @override
  _SettingsPageState createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  final Map<String, String> _settings = {};
  final GlobalKey _layersSectionKey = GlobalKey();
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
    if (widget.scrollToLayerSection) {
      _scheduleScrollToLayersSection();
    }
  }

  void _scheduleScrollToLayersSection() {
    void run() {
      if (!mounted) return;
      _scrollToLayersSection();
    }

    WidgetsBinding.instance.addPostFrameCallback((_) {
      run();
      WidgetsBinding.instance.addPostFrameCallback((_) => run());
    });
  }

  void _scrollToLayersSection() {
    if (!mounted) return;
    final ctx = _layersSectionKey.currentContext;
    if (ctx == null) return;
    Scrollable.ensureVisible(
      ctx,
      duration: const Duration(milliseconds: 360),
      curve: Curves.easeOutCubic,
      alignment: 0.0,
      alignmentPolicy: ScrollPositionAlignmentPolicy.explicit,
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
      'httpServerPort': AppLocalizations.of(context)!.http_server_port,
      'mapTopic': AppLocalizations.of(context)!.map_topic,
      'laserTopic': AppLocalizations.of(context)!.laser_topic,
      'globalPathTopic': AppLocalizations.of(context)!.global_path_topic,
      'localPathTopic': AppLocalizations.of(context)!.local_path_topic,
      'tracePathTopic': AppLocalizations.of(context)!.trace_path_topic,
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
      'robotSize': AppLocalizations.of(context)!.robot_size,
    };
    return displayNames[key] ?? key;
  }

  Future<void> _saveSettings(String key, String value) async {
    final prefs = await SharedPreferences.getInstance();

    if (key == 'httpServerPort') {
      final trimmed = value.trim();
      if (trimmed.isEmpty || trimmed == '7684') {
        globalSetting.setHttpServerPort('');
      } else {
        final n = int.tryParse(trimmed);
        if (n != null && n >= 1 && n <= 65535) {
          globalSetting.setHttpServerPort(trimmed);
        }
      }
      _loadSettings();
      return;
    }

    if (Setting.backendGuiStorageKeys.contains(key)) {
      try {
        await HttpChannel().saveGuiSettings({key: value});
        globalSetting.patchBackendGuiSetting(key, value);
      } catch (e) {
        if (mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(
              content: Text('$e'),
              duration: const Duration(seconds: 3),
            ),
          );
        }
        return;
      }
      _loadSettings();
      return;
    }

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
      case 'MaxVx':
        globalSetting.setMaxVx(value);
        break;
      case 'MaxVy':
        globalSetting.setMaxVy(value);
        break;
      case 'MaxVw':
        globalSetting.setMaxVw(value);
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
      case 'robotSize':
        globalSetting.setRobotSize(double.parse(value));
        break;
    }

    _loadSettings();
  }

  Future<void> _applyTemplate(TempConfigType type) async {
    switch (type) {
      case TempConfigType.ROS2:
        globalSetting.setDefaultCfgRos2();
        break;
      case TempConfigType.ROS1:
        globalSetting.setDefaultCfgRos1();
        break;
    }
    try {
      await HttpChannel()
          .saveGuiSettings(globalSetting.buildBackendGuiSettingsJson());
    } catch (_) {}
    setState(() {
      _selectedTempConfigType = type;
    });
  }

  List<Widget> _buildSettingGroups() {
    return [
      _buildLanguageSection(),
      _buildTempConfigTypeSection(),
      _buildBasicSection(),
      _buildSshSection(),
      _buildLayersSection(),
      _buildOtherTopicsSection(),
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
            dropdownColor: Colors.white,
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

  Widget _buildSshSection() {
    final l10n = AppLocalizations.of(context)!;
    return _buildSection(
      l10n.ssh_remote_section,
      [
        ListTile(
          title: Text(l10n.ssh_config_list_tile_title),
          subtitle: Text(
            globalSetting.sshCredentialsConfigured
                ? l10n.ssh_user_at_host_port(
                    globalSetting.sshUsername.trim(),
                    globalSetting.robotIp.trim(),
                    globalSetting.sshPort,
                  )
                : l10n.ssh_not_configured_hint,
          ),
          trailing: const Icon(Icons.keyboard_arrow_right),
          onTap: () => ShowSshConfigSheet(context),
        ),
      ],
    );
  }

  Widget _buildTempConfigTypeSection() {
    return _buildSection(
      AppLocalizations.of(context)!.robot_type,
      [
        ListTile(
          title: Text(AppLocalizations.of(context)!.default_config_template),
          trailing: Container(
            decoration: BoxDecoration(
              color: Colors.white,
              border: Border.all(
                color: Colors.grey.withOpacity(0.2),
              ),
              borderRadius: BorderRadius.circular(8),
            ),
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 0),
            child: DropdownButtonHideUnderline(
              child: DropdownButton<TempConfigType>(
                value: _selectedTempConfigType,
                dropdownColor: Colors.white,
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
                              if (context.mounted) {
                                _loadSettings();
                                setState(() {});
                              }
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
                        color: Colors.black87,
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
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Padding(
          padding: const EdgeInsets.fromLTRB(16, 16, 16, 8),
          child: Text(
            title,
            style: const TextStyle(
              color: Colors.grey,
              fontSize: 13,
              fontWeight: FontWeight.w500,
            ),
          ),
        ),
        Container(
          margin: const EdgeInsets.symmetric(horizontal: 16),
          decoration: BoxDecoration(
            color: Colors.white,
            borderRadius: BorderRadius.circular(10),
            border: Border.all(color: Colors.grey.withOpacity(0.2)),
          ),
          child: Column(children: children),
        ),
      ],
    );
  }

  Widget _buildSettingItem(String key, String value) {
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
                  style: const TextStyle(
                    fontSize: 16,
                    color: Colors.black,
                  ),
                ),
              ),
              Flexible(
                child: Text(
                  value,
                  style: const TextStyle(
                    color: Colors.grey,
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
                color: Colors.grey,
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
        _buildSettingItem('robotPort', _settings['robotPort'] ?? '7684'),
        _buildSettingItem('httpServerPort', _settings['httpServerPort'] ?? '7684'),
        _buildSettingItem('MaxVx', _settings['MaxVx'] ?? '0.1'),
        _buildSettingItem('MaxVy', _settings['MaxVy'] ?? '0.1'),
        _buildSettingItem('MaxVw', _settings['MaxVw'] ?? '0.3'),
        _buildSettingItem('robotSize', _settings['robotSize']?.toString() ?? '3.0'),
        _buildSettingItem('mapFrameName', globalSetting.mapFrameName),
        _buildSettingItem(
            'baseLinkFrameName', globalSetting.baseLinkFrameName),
        _buildSettingItem('imagePort', _settings['imagePort'] ?? '8080'),
        _buildSettingItem(
            'imageWidth', _settings['imageWidth']?.toString() ?? '640'),
        _buildSettingItem(
            'imageHeight', _settings['imageHeight']?.toString() ?? '480'),
      ],
    );
  }

  Widget _buildLayersSection() {
    return Column(
      key: _layersSectionKey,
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Padding(
          padding: const EdgeInsets.fromLTRB(16, 16, 16, 8),
          child: Text(
            AppLocalizations.of(context)!.layers,
            style: const TextStyle(
              color: Colors.grey,
              fontSize: 13,
              fontWeight: FontWeight.w500,
            ),
          ),
        ),
        Container(
          margin: const EdgeInsets.symmetric(horizontal: 16),
          decoration: BoxDecoration(
            color: Colors.white,
            borderRadius: BorderRadius.circular(10),
            border: Border.all(color: Colors.grey.withOpacity(0.2)),
          ),
          clipBehavior: Clip.antiAlias,
          child: const LayerSettingsPanel(),
        ),
      ],
    );
  }

  Widget _buildOtherTopicsSection() {
    return _buildSection(
      AppLocalizations.of(context)!.topic_setting,
      [
        _buildSettingItem('mapTopic', _settings['mapTopic'] ?? 'map'),
        _buildSettingItem('relocTopic', globalSetting.relocTopic),
        _buildSettingItem('navGoalTopic', globalSetting.navGoalTopic),
        _buildSettingItem('OdometryTopic', globalSetting.odomTopic),
        _buildSettingItem('SpeedCtrlTopic', globalSetting.speedCtrlTopic),
        _buildSettingItem('BatteryTopic', globalSetting.batteryTopic),
        _buildSettingItem(
            'imageTopic', _settings['imageTopic'] ?? '/image_raw'),
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
            dropdownColor: Colors.white,
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
    return Scaffold(
      backgroundColor: Colors.grey[100],
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
      body: SingleChildScrollView(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: _buildSettingGroups(),
        ),
      ),
    );
  }
}
