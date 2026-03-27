import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/http_channel.dart';

Future<void> saveGuiTopicToBackend(
    BuildContext context, String key, String raw) async {
  if (!Setting.backendGuiStorageKeys.contains(key)) return;
  final trimmed = raw.trim();
  try {
    await HttpChannel().saveGuiSettings({key: trimmed});
    globalSetting.patchBackendGuiSetting(key, trimmed);
    if (context.mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(AppLocalizations.of(context)!.config_saved),
          duration: const Duration(seconds: 2),
        ),
      );
    }
  } catch (e) {
    if (context.mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('$e'),
          duration: const Duration(seconds: 4),
        ),
      );
    }
  }
}

Future<Color?> pickPresetColor(BuildContext context, Color current) async {
  const presets = <Color>[
    Colors.red,
    Colors.deepOrange,
    Colors.orange,
    Colors.amber,
    Colors.yellow,
    Colors.lime,
    Colors.green,
    Colors.teal,
    Colors.cyan,
    Colors.lightBlue,
    Colors.blue,
    Colors.indigo,
    Colors.purple,
    Colors.pink,
    Colors.white,
    Colors.black54,
  ];
  return showDialog<Color>(
    context: context,
    builder: (ctx) => AlertDialog(
      title: Text(AppLocalizations.of(ctx)!.layers),
      content: SizedBox(
        width: 280,
        child: Wrap(
          spacing: 6,
          runSpacing: 6,
          children: presets.map((c) {
            return InkWell(
              onTap: () => Navigator.pop(ctx, c),
              child: Container(
                width: 32,
                height: 32,
                decoration: BoxDecoration(
                  color: c,
                  border: Border.all(color: Colors.grey),
                  borderRadius: BorderRadius.circular(4),
                ),
              ),
            );
          }).toList(),
        ),
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.pop(ctx),
          child: Text(AppLocalizations.of(ctx)!.cancel),
        ),
      ],
    ),
  );
}

class LayerSettingsPanel extends StatefulWidget {
  const LayerSettingsPanel({super.key});

  @override
  State<LayerSettingsPanel> createState() => _LayerSettingsPanelState();
}

class _LayerSettingsPanelState extends State<LayerSettingsPanel> {
  late final TextEditingController _laserTopic;
  late final TextEditingController _globalPathTopic;
  late final TextEditingController _localPathTopic;
  late final TextEditingController _tracePathTopic;
  late final TextEditingController _pointCloudTopic;
  late final TextEditingController _globalCostTopic;
  late final TextEditingController _localCostTopic;
  late final TextEditingController _footprintTopic;
  late double _laserDotDraft;

  final Set<String> _openLayerIds = <String>{};

  static const _dividerColor = Color(0x33000000);

  @override
  void initState() {
    super.initState();
    _laserDotDraft = Provider.of<GlobalState>(context, listen: false)
        .layerLaserDotRadius();
    _laserTopic = TextEditingController(text: globalSetting.laserTopic);
    _globalPathTopic =
        TextEditingController(text: globalSetting.globalPathTopic);
    _localPathTopic =
        TextEditingController(text: globalSetting.localPathTopic);
    _tracePathTopic =
        TextEditingController(text: globalSetting.tracePathTopic);
    _pointCloudTopic =
        TextEditingController(text: globalSetting.pointCloud2Topic);
    _globalCostTopic =
        TextEditingController(text: globalSetting.globalCostmapTopic);
    _localCostTopic =
        TextEditingController(text: globalSetting.localCostmapTopic);
    _footprintTopic =
        TextEditingController(text: globalSetting.robotFootprintTopic);
  }

  @override
  void dispose() {
    _laserTopic.dispose();
    _globalPathTopic.dispose();
    _localPathTopic.dispose();
    _tracePathTopic.dispose();
    _pointCloudTopic.dispose();
    _globalCostTopic.dispose();
    _localCostTopic.dispose();
    _footprintTopic.dispose();
    super.dispose();
  }

  void _toggleOpen(String id) {
    setState(() {
      if (_openLayerIds.contains(id)) {
        _openLayerIds.remove(id);
      } else {
        _openLayerIds.add(id);
      }
    });
  }

  Widget _groupRowBorder({required Widget child}) {
    return DecoratedBox(
      decoration: const BoxDecoration(
        border: Border(
          bottom: BorderSide(color: _dividerColor, width: 0.5),
        ),
      ),
      child: child,
    );
  }

  Widget _buildLayerHeader(
    BuildContext context, {
    required String id,
    required String title,
    required String layerKey,
  }) {
    return Consumer<GlobalState>(
      builder: (_, gs, __) {
        final open = _openLayerIds.contains(id);
        return _groupRowBorder(
          child: Material(
            color: Colors.transparent,
            child: InkWell(
              onTap: () => _toggleOpen(id),
              child: Padding(
                padding:
                    const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
                child: Row(
                  children: [
                    Icon(
                      open ? Icons.expand_less : Icons.chevron_right,
                      size: 20,
                      color: Colors.grey,
                    ),
                    const SizedBox(width: 4),
                    Expanded(
                      child: Text(
                        title,
                        style: const TextStyle(
                          fontSize: 16,
                          color: Colors.black,
                        ),
                      ),
                    ),
                    Switch.adaptive(
                      value: gs.isLayerVisible(layerKey),
                      onChanged: (v) => gs.setLayerState(layerKey, v),
                    ),
                  ],
                ),
              ),
            ),
          ),
        );
      },
    );
  }

  Widget _buildTopicField(
    BuildContext context, {
    required String label,
    required TextEditingController ctrl,
    required String backendKey,
  }) {
    return _groupRowBorder(
      child: Padding(
        padding: const EdgeInsets.fromLTRB(16, 8, 8, 8),
        child: Row(
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            Expanded(
              child: TextField(
                controller: ctrl,
                style: const TextStyle(fontSize: 16, color: Colors.black87),
                decoration: InputDecoration(
                  labelText: label,
                  isDense: true,
                  border: InputBorder.none,
                  labelStyle: const TextStyle(color: Colors.grey, fontSize: 13),
                  floatingLabelBehavior: FloatingLabelBehavior.auto,
                ),
                onSubmitted: (t) =>
                    saveGuiTopicToBackend(context, backendKey, t),
              ),
            ),
            IconButton(
              tooltip: AppLocalizations.of(context)!.save,
              icon: const Icon(Icons.check_circle_outline, size: 22),
              color: Theme.of(context).colorScheme.primary,
              onPressed: () =>
                  saveGuiTopicToBackend(context, backendKey, ctrl.text),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildColorRow(
    BuildContext context,
    GlobalState gs,
    String label,
    String layerId,
    Color fallback,
  ) {
    final c = gs.layerColorFor(layerId, fallback);
    return _groupRowBorder(
      child: Material(
        color: Colors.transparent,
        child: InkWell(
          onTap: () async {
            final p = await pickPresetColor(context, c);
            if (p != null) {
              gs.patchLayer(layerId, colorArgb: p.toARGB32().toString());
              await gs.saveLayerSettings();
              setState(() {});
            }
          },
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
            child: Row(
              children: [
                Expanded(
                  child: Text(
                    label,
                    style: const TextStyle(fontSize: 16, color: Colors.black),
                  ),
                ),
                Container(
                  width: 44,
                  height: 28,
                  decoration: BoxDecoration(
                    color: c,
                    borderRadius: BorderRadius.circular(6),
                    border: Border.all(color: Colors.grey.withValues(alpha: 0.35)),
                  ),
                ),
                const SizedBox(width: 4),
                const Icon(Icons.chevron_right, color: Colors.grey, size: 20),
              ],
            ),
          ),
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final l10n = AppLocalizations.of(context)!;
    final gs = Provider.of<GlobalState>(context);

    return Column(
      mainAxisSize: MainAxisSize.min,
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        _buildLayerHeader(context,
            id: 'grid', title: l10n.layer_grid, layerKey: 'grid'),
        _buildLayerHeader(context,
            id: 'gcost',
            title: l10n.layer_global_costmap,
            layerKey: 'globalCostmap'),
        if (_openLayerIds.contains('gcost')) ...[
          _buildTopicField(
            context,
            label: l10n.global_costmap_topic,
            ctrl: _globalCostTopic,
            backendKey: 'globalCostmapTopic',
          ),
        ],
        _buildLayerHeader(context,
            id: 'lcost',
            title: l10n.layer_local_costmap,
            layerKey: 'localCostmap'),
        if (_openLayerIds.contains('lcost')) ...[
          _buildTopicField(
            context,
            label: l10n.local_cost_map_topic,
            ctrl: _localCostTopic,
            backendKey: 'localCostmapTopic',
          ),
        ],
        _buildLayerHeader(context,
            id: 'laser', title: l10n.layer_laser, layerKey: 'laser'),
        if (_openLayerIds.contains('laser')) ...[
          _buildColorRow(context, gs, l10n.layer_color, 'laser', Colors.red),
          _groupRowBorder(
            child: Padding(
              padding: const EdgeInsets.fromLTRB(16, 4, 16, 8),
              child: Row(
                children: [
                  SizedBox(
                    width: 72,
                    child: Text(
                      l10n.layer_dot_size,
                      style: Theme.of(context).textTheme.bodySmall,
                    ),
                  ),
                  Expanded(
                    child: Slider.adaptive(
                      value: _laserDotDraft,
                      min: 0.5,
                      max: 8,
                      divisions: 15,
                      label: _laserDotDraft.toStringAsFixed(1),
                      onChanged: (v) {
                        setState(() => _laserDotDraft = v);
                      },
                      onChangeEnd: (_) {
                        gs.patchLayer(
                            'laser', dotRadius: _laserDotDraft.toString());
                        gs.saveLayerSettings();
                      },
                    ),
                  ),
                ],
              ),
            ),
          ),
          _buildTopicField(
            context,
            label: l10n.laser_topic,
            ctrl: _laserTopic,
            backendKey: 'laserTopic',
          ),
        ],
        _buildLayerHeader(context,
            id: 'pc',
            title: l10n.layer_pointcloud,
            layerKey: 'pointCloud'),
        if (_openLayerIds.contains('pc')) ...[
          _buildTopicField(
            context,
            label: l10n.pointcloud2_topic,
            ctrl: _pointCloudTopic,
            backendKey: 'pointCloud2Topic',
          ),
        ],
        _buildLayerHeader(context,
            id: 'gpath',
            title: l10n.layer_global_path,
            layerKey: 'globalPath'),
        if (_openLayerIds.contains('gpath')) ...[
          _buildColorRow(context, gs, l10n.layer_color, 'globalPath',
              const Color(0xFF2196F3)),
          _buildTopicField(
            context,
            label: l10n.global_path_topic,
            ctrl: _globalPathTopic,
            backendKey: 'globalPathTopic',
          ),
        ],
        _buildLayerHeader(context,
            id: 'lpath',
            title: l10n.layer_local_path,
            layerKey: 'localPath'),
        if (_openLayerIds.contains('lpath')) ...[
          _buildColorRow(
              context, gs, l10n.layer_color, 'localPath', Colors.green),
          _buildTopicField(
            context,
            label: l10n.local_path_topic,
            ctrl: _localPathTopic,
            backendKey: 'localPathTopic',
          ),
        ],
        _buildLayerHeader(context,
            id: 'tpath',
            title: l10n.layer_trace,
            layerKey: 'tracePath'),
        if (_openLayerIds.contains('tpath')) ...[
          _buildColorRow(
              context, gs, l10n.layer_color, 'tracePath', Colors.yellow),
          _buildTopicField(
            context,
            label: l10n.trace_path_topic,
            ctrl: _tracePathTopic,
            backendKey: 'tracePathTopic',
          ),
        ],
        _buildLayerHeader(context,
            id: 'topo',
            title: l10n.layer_topology,
            layerKey: 'topology'),
        _buildLayerHeader(context,
            id: 'foot',
            title: l10n.layer_robot_footprint,
            layerKey: 'robotFootprint'),
        if (_openLayerIds.contains('foot')) ...[
          _buildTopicField(
            context,
            label: l10n.robot_footprint_topic,
            ctrl: _footprintTopic,
            backendKey: 'robotFootprintTopic',
          ),
        ],
      ],
    );
  }
}
