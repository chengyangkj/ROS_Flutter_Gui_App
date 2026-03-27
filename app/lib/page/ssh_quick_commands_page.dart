import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/ssh_quick_cmd.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';
import 'package:ros_flutter_gui_app/provider/http_channel.dart';
import 'package:ros_flutter_gui_app/ssh/ssh_remote.dart';

class SshQuickCommandsPage extends StatefulWidget {
  const SshQuickCommandsPage({super.key});

  @override
  State<SshQuickCommandsPage> createState() => _SshQuickCommandsPageState();
}

class _SshQuickCommandsPageState extends State<SshQuickCommandsPage> {
  late List<SshQuickCmd> _items;
  bool _running = false;

  @override
  void initState() {
    super.initState();
    _items = List.from(
      globalSetting.sshQuickCommands.isEmpty
          ? defaultSshQuickCommands()
          : globalSetting.sshQuickCommands,
    );
  }

  Future<void> _persist() async {
    final body = globalSetting.buildBackendGuiSettingsJson();
    body['sshQuickCommands'] = _items.map((e) => e.toJson()).toList();
    await HttpChannel().saveGuiSettings(body);
    globalSetting.applyBackendGuiSettings(body);
  }

  Future<void> _runCmd(SshQuickCmd c) async {
    final l10n = AppLocalizations.of(context)!;
    if (!sshRemotePlatformSupported) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text(l10n.ssh_quick_platform_unsupported)),
        );
      }
      return;
    }
    if (c.useSudo && globalSetting.sshPassword.isEmpty) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text(l10n.ssh_quick_sudo_need_password)),
        );
      }
      return;
    }
    setState(() => _running = true);
    Object? client;
    try {
      client = await sshConnect(
        username: globalSetting.sshUsername.trim(),
        password: globalSetting.sshPassword,
      );
      final line = sshQuickCmdRemoteLine(c, globalSetting.sshPassword);
      final out = await sshRunRemoteCommand(client, line);
      sshClientClose(client);
      client = null;
      if (mounted) {
        await showDialog<void>(
          context: context,
          builder: (ctx) => AlertDialog(
            title: Text(c.name),
            content: SingleChildScrollView(
              child: SelectableText(
                  out.isEmpty ? l10n.ssh_quick_no_output : out),
            ),
            actions: [
              TextButton(
                onPressed: () => Navigator.pop(ctx),
                child: Text(l10n.ssh_quick_close),
              ),
            ],
          ),
        );
      }
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('$e')),
        );
      }
    } finally {
      if (client != null) {
        sshClientClose(client);
      }
      if (mounted) setState(() => _running = false);
    }
  }

  void _addCmd() {
    final l10n = AppLocalizations.of(context)!;
    final nameCtrl = TextEditingController();
    final cmdCtrl = TextEditingController();
    var useSudo = false;
    showDialog<void>(
      context: context,
      builder: (ctx) => StatefulBuilder(
        builder: (ctx, setDlg) => AlertDialog(
          title: Text(l10n.ssh_quick_add_title),
          content: SingleChildScrollView(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                TextField(
                  controller: nameCtrl,
                  decoration: InputDecoration(
                    labelText: l10n.ssh_quick_label_name,
                    border: const OutlineInputBorder(),
                  ),
                ),
                const SizedBox(height: 8),
                TextField(
                  controller: cmdCtrl,
                  decoration: InputDecoration(
                    labelText: l10n.ssh_quick_label_cmd,
                    border: const OutlineInputBorder(),
                  ),
                ),
                const SizedBox(height: 8),
                Builder(
                  builder: (childCtx) {
                    final cs = Theme.of(childCtx).colorScheme;
                    return CheckboxListTile(
                      value: useSudo,
                      onChanged: (v) => setDlg(() => useSudo = v ?? false),
                      title: Text(l10n.ssh_quick_use_sudo),
                      controlAffinity: ListTileControlAffinity.leading,
                      contentPadding: EdgeInsets.zero,
                      side: BorderSide(
                        color: cs.outline,
                        width: 1.5,
                      ),
                      fillColor: WidgetStateProperty.resolveWith((states) {
                        if (states.contains(WidgetState.selected)) {
                          return cs.primary;
                        }
                        return cs.surfaceContainerHighest;
                      }),
                      checkColor: cs.onPrimary,
                    );
                  },
                ),
              ],
            ),
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.pop(ctx),
              child: Text(l10n.ssh_quick_cancel),
            ),
            FilledButton(
              onPressed: () {
                final n = nameCtrl.text.trim();
                final cmd = cmdCtrl.text.trim();
                if (n.isEmpty || cmd.isEmpty) return;
                setState(() => _items.add(SshQuickCmd(
                      name: n,
                      cmd: cmd,
                      useSudo: useSudo,
                    )));
                Navigator.pop(ctx);
                _persist();
              },
              child: Text(l10n.ssh_quick_add_btn),
            ),
          ],
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final l10n = AppLocalizations.of(context)!;
    return Scaffold(
      appBar: AppBar(
        title: Text(l10n.ssh_quick_page_title),
        actions: [
          IconButton(
            icon: const Icon(Icons.save),
            onPressed: _running ? null : _persist,
            tooltip: l10n.ssh_quick_save_list_tooltip,
          ),
        ],
      ),
      body: Stack(
        children: [
          ListView.builder(
            itemCount: _items.length,
            itemBuilder: (_, i) {
              final c = _items[i];
              return ListTile(
                title: Text(c.name),
                subtitle: Text(
                  c.cmd,
                  maxLines: 2,
                  overflow: TextOverflow.ellipsis,
                ),
                trailing: SizedBox(
                  width: 168,
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.end,
                    children: [
                      Tooltip(
                        message: l10n.ssh_quick_use_sudo,
                        child: Builder(
                          builder: (switchCtx) {
                            final scheme = Theme.of(switchCtx).colorScheme;
                            return Theme(
                              data: Theme.of(switchCtx).copyWith(
                                switchTheme: SwitchThemeData(
                                  thumbColor: WidgetStateProperty.resolveWith(
                                      (states) {
                                    if (states.contains(WidgetState.disabled)) {
                                      return scheme.onSurface
                                          .withValues(alpha: 0.38);
                                    }
                                    if (states
                                        .contains(WidgetState.selected)) {
                                      return scheme.onPrimary;
                                    }
                                    return const Color(0xFF616161);
                                  }),
                                  trackColor: WidgetStateProperty.resolveWith(
                                      (states) {
                                    if (states.contains(WidgetState.disabled)) {
                                      return const Color(0xFFE0E0E0)
                                          .withValues(alpha: 0.6);
                                    }
                                    if (states
                                        .contains(WidgetState.selected)) {
                                      return scheme.primary;
                                    }
                                    return const Color(0xFFE0E0E0);
                                  }),
                                ),
                              ),
                              child: Switch.adaptive(
                                value: c.useSudo,
                                onChanged: _running
                                    ? null
                                    : (v) {
                                        setState(() => _items[i] =
                                            c.copyWith(useSudo: v));
                                        _persist();
                                      },
                              ),
                            );
                          },
                        ),
                      ),
                      IconButton(
                        icon: const Icon(Icons.play_arrow),
                        onPressed: _running ? null : () => _runCmd(c),
                      ),
                      IconButton(
                        icon: const Icon(Icons.delete_outline),
                        onPressed: () {
                          setState(() => _items.removeAt(i));
                          _persist();
                        },
                      ),
                    ],
                  ),
                ),
              );
            },
          ),
          if (_running)
            const Center(child: CircularProgressIndicator()),
        ],
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: _addCmd,
        child: const Icon(Icons.add),
      ),
    );
  }
}
