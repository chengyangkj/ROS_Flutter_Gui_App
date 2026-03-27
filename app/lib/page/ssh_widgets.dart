import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';
import 'package:ros_flutter_gui_app/provider/http_channel.dart';

Future<void> ShowSshConfigSheet(BuildContext parentContext) async {
  final l10n = AppLocalizations.of(parentContext)!;
  final portCtrl = TextEditingController(text: '${globalSetting.sshPort}');
  final userCtrl = TextEditingController(text: globalSetting.sshUsername);
  final passCtrl = TextEditingController(text: globalSetting.sshPassword);
  final robotIp = globalSetting.robotIp.trim();

  final ok = await showModalBottomSheet<bool>(
    context: parentContext,
    isScrollControlled: true,
    builder: (ctx) => Padding(
      padding: EdgeInsets.only(
        left: 16,
        right: 16,
        top: 16,
        bottom: MediaQuery.viewInsetsOf(ctx).bottom + 16,
      ),
      child: SingleChildScrollView(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Text(l10n.ssh_config_title,
                style: const TextStyle(fontSize: 18, fontWeight: FontWeight.w600)),
            const SizedBox(height: 12),
            Text(
              l10n.ssh_target_same_as_robot(robotIp.isEmpty ? '—' : robotIp),
              style: TextStyle(
                fontSize: 14,
                color: Theme.of(ctx).colorScheme.onSurfaceVariant,
              ),
            ),
            const SizedBox(height: 12),
            TextField(
              controller: portCtrl,
              keyboardType: TextInputType.number,
              decoration: InputDecoration(
                labelText: l10n.port,
                border: const OutlineInputBorder(),
              ),
            ),
            const SizedBox(height: 8),
            TextField(
              controller: userCtrl,
              decoration: InputDecoration(
                labelText: l10n.ssh_username,
                border: const OutlineInputBorder(),
              ),
            ),
            const SizedBox(height: 8),
            TextField(
              controller: passCtrl,
              obscureText: true,
              decoration: InputDecoration(
                labelText: l10n.ssh_password,
                border: const OutlineInputBorder(),
              ),
            ),
            const SizedBox(height: 16),
            FilledButton(
              onPressed: () async {
                final port = int.tryParse(portCtrl.text.trim()) ?? 22;
                final body = globalSetting.buildBackendGuiSettingsJson();
                body['sshHost'] = robotIp;
                body['sshPort'] = port.clamp(1, 65535);
                body['sshUsername'] = userCtrl.text.trim();
                body['sshPassword'] = passCtrl.text;
                body['sshQuickCommands'] =
                    globalSetting.sshQuickCommands.map((e) => e.toJson()).toList();
                try {
                  await HttpChannel().saveGuiSettings(body);
                  globalSetting.applyBackendGuiSettings(body);
                  if (ctx.mounted) Navigator.pop(ctx, true);
                } catch (e) {
                  if (ctx.mounted) {
                    ScaffoldMessenger.of(ctx).showSnackBar(
                      SnackBar(content: Text('$e')),
                    );
                  }
                }
              },
              child: Text(l10n.ssh_save_to_backend),
            ),
            const SizedBox(height: 8),
          ],
        ),
      ),
    ),
  );
  portCtrl.dispose();
  userCtrl.dispose();
  passCtrl.dispose();
  if (ok == true && parentContext.mounted) {
    ScaffoldMessenger.of(parentContext).showSnackBar(
      SnackBar(content: Text(AppLocalizations.of(parentContext)!.ssh_config_saved)),
    );
  }
}
