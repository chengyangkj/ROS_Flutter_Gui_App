import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/provider/ws_channel.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:toastification/toastification.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';

const bool kSkipRosConnectForDebug = false;

class ConnectPage extends StatefulWidget {
  @override
  _ConnectPageState createState() => _ConnectPageState();
}

class _ConnectPageState extends State<ConnectPage>
    with SingleTickerProviderStateMixin {
  final TextEditingController _ipController =
      TextEditingController(text: '127.0.0.1');
  final TextEditingController _portController =
      TextEditingController(text: '7684');
  bool _isConnecting = false;
  late AnimationController _animationController;
  late CurvedAnimation _fadeAnimation;

  @override
  void initState() {
    super.initState();
    _animationController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 600),
    );
    _fadeAnimation = CurvedAnimation(
      parent: _animationController,
      curve: Curves.easeOutCubic,
    );
    _animationController.forward();
  }

  @override
  void dispose() {
    _fadeAnimation.dispose();
    _animationController.dispose();
    _ipController.dispose();
    _portController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: FutureBuilder<bool>(
        future: initGlobalSetting(),
        builder: (BuildContext context, AsyncSnapshot<bool> snapshot) {
          if (snapshot.connectionState == ConnectionState.waiting) {
            return const Center(child: CircularProgressIndicator());
          } else if (snapshot.hasError) {
            return Center(child: Text(AppLocalizations.of(context)!.init_error(snapshot.error.toString())));
          }

          _ipController.text = globalSetting.robotIp;
          _portController.text = globalSetting.httpServerPort;

          final scheme = Theme.of(context).colorScheme;
          final primaryColor = scheme.primary;

          return Container(
            decoration: BoxDecoration(
              gradient: LinearGradient(
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
                colors: [
                  primaryColor.withOpacity(0.18),
                  primaryColor.withOpacity(0.10),
                  primaryColor.withOpacity(0.15),
                ],
                stops: const [0.0, 0.5, 1.0],
              ),
            ),
            child: SafeArea(
              child: Stack(
                children: [
                  Center(
                    child: FadeTransition(
                      opacity: _fadeAnimation,
                      child: Padding(
                        padding: const EdgeInsets.symmetric(horizontal: 28),
                        child: ConstrainedBox(
                          constraints: const BoxConstraints(maxWidth: 440),
                          child: Column(
                            mainAxisSize: MainAxisSize.min,
                            children: [
                              const SizedBox(height: 24),
                              Icon(
                                Icons.hub_rounded,
                                size: 32,
                                color: primaryColor.withOpacity(0.9),
                              ),
                              const SizedBox(height: 12),
                              Text(
                                AppLocalizations.of(context)!.connect_robot,
                                style: Theme.of(context).textTheme.titleMedium?.copyWith(
                                  fontWeight: FontWeight.w600,
                                  letterSpacing: -0.2,
                                ),
                              ),
                              const SizedBox(height: 4),
                              Text(
                                AppLocalizations.of(context)!.connect_robot_subtitle,
                                style: Theme.of(context).textTheme.bodySmall?.copyWith(
                                  color: scheme.onSurfaceVariant.withOpacity(0.8),
                                ),
                                textAlign: TextAlign.center,
                              ),
                              const SizedBox(height: 24),
                              Row(
                                crossAxisAlignment: CrossAxisAlignment.start,
                                children: [
                                  Expanded(
                                    flex: 2,
                                    child: TextField(
                                      controller: _ipController,
                                      style: const TextStyle(fontSize: 14),
                                      decoration: InputDecoration(
                                        labelText: AppLocalizations.of(context)!.ip_address,
                                        isDense: true,
                                        prefixIcon: Icon(Icons.lan_rounded, size: 18, color: primaryColor),
                                        filled: true,
                                        fillColor: scheme.surfaceContainerHighest.withOpacity(0.5),
                                        border: OutlineInputBorder(
                                          borderRadius: BorderRadius.circular(10),
                                          borderSide: BorderSide.none,
                                        ),
                                        contentPadding: const EdgeInsets.symmetric(
                                          horizontal: 12,
                                          vertical: 12,
                                        ),
                                      ),
                                    ),
                                  ),
                                  const SizedBox(width: 12),
                                  Expanded(
                                    flex: 1,
                                    child: TextField(
                                      controller: _portController,
                                      style: const TextStyle(fontSize: 14),
                                      keyboardType: TextInputType.number,
                                      decoration: InputDecoration(
                                        labelText: AppLocalizations.of(context)!.port,
                                        isDense: true,
                                        filled: true,
                                        fillColor: scheme.surfaceContainerHighest.withOpacity(0.5),
                                        border: OutlineInputBorder(
                                          borderRadius: BorderRadius.circular(10),
                                          borderSide: BorderSide.none,
                                        ),
                                        contentPadding: const EdgeInsets.symmetric(
                                          horizontal: 12,
                                          vertical: 12,
                                        ),
                                      ),
                                    ),
                                  ),
                                ],
                              ),
                              const SizedBox(height: 20),
                              SizedBox(
                                width: double.infinity,
                                height: 44,
                                child: FilledButton(
                                  onPressed: _isConnecting ? null : _handleConnect,
                                  style: FilledButton.styleFrom(
                                    shape: RoundedRectangleBorder(
                                      borderRadius: BorderRadius.circular(12),
                                    ),
                                  ),
                                  child: _isConnecting
                                      ? SizedBox(
                                          height: 22,
                                          width: 22,
                                          child: CircularProgressIndicator(
                                            strokeWidth: 2.5,
                                            color: scheme.onPrimary,
                                          ),
                                        )
                                      : Text(
                                          AppLocalizations.of(context)!.connect_robot,
                                          style: const TextStyle(
                                            fontSize: 15,
                                            fontWeight: FontWeight.w600,
                                          ),
                                        ),
                                ),
                              ),
                            ],
                          ),
                        ),
                      ),
                    ),
                  ),
                  Positioned(
                    top: 12,
                    right: 12,
                    child: FadeTransition(
                      opacity: _fadeAnimation,
                      child: IconButton(
                        onPressed: () => Navigator.pushNamed(context, "/setting"),
                        icon: const Icon(Icons.settings_rounded, size: 24),
                        style: IconButton.styleFrom(
                          backgroundColor: scheme.surfaceContainerHighest.withOpacity(0.6),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          );
        },
      ),
    );
  }

  Future<void> _handleConnect() async {
    setState(() => _isConnecting = true);
    try {
      final String ip = _ipController.text.trim();
      final int port = int.tryParse(_portController.text) ?? 7684;

      final provider = Provider.of<WsChannel>(context, listen: false);
      globalSetting.setRobotIp(ip);
      globalSetting.setHttpServerPort(_portController.text);
      globalSetting.setRobotPort(_portController.text);

      if (!kSkipRosConnectForDebug) {
        final err = await provider.connectBackend(ip, port);
        if (err.isNotEmpty) {
          if (!context.mounted) {
            return;
          }
          toastification.show(
            context: context,
            title: Text("后台连接失败: $err"),
            autoCloseDuration: const Duration(seconds: 5),
          );
          return;
        }
      }

      if (!context.mounted) {
        return;
      }
      Navigator.pushNamed(context, "/map");
    } finally {
      if (mounted) {
        setState(() => _isConnecting = false);
      }
    }
  }
}
