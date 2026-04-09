import 'dart:ui';

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
      TextEditingController(text: '8080');
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
            return Center(
              child: Text(
                AppLocalizations.of(context)!.init_error(snapshot.error.toString()),
              ),
            );
          }

          _ipController.text = globalSetting.robotIp;
          _portController.text = globalSetting.httpServerPort;

          final scheme = Theme.of(context).colorScheme;
          final primaryColor = scheme.primary;
          final surface = scheme.surface;
          final onSurface = scheme.onSurface;
          final viewPadding = MediaQuery.of(context).viewPadding;

          final titleStyle = Theme.of(context).textTheme.titleMedium?.copyWith(
                fontWeight: FontWeight.w700,
                letterSpacing: -0.3,
                color: onSurface,
              );
          final subTitleStyle = Theme.of(context).textTheme.bodySmall?.copyWith(
                color: scheme.onSurfaceVariant.withOpacity(0.82),
                height: 1.25,
              );

          return Container(
            color: surface,
            child: Stack(
              children: [
                Positioned.fill(
                  child: DecoratedBox(
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                        begin: Alignment.topLeft,
                        end: Alignment.bottomRight,
                        colors: [
                          Color.lerp(surface, primaryColor, 0.10)!,
                          Color.lerp(surface, primaryColor, 0.06)!,
                          Color.lerp(surface, scheme.secondary, 0.05)!,
                          Color.lerp(surface, scheme.tertiary, 0.04)!,
                        ],
                        stops: const [0.0, 0.35, 0.7, 1.0],
                      ),
                    ),
                  ),
                ),
                Positioned.fill(
                  child: CustomPaint(
                    painter: TechBackgroundPainter(
                      seedColor: primaryColor,
                      surfaceColor: surface,
                      brightness: Theme.of(context).brightness,
                    ),
                  ),
                ),
                Positioned(
                  left: -80,
                  top: -70,
                  child: _SoftGlowBlob(
                    diameter: 240,
                    color: scheme.tertiary.withOpacity(0.22),
                  ),
                ),
                Positioned(
                  right: -60,
                  top: 110,
                  child: _SoftGlowBlob(
                    diameter: 200,
                    color: scheme.secondary.withOpacity(0.18),
                  ),
                ),
                Positioned(
                  right: -90,
                  bottom: -80,
                  child: _SoftGlowBlob(
                    diameter: 260,
                    color: primaryColor.withOpacity(0.20),
                  ),
                ),
                Center(
                  child: FadeTransition(
                    opacity: _fadeAnimation,
                    child: Padding(
                      padding: const EdgeInsets.symmetric(horizontal: 22),
                      child: ConstrainedBox(
                        constraints: const BoxConstraints(maxWidth: 460),
                        child: _GlassPanel(
                          borderRadius: 22,
                          child: Padding(
                            padding: const EdgeInsets.fromLTRB(18, 18, 18, 16),
                            child: Column(
                              mainAxisSize: MainAxisSize.min,
                              children: [
                                  Row(
                                    children: [
                                      _TechMark(color: primaryColor),
                                      const SizedBox(width: 10),
                                      Expanded(
                                        child: Column(
                                          crossAxisAlignment:
                                              CrossAxisAlignment.start,
                                          children: [
                                            Text(
                                              AppLocalizations.of(context)!
                                                  .connect_robot,
                                              style: titleStyle,
                                            ),
                                            const SizedBox(height: 2),
                                            Text(
                                              AppLocalizations.of(context)!
                                                  .connect_robot_subtitle,
                                              style: subTitleStyle,
                                            ),
                                          ],
                                        ),
                                      ),
                                    ],
                                  ),
                                  const SizedBox(height: 16),
                                  Row(
                                    crossAxisAlignment:
                                        CrossAxisAlignment.start,
                                    children: [
                                      Expanded(
                                        flex: 2,
                                        child: _FrostedTextField(
                                          controller: _ipController,
                                          labelText:
                                              AppLocalizations.of(context)!
                                                  .ip_address,
                                          prefixIcon: Icons.lan_rounded,
                                          tintColor: primaryColor,
                                        ),
                                      ),
                                      const SizedBox(width: 12),
                                      Expanded(
                                        flex: 1,
                                        child: _FrostedTextField(
                                          controller: _portController,
                                          labelText: AppLocalizations.of(
                                                  context)!
                                              .port,
                                          keyboardType:
                                              TextInputType.number,
                                          tintColor: primaryColor,
                                        ),
                                      ),
                                    ],
                                  ),
                                  const SizedBox(height: 14),
                                  SizedBox(
                                    width: double.infinity,
                                    height: 46,
                                    child: FilledButton(
                                      onPressed: _isConnecting
                                          ? null
                                          : _handleConnect,
                                      style: FilledButton.styleFrom(
                                        shape: RoundedRectangleBorder(
                                          borderRadius:
                                              BorderRadius.circular(14),
                                        ),
                                      ),
                                      child: _isConnecting
                                          ? SizedBox(
                                              height: 22,
                                              width: 22,
                                              child:
                                                  CircularProgressIndicator(
                                                strokeWidth: 2.5,
                                                color: scheme.onPrimary,
                                              ),
                                            )
                                          : Row(
                                              mainAxisAlignment:
                                                  MainAxisAlignment.center,
                                              children: [
                                                Icon(
                                                  Icons
                                                      .bolt_rounded,
                                                  size: 18,
                                                  color:
                                                      scheme.onPrimary,
                                                ),
                                                const SizedBox(width: 8),
                                                Text(
                                                  AppLocalizations.of(
                                                          context)!
                                                      .connect_robot,
                                                  style: const TextStyle(
                                                    fontSize: 15,
                                                    fontWeight:
                                                        FontWeight.w700,
                                                    letterSpacing: -0.2,
                                                  ),
                                                ),
                                              ],
                                            ),
                                    ),
                                  ),
                              ],
                            ),
                          ),
                        ),
                      ),
                    ),
                  ),
                ),
                Positioned(
                  top: 12 + viewPadding.top,
                  right: 12 + viewPadding.right,
                  child: FadeTransition(
                    opacity: _fadeAnimation,
                    child: _GlassIconButton(
                      onPressed: () => Navigator.pushNamed(context, "/setting"),
                      icon: Icons.settings_rounded,
                      tint: primaryColor,
                    ),
                  ),
                ),
              ],
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
      final int port = int.tryParse(_portController.text) ?? 8080;

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

class _GlassPanel extends StatelessWidget {
  final Widget child;
  final double borderRadius;

  const _GlassPanel({
    required this.child,
    required this.borderRadius,
  });

  @override
  Widget build(BuildContext context) {
    final scheme = Theme.of(context).colorScheme;
    final borderColor = scheme.outlineVariant.withOpacity(0.55);
    final highlight = Colors.white.withOpacity(0.50);

    return ClipRRect(
      borderRadius: BorderRadius.circular(borderRadius),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 10, sigmaY: 10),
        child: DecoratedBox(
          decoration: BoxDecoration(
            gradient: LinearGradient(
              begin: Alignment.topLeft,
              end: Alignment.bottomRight,
              colors: [
                scheme.surface.withOpacity(0.70),
                scheme.surface.withOpacity(0.58),
              ],
            ),
            borderRadius: BorderRadius.circular(borderRadius),
            border: Border.all(color: borderColor, width: 1),
            boxShadow: [
              BoxShadow(
                color: Colors.black.withOpacity(0.08),
                blurRadius: 22,
                offset: const Offset(0, 16),
              ),
            ],
          ),
          child: Stack(
            children: [
              Positioned(
                left: -60,
                top: -60,
                child: Container(
                  width: 160,
                  height: 160,
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    gradient: RadialGradient(
                      colors: [
                        highlight.withOpacity(0.60),
                        Colors.transparent,
                      ],
                    ),
                  ),
                ),
              ),
              child,
            ],
          ),
        ),
      ),
    );
  }
}

class _SoftGlowBlob extends StatelessWidget {
  final double diameter;
  final Color color;

  const _SoftGlowBlob({
    required this.diameter,
    required this.color,
  });

  @override
  Widget build(BuildContext context) {
    return IgnorePointer(
      child: Container(
        width: diameter,
        height: diameter,
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          gradient: RadialGradient(
            colors: [
              color,
              Colors.transparent,
            ],
          ),
        ),
      ),
    );
  }
}

class _GlassIconButton extends StatelessWidget {
  final VoidCallback onPressed;
  final IconData icon;
  final Color tint;

  const _GlassIconButton({
    required this.onPressed,
    required this.icon,
    required this.tint,
  });

  @override
  Widget build(BuildContext context) {
    final scheme = Theme.of(context).colorScheme;
    return ClipRRect(
      borderRadius: BorderRadius.circular(14),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 9, sigmaY: 9),
        child: DecoratedBox(
          decoration: BoxDecoration(
            color: scheme.surface.withOpacity(0.55),
            border: Border.all(
              color: scheme.outlineVariant.withOpacity(0.45),
            ),
            borderRadius: BorderRadius.circular(14),
          ),
          child: IconButton(
            onPressed: onPressed,
            icon: Icon(icon, size: 22, color: tint.withOpacity(0.95)),
            splashRadius: 22,
            tooltip: MaterialLocalizations.of(context).openAppDrawerTooltip,
          ),
        ),
      ),
    );
  }
}

class _TechMark extends StatelessWidget {
  final Color color;

  const _TechMark({required this.color});

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: 38,
      height: 38,
      child: DecoratedBox(
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(12),
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
            colors: [
              color.withOpacity(0.90),
              color.withOpacity(0.55),
            ],
          ),
          boxShadow: [
            BoxShadow(
              color: color.withOpacity(0.18),
              blurRadius: 18,
              offset: const Offset(0, 10),
            ),
          ],
        ),
        child: Stack(
          children: [
            Positioned.fill(
              child: Opacity(
                opacity: 0.22,
                child: CustomPaint(
                  painter: _MicroCircuitPainter(),
                ),
              ),
            ),
            const Center(
              child: Icon(Icons.hub_rounded, size: 18, color: Colors.white),
            ),
          ],
        ),
      ),
    );
  }
}

class _FrostedTextField extends StatelessWidget {
  final TextEditingController controller;
  final String labelText;
  final TextInputType? keyboardType;
  final IconData? prefixIcon;
  final Color tintColor;

  const _FrostedTextField({
    required this.controller,
    required this.labelText,
    required this.tintColor,
    this.keyboardType,
    this.prefixIcon,
  });

  @override
  Widget build(BuildContext context) {
    final scheme = Theme.of(context).colorScheme;
    final fill = scheme.surfaceContainerHighest.withOpacity(0.35);
    final border = scheme.outlineVariant.withOpacity(0.55);

    return TextField(
      controller: controller,
      keyboardType: keyboardType,
      style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600),
      decoration: InputDecoration(
        labelText: labelText,
        isDense: true,
        prefixIcon: prefixIcon == null
            ? null
            : Icon(prefixIcon, size: 18, color: tintColor.withOpacity(0.95)),
        filled: true,
        fillColor: fill,
        border: OutlineInputBorder(
          borderRadius: BorderRadius.circular(14),
          borderSide: BorderSide(color: border),
        ),
        enabledBorder: OutlineInputBorder(
          borderRadius: BorderRadius.circular(14),
          borderSide: BorderSide(color: border),
        ),
        focusedBorder: OutlineInputBorder(
          borderRadius: BorderRadius.circular(14),
          borderSide: BorderSide(color: tintColor.withOpacity(0.75), width: 1.2),
        ),
        contentPadding: const EdgeInsets.symmetric(
          horizontal: 12,
          vertical: 12,
        ),
      ),
    );
  }
}

class TechBackgroundPainter extends CustomPainter {
  final Color seedColor;
  final Color surfaceColor;
  final Brightness brightness;

  TechBackgroundPainter({
    required this.seedColor,
    required this.surfaceColor,
    required this.brightness,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final base = seedColor;
    final isDark = brightness == Brightness.dark;

    final gridPaint = Paint()
      ..color = (isDark ? Colors.white : base)
          .withOpacity(isDark ? 0.06 : 0.07)
      ..strokeWidth = 1;

    const step = 22.0;
    for (double x = 0; x <= size.width; x += step) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), gridPaint);
    }
    for (double y = 0; y <= size.height; y += step) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), gridPaint);
    }

    final pathPaint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.6
      ..strokeCap = StrokeCap.round
      ..color = base.withOpacity(isDark ? 0.18 : 0.16);

    final glowPaint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 6
      ..strokeCap = StrokeCap.round
      ..color = base.withOpacity(isDark ? 0.10 : 0.09)
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 6);

    final p = Path()
      ..moveTo(size.width * 0.08, size.height * 0.22)
      ..quadraticBezierTo(
        size.width * 0.26,
        size.height * 0.12,
        size.width * 0.40,
        size.height * 0.20,
      )
      ..quadraticBezierTo(
        size.width * 0.62,
        size.height * 0.30,
        size.width * 0.86,
        size.height * 0.16,
      )
      ..quadraticBezierTo(
        size.width * 0.92,
        size.height * 0.52,
        size.width * 0.72,
        size.height * 0.64,
      )
      ..quadraticBezierTo(
        size.width * 0.50,
        size.height * 0.78,
        size.width * 0.18,
        size.height * 0.70,
      );

    canvas.drawPath(p, glowPaint);
    canvas.drawPath(p, pathPaint);

    final nodePaint = Paint()..color = base.withOpacity(isDark ? 0.28 : 0.26);
    final nodeGlow = Paint()
      ..color = base.withOpacity(isDark ? 0.18 : 0.16)
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 8);

    final nodes = <Offset>[
      Offset(size.width * 0.08, size.height * 0.22),
      Offset(size.width * 0.40, size.height * 0.20),
      Offset(size.width * 0.86, size.height * 0.16),
      Offset(size.width * 0.72, size.height * 0.64),
      Offset(size.width * 0.18, size.height * 0.70),
    ];

    for (final o in nodes) {
      canvas.drawCircle(o, 7, nodeGlow);
      canvas.drawCircle(o, 2.2, nodePaint);
    }
  }

  @override
  bool shouldRepaint(covariant TechBackgroundPainter oldDelegate) {
    return oldDelegate.seedColor != seedColor ||
        oldDelegate.surfaceColor != surfaceColor ||
        oldDelegate.brightness != brightness;
  }
}

class _MicroCircuitPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.white
      ..strokeWidth = 1.2
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round;

    final p = Path()
      ..moveTo(size.width * 0.18, size.height * 0.62)
      ..lineTo(size.width * 0.45, size.height * 0.62)
      ..quadraticBezierTo(
        size.width * 0.62,
        size.height * 0.62,
        size.width * 0.70,
        size.height * 0.46,
      )
      ..lineTo(size.width * 0.82, size.height * 0.22);

    canvas.drawPath(p, paint);
    canvas.drawCircle(Offset(size.width * 0.18, size.height * 0.62), 2.0, paint);
    canvas.drawCircle(Offset(size.width * 0.45, size.height * 0.62), 2.0, paint);
    canvas.drawCircle(Offset(size.width * 0.82, size.height * 0.22), 2.0, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}
