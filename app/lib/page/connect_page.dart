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
    with TickerProviderStateMixin {
  final TextEditingController _ipController =
      TextEditingController(text: '127.0.0.1');
  final TextEditingController _portController =
      TextEditingController(text: '8080');
  bool _isConnecting = false;
  late AnimationController _animationController;
  late CurvedAnimation _fadeAnimation;
  late AnimationController _connectShineController;  

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
    _connectShineController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 2600),
    )..repeat();
  }

  @override
  void dispose() {
    _animationController.dispose();
    _connectShineController.dispose();
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
          final screenWidth = MediaQuery.sizeOf(context).width;
          final cardMaxWidth = screenWidth < 600
              ? (screenWidth * 0.92).clamp(280.0, 460.0)
              : screenWidth < 1100
                  ? (screenWidth * 0.52).clamp(400.0, 560.0)
                  : (screenWidth * 0.44).clamp(520.0, 720.0);
          final cardHorizontalPadding =
              screenWidth < 600 ? 22.0 : screenWidth < 1100 ? 32.0 : 48.0;

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
                  child: _CyberBezierBackdrop(
                    seedColor: primaryColor,
                    surfaceColor: surface,
                    brightness: Theme.of(context).brightness,
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
                      padding: EdgeInsets.symmetric(
                        horizontal: cardHorizontalPadding,
                      ),
                      child: ConstrainedBox(
                        constraints: BoxConstraints(maxWidth: cardMaxWidth),
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
                                    child: ClipRRect(
                                      borderRadius: BorderRadius.circular(14),
                                      child: Stack(
                                        fit: StackFit.expand,
                                        children: [
                                          FilledButton(
                                            onPressed: _isConnecting
                                                ? null
                                                : _handleConnect,
                                            style: FilledButton.styleFrom(
                                              minimumSize: const Size(
                                                double.infinity,
                                                46,
                                              ),
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
                                                        MainAxisAlignment
                                                            .center,
                                                    children: [
                                                      Icon(
                                                        Icons.bolt_rounded,
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
                                          if (!_isConnecting)
                                            Positioned.fill(
                                              child: IgnorePointer(
                                                child: AnimatedBuilder(
                                                  animation:
                                                      _connectShineController,
                                                  builder:
                                                      (BuildContext context,
                                                          Widget? child) {
                                                    return CustomPaint(
                                                      painter:
                                                          _ConnectButtonShinePainter(
                                                        progress:
                                                            _connectShineController
                                                                .value,
                                                      ),
                                                    );
                                                  },
                                                ),
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
          child: child,
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

class _ConnectButtonShinePainter extends CustomPainter {
  _ConnectButtonShinePainter({required this.progress});

  final double progress;

  @override
  void paint(Canvas canvas, Size size) {
    final Rect rect = Offset.zero & size;
    final Paint paint = Paint()
      ..blendMode = BlendMode.softLight
      ..shader = LinearGradient(
        begin: Alignment(-1.35 + progress * 2.7, -1.1),
        end: Alignment(-0.25 + progress * 2.7, 1.1),
        colors: <Color>[
          Colors.transparent,
          Colors.white.withValues(alpha: 0.0),
          Colors.white.withValues(alpha: 0.38),
          Colors.white.withValues(alpha: 0.58),
          Colors.white.withValues(alpha: 0.38),
          Colors.white.withValues(alpha: 0.0),
          Colors.transparent,
        ],
        stops: const <double>[0.0, 0.38, 0.46, 0.5, 0.54, 0.62, 1.0],
      ).createShader(rect);
    canvas.drawRect(rect, paint);
  }

  @override
  bool shouldRepaint(covariant _ConnectButtonShinePainter oldDelegate) {
    return oldDelegate.progress != progress;
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

class _CyberBezierBackdrop extends StatefulWidget {
  final Color seedColor;
  final Color surfaceColor;
  final Brightness brightness;

  const _CyberBezierBackdrop({
    required this.seedColor,
    required this.surfaceColor,
    required this.brightness,
  });

  @override
  State<_CyberBezierBackdrop> createState() => _CyberBezierBackdropState();
}

class _CyberBezierBackdropState extends State<_CyberBezierBackdrop>
    with SingleTickerProviderStateMixin {
  late AnimationController _flowController;

  @override
  void initState() {
    super.initState();
    _flowController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 6400),
    )..repeat();
  }

  @override
  void dispose() {
    _flowController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      painter: _ZigzagFlowBackdropPainter(
        animation: _flowController,
        seedColor: widget.seedColor,
        surfaceColor: widget.surfaceColor,
        brightness: widget.brightness,
      ),
    );
  }
}

class _ZigzagFlowBackdropPainter extends CustomPainter {
  final Animation<double> animation;
  final Color seedColor;
  final Color surfaceColor;
  final Brightness brightness;

  _ZigzagFlowBackdropPainter({
    required this.animation,
    required this.seedColor,
    required this.surfaceColor,
    required this.brightness,
  }) : super(repaint: animation);

  static Path _flowBackdropPath(Size s) {
    final double w = s.width;
    final double h = s.height;
    return Path()
      ..moveTo(w * 0.08, h * 0.22)
      ..quadraticBezierTo(w * 0.26, h * 0.12, w * 0.40, h * 0.20)
      ..quadraticBezierTo(w * 0.62, h * 0.30, w * 0.86, h * 0.16)
      ..quadraticBezierTo(w * 0.92, h * 0.52, w * 0.72, h * 0.64)
      ..quadraticBezierTo(w * 0.50, h * 0.78, w * 0.18, h * 0.70);
  }

  void _paintGrid(Canvas canvas, Size size, Color base, bool isDark) {
    final gridPaint = Paint()
      ..color = (isDark ? Colors.white : base)
          .withOpacity(isDark ? 0.048 : 0.058)
      ..strokeWidth = 1;
    const double step = 36.0;
    for (double x = 0; x <= size.width; x += step) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), gridPaint);
    }
    for (double y = 0; y <= size.height; y += step) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), gridPaint);
    }
  }

  void _paintStaticRail(
    Canvas canvas,
    Path path,
    Color base,
    bool isDark,
  ) {
    final track = Paint()
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round
      ..strokeWidth = 2.2
      ..color = base.withOpacity(isDark ? 0.14 : 0.12);
    canvas.drawPath(path, track);
  }

  void _paintFlowPulse(
    Canvas canvas,
    Path path,
    double phase01,
    Color neon,
  ) {
    for (final PathMetric metric in path.computeMetrics()) {
      final double len = metric.length;
      if (len < 24) continue;
      final double head = (phase01 * len) % len;
      const double tailFrac = 0.12;
      final double tail = len * tailFrac;
      double tailStart = head - tail;

      final Paint glow = Paint()
        ..style = PaintingStyle.stroke
        ..strokeCap = StrokeCap.round
        ..strokeJoin = StrokeJoin.round
        ..strokeWidth = 6
        ..color = neon.withOpacity(0.22)
        ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 4);
      final Paint core = Paint()
        ..style = PaintingStyle.stroke
        ..strokeCap = StrokeCap.round
        ..strokeJoin = StrokeJoin.round
        ..strokeWidth = 2.2
        ..color = neon.withOpacity(0.92);
      final Paint headDot = Paint()
        ..color = Colors.white.withOpacity(0.85)
        ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 3);

      void drawRange(double a, double b) {
        if (b <= a) return;
        final Path sub = metric.extractPath(a, b);
        canvas.drawPath(sub, glow);
        canvas.drawPath(sub, core);
      }

      if (tailStart >= 0) {
        drawRange(tailStart, head);
      } else {
        drawRange(0, head);
        drawRange(len + tailStart, len);
      }

      final Tangent? tan = metric.getTangentForOffset(head);
      if (tan != null) {
        canvas.drawCircle(tan.position, 4.5, headDot);
        canvas.drawCircle(tan.position, 2.4, Paint()..color = neon);
      }
    }
  }

  @override
  void paint(Canvas canvas, Size size) {
    final double flowT = animation.value;
    final Color base = seedColor;
    final bool isDark = brightness == Brightness.dark;
    _paintGrid(canvas, size, base, isDark);

    final Path track = _flowBackdropPath(size);
    _paintStaticRail(canvas, track, base, isDark);

    final Color pulse = Color.lerp(Colors.cyanAccent, base, 0.38)!;
    _paintFlowPulse(canvas, track, flowT, pulse);
  }

  @override
  bool shouldRepaint(covariant _ZigzagFlowBackdropPainter oldDelegate) {
    return oldDelegate.seedColor != seedColor ||
        oldDelegate.surfaceColor != surfaceColor ||
        oldDelegate.brightness != brightness ||
        oldDelegate.animation != animation;
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
