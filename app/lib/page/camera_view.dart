import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/provider/ws_channel.dart';

class CameraView extends StatefulWidget {
  const CameraView({
    super.key,
    required this.width,
    required this.height,
  });

  final double width;
  final double height;

  @override
  State<CameraView> createState() => _CameraViewState();
}

class _CameraViewState extends State<CameraView> {
  WsChannel? _ws;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _ws ??= context.read<WsChannel>();
  }

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) {
        return;
      }
      _ws?.beginImageStream(globalSetting.imageTopic);
    });
  }

  @override
  void dispose() {
    _ws?.endImageStream();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final streamError =
        context.select<WsChannel, String?>((c) => c.streamError);
    if (streamError != null) {
      return Container(
        width: widget.width,
        height: widget.height,
        color: Colors.grey[300],
        child: Icon(Icons.videocam_off, size: 48, color: Colors.grey[600]),
      );
    }
    final prov = context.read<WsChannel>();
    return RepaintBoundary(
      child: ListenableBuilder(
        listenable: prov.imageFrame,
        builder: (context, _) {
          final bytes = prov.imageFrame.value;
          if (bytes == null) {
            return SizedBox(
              width: widget.width,
              height: widget.height,
              child:
                  const Center(child: CircularProgressIndicator(strokeWidth: 2)),
            );
          }
          return _AsyncCameraImage(
            bytes: bytes,
            width: widget.width,
            height: widget.height,
          );
        },
      ),
    );
  }
}

class _AsyncCameraImage extends StatefulWidget {
  const _AsyncCameraImage({
    required this.bytes,
    required this.width,
    required this.height,
  });

  final Uint8List bytes;
  final double width;
  final double height;

  @override
  State<_AsyncCameraImage> createState() => _AsyncCameraImageState();
}

class _AsyncCameraImageState extends State<_AsyncCameraImage> {
  ui.Image? _image;

  @override
  void initState() {
    super.initState();
    _decode();
  }

  @override
  void didUpdateWidget(covariant _AsyncCameraImage oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.bytes != widget.bytes) {
      _decode();
    }
  }

  Future<void> _decode() async {
    final codec = await ui.instantiateImageCodec(widget.bytes);
    final frame = await codec.getNextFrame();
    if (!mounted) {
      return;
    }
    setState(() {
      _image = frame.image;
    });
  }

  @override
  void dispose() {
    _image?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final img = _image;
    if (img == null) {
      return SizedBox(
        width: widget.width,
        height: widget.height,
        child: const Center(child: CircularProgressIndicator(strokeWidth: 2)),
      );
    }
    return CustomPaint(
      size: Size(widget.width, widget.height),
      painter: _ImagePainter(img),
    );
  }
}

class _ImagePainter extends CustomPainter {
  _ImagePainter(this.image);

  final ui.Image image;

  @override
  void paint(Canvas canvas, Size size) {
    final src =
        Rect.fromLTWH(0, 0, image.width.toDouble(), image.height.toDouble());
    final dst = Rect.fromLTWH(0, 0, size.width, size.height);
    canvas.drawImageRect(image, src, dst, Paint());
  }

  @override
  bool shouldRepaint(covariant _ImagePainter oldDelegate) {
    return oldDelegate.image != image;
  }
}
