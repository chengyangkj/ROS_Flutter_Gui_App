import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:dio/dio.dart';
import 'package:http/http.dart' as http;

class StreamImage extends StatefulWidget {
  final String url;

  const StreamImage({Key? key, required this.url}) : super(key: key);

  @override
  _StreamImageState createState() => _StreamImageState();
}

class _StreamImageState extends State<StreamImage> {
  final StreamController<Uint8List> _streamController =
      StreamController<Uint8List>();
  List<int> _bytes = [];
  Uint8List lastImageData = Uint8List(0);

  @override
  void initState() {
    super.initState();
    _loadImagesData();
  }

  Future _loadImagesData() async {
    var uri = Uri.parse(widget.url);
    var response = await http.Client().send(http.Request('GET', uri));
    response.stream.listen((value) {
      _bytes.addAll(value);
      if (lastImageData.length == 36) {
        _streamController.sink.add(Uint8List.fromList(_bytes));
        _bytes.clear();
      }
      lastImageData = Uint8List.fromList(value);
    });
  }

  @override
  void dispose() {
    _streamController.close();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return StreamBuilder<Uint8List>(
      stream: _streamController.stream,
      initialData: Uint8List(0),
      builder: (BuildContext context, AsyncSnapshot<Uint8List> snapshot) {
        if (snapshot.connectionState == ConnectionState.waiting) {
          return Center(child: CircularProgressIndicator());
        } else if (snapshot.hasError) {
          return Center(child: Text('Error: ${snapshot.error}'));
        } else if (snapshot.hasData) {
          return Image.memory(snapshot.data!);
        } else {
          return Center(child: Text('No image data'));
        }
      },
    );
  }
}
