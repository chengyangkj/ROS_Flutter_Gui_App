import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:ros_flutter_gui_app/ssh/ssh_remote.dart';

class _SgrState {
  _SgrState();

  Color? fg;
  Color? bg;
  FontWeight fontWeight = FontWeight.normal;
  FontStyle fontStyle = FontStyle.normal;
  TextDecoration decoration = TextDecoration.none;

  void reset(TextStyle base) {
    fg = null;
    bg = null;
    fontWeight = FontWeight.normal;
    fontStyle = FontStyle.normal;
    decoration = TextDecoration.none;
  }

  TextStyle toTextStyle(TextStyle base) {
    return base.copyWith(
      color: fg ?? base.color,
      backgroundColor: bg,
      fontWeight: fontWeight,
      fontStyle: fontStyle,
      decoration: decoration,
    );
  }
}

const List<Color> _ansiFg = [
  Color(0xFF000000),
  Color(0xFFCD3131),
  Color(0xFF0DBC79),
  Color(0xFFE5E510),
  Color(0xFF2472C8),
  Color(0xFFBC3FBC),
  Color(0xFF11A8CD),
  Color(0xFFE5E5E5),
];

const List<Color> _ansiBrightFg = [
  Color(0xFF666666),
  Color(0xFFF14C4C),
  Color(0xFF23D18B),
  Color(0xFFF5F543),
  Color(0xFF3B8EEA),
  Color(0xFFD670D6),
  Color(0xFF29B8DB),
  Color(0xFFFFFFFF),
];

Color _xterm256Color(int n) {
  if (n < 0) return _ansiFg[7];
  if (n < 8) return _ansiFg[n];
  if (n < 16) return _ansiBrightFg[n - 8];
  if (n < 232) {
    final v = n - 16;
    final b = v % 6;
    final g = (v ~/ 6) % 6;
    final r = v ~/ 36;
    int ch(int x) => x == 0 ? 0 : (55 + x * 40);
    return Color.fromARGB(255, ch(r), ch(g), ch(b));
  }
  if (n < 256) {
    final g = 8 + (n - 232) * 10;
    return Color.fromARGB(255, g, g, g);
  }
  return _ansiFg[7];
}

List<TextSpan> _ansiStringToSpans(String input, TextStyle baseStyle) {
  final spans = <TextSpan>[];
  final buf = StringBuffer();
  var st = _SgrState();

  void flush() {
    if (buf.isEmpty) return;
    spans.add(TextSpan(text: buf.toString(), style: st.toTextStyle(baseStyle)));
    buf.clear();
  }

  var i = 0;
  while (i < input.length) {
    final u = input.codeUnitAt(i);
    if (u == 0x1b) {
      flush();
      if (i + 1 < input.length && input.codeUnitAt(i + 1) == 0x5b) {
        var j = i + 2;
        while (j < input.length) {
          final c = input.codeUnitAt(j);
          if (c >= 0x40 && c <= 0x7E) {
            final cmd = String.fromCharCode(c);
            final mid = input.substring(i + 2, j);
            if (cmd == 'm') {
              _applySgrMid(mid, st, baseStyle);
            }
            i = j + 1;
            break;
          }
          j++;
        }
        if (j >= input.length || input.codeUnitAt(j) < 0x40) {
          i++;
        }
        continue;
      }
      if (i + 1 < input.length && input.codeUnitAt(i + 1) == 0x5d) {
        var j = i + 2;
        while (j < input.length) {
          final c = input.codeUnitAt(j);
          if (c == 0x07) {
            i = j + 1;
            break;
          }
          if (c == 0x1b &&
              j + 1 < input.length &&
              input.codeUnitAt(j + 1) == 0x5c) {
            i = j + 2;
            break;
          }
          j++;
        }
        if (j >= input.length) {
          i++;
        }
        continue;
      }
      i++;
      continue;
    }
    if (u == 0x0d) {
      flush();
      i++;
      continue;
    }
    buf.writeCharCode(u);
    i++;
  }
  flush();
  return spans;
}

void _applySgrMid(String mid, _SgrState st, TextStyle baseStyle) {
  List<int> codes;
  if (mid.isEmpty) {
    codes = [0];
  } else {
    codes = mid.split(';').map((s) => int.tryParse(s) ?? 0).toList();
  }
  var k = 0;
  while (k < codes.length) {
    final c = codes[k];
    switch (c) {
      case 0:
        st.reset(baseStyle);
        break;
      case 1:
        st.fontWeight = FontWeight.bold;
        break;
      case 2:
        st.fontWeight = FontWeight.w300;
        break;
      case 22:
        st.fontWeight = FontWeight.normal;
        break;
      case 3:
        st.fontStyle = FontStyle.italic;
        break;
      case 23:
        st.fontStyle = FontStyle.normal;
        break;
      case 4:
        st.decoration = TextDecoration.underline;
        break;
      case 24:
        st.decoration = TextDecoration.none;
        break;
      case 7:
        if (st.fg == null && st.bg == null) {
          st.fg = const Color(0xFF111111);
          st.bg = const Color(0xFFDDDDDD);
        } else {
          final t = st.fg;
          st.fg = st.bg;
          st.bg = t;
        }
        break;
      case 27:
        st.fg = null;
        st.bg = null;
        break;
      case 39:
        st.fg = null;
        break;
      case 49:
        st.bg = null;
        break;
      default:
        if (c >= 30 && c <= 37) {
          st.fg = _ansiFg[c - 30];
        } else if (c >= 40 && c <= 47) {
          st.bg = _ansiFg[c - 40];
        } else if (c >= 90 && c <= 97) {
          st.fg = _ansiBrightFg[c - 90];
        } else if (c >= 100 && c <= 107) {
          st.bg = _ansiBrightFg[c - 100];
        } else if (c == 38 && k + 2 < codes.length && codes[k + 1] == 5) {
          st.fg = _xterm256Color(codes[k + 2]);
          k += 2;
        } else if (c == 38 && k + 4 < codes.length && codes[k + 1] == 2) {
          final r = codes[k + 2].clamp(0, 255);
          final g = codes[k + 3].clamp(0, 255);
          final b = codes[k + 4].clamp(0, 255);
          st.fg = Color.fromARGB(255, r, g, b);
          k += 4;
        } else if (c == 48 && k + 2 < codes.length && codes[k + 1] == 5) {
          st.bg = _xterm256Color(codes[k + 2]);
          k += 2;
        } else if (c == 48 && k + 4 < codes.length && codes[k + 1] == 2) {
          final r = codes[k + 2].clamp(0, 255);
          final g = codes[k + 3].clamp(0, 255);
          final b = codes[k + 4].clamp(0, 255);
          st.bg = Color.fromARGB(255, r, g, b);
          k += 4;
        }
        break;
    }
    k++;
  }
}

class SshTerminalPage extends StatefulWidget {
  const SshTerminalPage({super.key});

  @override
  State<SshTerminalPage> createState() => _SshTerminalPageState();
}

class _SshTerminalPageState extends State<SshTerminalPage> {
  final _scroll = ScrollController();
  final _inputFocus = FocusNode();
  final _input = TextEditingController();
  final _buffer = StringBuffer();
  Object? _client;
  Object? _session;
  StreamSubscription<Uint8List>? _subOut;
  StreamSubscription<Uint8List>? _subErr;
  bool _busy = true;
  String? _error;

  @override
  void initState() {
    super.initState();
    _connect();
  }

  Future<void> _connect() async {
    if (!sshRemotePlatformSupported) {
      setState(() {
        _busy = false;
        _error = '当前平台不支持 SSH';
      });
      return;
    }
    setState(() {
      _busy = true;
      _error = null;
    });
    try {
      final client = await sshConnect(
        username: globalSetting.sshUsername.trim(),
        password: globalSetting.sshPassword,
      );
      final session = await sshStartShell(client);
      _client = client;
      _session = session;
      _subOut = sshSessionListenStdout(session, _onBytes);
      _subErr = sshSessionListenStderr(session, _onBytes);
      if (mounted) {
        setState(() => _busy = false);
        WidgetsBinding.instance.addPostFrameCallback((_) {
          if (mounted && _session != null) {
            _inputFocus.requestFocus();
          }
        });
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _busy = false;
          _error = '$e';
        });
      }
    }
  }

  void _onBytes(Uint8List data) {
    final s = utf8.decode(data, allowMalformed: true);
    _buffer.write(s);
    if (mounted) {
      setState(() {});
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (_scroll.hasClients) {
          _scroll.jumpTo(_scroll.position.maxScrollExtent);
        }
      });
    }
  }

  void _sendLine() {
    final line = _input.text;
    _input.clear();
    if (_session == null || line.isEmpty) {
      _inputFocus.requestFocus();
      return;
    }
    sshSessionWriteBytes(_session!, Uint8List.fromList(utf8.encode('$line\n')));
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (mounted) {
        _inputFocus.requestFocus();
      }
    });
  }

  @override
  void dispose() {
    _subOut?.cancel();
    _subErr?.cancel();
    sshSessionClose(_session);
    sshClientClose(_client);
    _scroll.dispose();
    _inputFocus.dispose();
    _input.dispose();
    super.dispose();
  }

  Widget _buildTerminalText(BuildContext context) {
    final theme = Theme.of(context);
    final baseStyle = TextStyle(
      fontFamily: 'monospace',
      fontSize: 12,
      height: 1.25,
      color: theme.colorScheme.onSurface,
    );
    final raw = _buffer.toString();
    if (raw.isEmpty) {
      return SelectableText(
        '(已连接，输入命令后回车)',
        style: baseStyle.copyWith(color: theme.hintColor),
        textAlign: TextAlign.start,
      );
    }
    final spans = _ansiStringToSpans(raw, baseStyle);
    if (spans.isEmpty) {
      return SelectableText(
        '',
        style: baseStyle,
        textAlign: TextAlign.start,
      );
    }
    return SelectableText.rich(
      TextSpan(children: spans),
      style: baseStyle,
      textAlign: TextAlign.start,
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('SSH 终端'),
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: _busy ? null : () async {
              _subOut?.cancel();
              _subErr?.cancel();
              sshSessionClose(_session);
              sshClientClose(_client);
              _session = null;
              _client = null;
              _buffer.clear();
              await _connect();
            },
          ),
        ],
      ),
      body: Column(
        children: [
          if (_error != null)
            MaterialBanner(
              content: SelectableText(_error!),
              actions: [
                TextButton(
                  onPressed: () => setState(() => _error = null),
                  child: const Text('关闭'),
                ),
              ],
            ),
          Expanded(
            child: _busy
                ? const Center(child: CircularProgressIndicator())
                : SingleChildScrollView(
                    controller: _scroll,
                    padding: const EdgeInsets.all(8),
                    child: LayoutBuilder(
                      builder: (context, c) {
                        return ConstrainedBox(
                          constraints: BoxConstraints(minWidth: c.maxWidth),
                          child: Align(
                            alignment: Alignment.topLeft,
                            child: _buildTerminalText(context),
                          ),
                        );
                      },
                    ),
                  ),
          ),
          Padding(
            padding: const EdgeInsets.fromLTRB(8, 0, 8, 8),
            child: Row(
              children: [
                Expanded(
                  child: TextField(
                    focusNode: _inputFocus,
                    controller: _input,
                    enabled: !_busy && _session != null,
                    decoration: const InputDecoration(
                      hintText: '命令',
                      border: OutlineInputBorder(),
                      isDense: true,
                    ),
                    onSubmitted: (_) => _sendLine(),
                  ),
                ),
                const SizedBox(width: 8),
                FilledButton(
                  onPressed:
                      (_busy || _session == null) ? null : _sendLine,
                  child: const Text('发送'),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
