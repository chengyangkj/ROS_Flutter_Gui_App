String _shellSingleQuoted(String s) {
  return "'${s.replaceAll("'", "'\\''")}'";
}

String _stripLeadingSudoToken(String cmd) {
  var s = cmd.trim();
  if (s.startsWith('sudo ')) {
    return s.substring(5).trimLeft();
  }
  return s;
}

String sshQuickCmdRemoteLine(SshQuickCmd c, String sshPassword) {
  final raw = c.cmd.trim();
  if (!c.useSudo) {
    return raw;
  }
  final cmd = _stripLeadingSudoToken(raw);
  if (cmd.isEmpty) {
    return raw;
  }
  final pass = sshPassword;
  if (pass.isEmpty) {
    return raw;
  }
  return 'echo ${_shellSingleQuoted(pass)} | sudo -S sh -c ${_shellSingleQuoted(cmd)}';
}

class SshQuickCmd {
  final String name;
  final String cmd;
  final bool useSudo;

  const SshQuickCmd({
    required this.name,
    required this.cmd,
    this.useSudo = false,
  });

  Map<String, dynamic> toJson() => {
        'name': name,
        'cmd': cmd,
        'useSudo': useSudo,
      };

  factory SshQuickCmd.fromJson(Map<String, dynamic> j) {
    bool useFlag = false;
    final u = j['useSudo'];
    if (u is bool) {
      useFlag = u;
    } else if (u is String && u.toLowerCase() == 'true') {
      useFlag = true;
    }
    return SshQuickCmd(
      name: '${j['name'] ?? ''}',
      cmd: '${j['cmd'] ?? ''}',
      useSudo: useFlag,
    );
  }

  SshQuickCmd copyWith({String? name, String? cmd, bool? useSudo}) {
    return SshQuickCmd(
      name: name ?? this.name,
      cmd: cmd ?? this.cmd,
      useSudo: useSudo ?? this.useSudo,
    );
  }
}

List<SshQuickCmd> defaultSshQuickCommands() => const [
      SshQuickCmd(name: '关机', cmd: 'shutdown -h now', useSudo: true),
      SshQuickCmd(name: '重启', cmd: 'reboot', useSudo: true),
    ];
