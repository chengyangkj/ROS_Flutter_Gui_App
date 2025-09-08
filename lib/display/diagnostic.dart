import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/basic/diagnostic_array.dart';
import 'package:ros_flutter_gui_app/basic/diagnostic_status.dart';

class DiagnosticDisplay extends StatelessWidget {
  final DiagnosticArray diagnosticData;

  const DiagnosticDisplay({
    super.key,
    required this.diagnosticData,
  });

  @override
  Widget build(BuildContext context) {
    if (diagnosticData.status.isEmpty) {
      return const Center(
        child: Text(
          '暂无诊断数据',
          style: TextStyle(
            color: Colors.grey,
            fontSize: 16,
          ),
        ),
      );
    }

    return ListView.builder(
      itemCount: diagnosticData.status.length,
      itemBuilder: (context, index) {
        final status = diagnosticData.status[index];
        return _buildDiagnosticItem(status);
      },
    );
  }

  Widget _buildDiagnosticItem(DiagnosticStatus status) {
    Color statusColor;
    IconData statusIcon;
    
    switch (status.level) {
      case DiagnosticStatus.OK:
        statusColor = Colors.green;
        statusIcon = Icons.check_circle;
        break;
      case DiagnosticStatus.WARN:
        statusColor = Colors.orange;
        statusIcon = Icons.warning;
        break;
      case DiagnosticStatus.ERROR:
        statusColor = Colors.red;
        statusIcon = Icons.error;
        break;
      case DiagnosticStatus.STALE:
        statusColor = Colors.grey;
        statusIcon = Icons.schedule;
        break;
      default:
        statusColor = Colors.grey;
        statusIcon = Icons.help;
    }

    return Card(
      margin: const EdgeInsets.symmetric(vertical: 4, horizontal: 8),
      child: ExpansionTile(
        leading: Icon(statusIcon, color: statusColor),
        title: Text(
          status.name,
          style: const TextStyle(
            fontWeight: FontWeight.bold,
            fontSize: 16,
          ),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              '状态: ${status.levelDisplayName}',
              style: TextStyle(
                color: statusColor,
                fontWeight: FontWeight.w500,
              ),
            ),
            if (status.message.isNotEmpty)
              Text(
                status.message,
                style: const TextStyle(
                  fontSize: 12,
                  color: Colors.grey,
                ),
                maxLines: 2,
                overflow: TextOverflow.ellipsis,
              ),
          ],
        ),
        children: [
          if (status.hardwareId.isNotEmpty)
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
              child: Row(
                children: [
                  const Text(
                    '硬件ID: ',
                    style: TextStyle(fontWeight: FontWeight.bold),
                  ),
                  Expanded(
                    child: Text(
                      status.hardwareId,
                      style: const TextStyle(fontSize: 12),
                    ),
                  ),
                ],
              ),
            ),
          if (status.values.isNotEmpty)
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text(
                    '详细信息:',
                    style: TextStyle(
                      fontWeight: FontWeight.bold,
                      fontSize: 14,
                    ),
                  ),
                  const SizedBox(height: 8),
                  ...status.values.map((kv) => Padding(
                    padding: const EdgeInsets.symmetric(vertical: 2),
                    child: Row(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        SizedBox(
                          width: 100,
                          child: Text(
                            '${kv.key}:',
                            style: const TextStyle(
                              fontWeight: FontWeight.w500,
                              fontSize: 12,
                            ),
                          ),
                        ),
                        Expanded(
                          child: Text(
                            kv.value,
                            style: const TextStyle(fontSize: 12),
                          ),
                        ),
                      ],
                    ),
                  )),
                ],
              ),
            ),
        ],
      ),
    );
  }
}

class DiagnosticSummary extends StatelessWidget {
  final DiagnosticArray diagnosticData;

  const DiagnosticSummary({
    super.key,
    required this.diagnosticData,
  });

  @override
  Widget build(BuildContext context) {
    if (diagnosticData.status.isEmpty) {
      return const SizedBox.shrink();
    }

    int okCount = 0;
    int warnCount = 0;
    int errorCount = 0;
    int staleCount = 0;

    for (var status in diagnosticData.status) {
      switch (status.level) {
        case DiagnosticStatus.OK:
          okCount++;
          break;
        case DiagnosticStatus.WARN:
          warnCount++;
          break;
        case DiagnosticStatus.ERROR:
          errorCount++;
          break;
        case DiagnosticStatus.STALE:
          staleCount++;
          break;
      }
    }

    return Card(
      margin: const EdgeInsets.all(8),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              '诊断状态总览',
              style: TextStyle(
                fontSize: 18,
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(height: 12),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: [
                _buildStatusChip('正常', okCount, Colors.green),
                _buildStatusChip('警告', warnCount, Colors.orange),
                _buildStatusChip('错误', errorCount, Colors.red),
                _buildStatusChip('过期', staleCount, Colors.grey),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildStatusChip(String label, int count, Color color) {
    return Column(
      children: [
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
          decoration: BoxDecoration(
            color: color.withOpacity(0.1),
            borderRadius: BorderRadius.circular(16),
            border: Border.all(color: color.withOpacity(0.3)),
          ),
          child: Text(
            count.toString(),
            style: TextStyle(
              color: color,
              fontWeight: FontWeight.bold,
              fontSize: 16,
            ),
          ),
        ),
        const SizedBox(height: 4),
        Text(
          label,
          style: TextStyle(
            color: color,
            fontSize: 12,
            fontWeight: FontWeight.w500,
          ),
        ),
      ],
    );
  }
}
