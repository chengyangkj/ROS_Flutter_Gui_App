import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/basic/diagnostic_status.dart';
import 'package:ros_flutter_gui_app/provider/diagnostic_manager.dart';

class DiagnosticPage extends StatefulWidget {
  const DiagnosticPage({super.key});

  @override
  State<DiagnosticPage> createState() => _DiagnosticPageState();
}

class _DiagnosticPageState extends State<DiagnosticPage> {
  late RosChannel rosChannel;
  String _searchQuery = '';
  int _filterLevel = -1; // -1: 全部, 0: OK, 1: WARN, 2: ERROR, 3: STALE
  Map<String, bool> _expandedHardware = {}; // 硬件ID展开状态
  Map<String, Map<String, bool>> _expandedComponents = {}; // 组件展开状态
  late TextEditingController _searchController;

  @override
  void initState() {
    super.initState();
    rosChannel = Provider.of<RosChannel>(context, listen: false);
    _searchController = TextEditingController(text: _searchQuery);
  }

  @override
  void dispose() {
    _searchController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('系统诊断'),
        backgroundColor: Colors.orange,
        // foregroundColor: Colors.white,
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: () {
              setState(() {});
            },
            tooltip: '刷新',
          ),
        ],
      ),
      body: Column(
        children: [
          
          _buildSummaryBar(),
          _buildFilterBar(),
          Expanded(
            child: Consumer<RosChannel>(
              builder: (context, rosChannel, child) {
                return ListenableBuilder(
                  listenable: rosChannel.diagnosticManager,
                  builder: (context, child) {
                    final diagnosticManager = rosChannel.diagnosticManager;
                    final filteredData = _getFilteredData(diagnosticManager);
                    
                    if (filteredData.isEmpty) {
                      return Center(
                        child: Column(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            Icon(
                              Icons.search_off,
                              size: 64,
                              color: Colors.grey[400],
                            ),
                            const SizedBox(height: 16),
                            Text(
                              _searchQuery.isNotEmpty || _filterLevel != -1
                                  ? '没有找到匹配的诊断数据'
                                  : '暂无诊断数据',
                              style: TextStyle(
                                color: Colors.grey[600],
                                fontSize: 16,
                              ),
                            ),
                            if (_searchQuery.isNotEmpty || _filterLevel != -1) ...[
                              const SizedBox(height: 8),
                              TextButton(
                                onPressed: () {
                                  setState(() {
                                    _searchQuery = '';
                                    _filterLevel = -1;
                                    _searchController.clear();
                                  });
                                },
                                child: const Text('清除筛选条件'),
                              ),
                            ],
                          ],
                        ),
                      );
                    }
                
                return Column(
                  children: [
                        // 筛选结果统计
                        if (_searchQuery.isNotEmpty || _filterLevel != -1)
                          Container(
                            width: double.infinity,
                            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                            child: Row(
                              children: [
                                Icon(Icons.filter_list, color: Colors.blue[600], size: 16),
                                const SizedBox(width: 8),
                                Text(
                                  '显示 ${filteredData.length} 个硬件组',
                                  style: TextStyle(
                                    color: Colors.blue[800],
                                    fontWeight: FontWeight.w500,
                                  ),
                                ),
                                const Spacer(),
                                TextButton(
                                  onPressed: () {
                                    setState(() {
                                      _searchQuery = '';
                                      _filterLevel = -1;
                                      _searchController.clear();
                                    });
                                  },
                                  child: const Text('清除筛选'),
                                ),
                              ],
                            ),
                          ),
                        // 诊断数据列表
                    Expanded(
                          child: ListView.builder(
                            itemCount: filteredData.length,
                            itemBuilder: (context, index) {
                              final entry = filteredData[index];
                              return _buildHardwareGroup(entry.key, entry.value);
                            },
                          ),
                        ),
                      ],
                    );
                  },
                );
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildFilterBar() {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
      
      ),
      child: Column(
        children: [
          // 搜索框
          TextField(
            controller: _searchController,
            decoration: InputDecoration(
              hintText: '搜索组件名称...',
              prefixIcon: const Icon(Icons.search),
              suffixIcon: _searchQuery.isNotEmpty
                  ? IconButton(
                      icon: const Icon(Icons.clear),
                      onPressed: () {
                        setState(() {
                          _searchQuery = '';
                          _searchController.clear();
                        });
                      },
                    )
                  : null,
              border: const OutlineInputBorder(),
              contentPadding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
            ),
            onSubmitted: (value) {
              setState(() {
                _searchQuery = value.toLowerCase();
              });
            },
            onEditingComplete: () {
              setState(() {
                _searchQuery = _searchController.text.toLowerCase();
              });
            },
          ),
          const SizedBox(height: 12),
          // 状态筛选
          Row(
            children: [
              const Text('状态筛选: '),
              const SizedBox(width: 8),
              Expanded(
                child: SingleChildScrollView(
                  scrollDirection: Axis.horizontal,
                  child: Row(
                    children: [
                      _buildFilterChip('全部', -1),
                      const SizedBox(width: 8),
                      _buildFilterChip('正常', 0),
                      const SizedBox(width: 8),
                      _buildFilterChip('警告', 1),
                      const SizedBox(width: 8),
                      _buildFilterChip('错误', 2),
                      const SizedBox(width: 8),
                      _buildFilterChip('过期', 3),
                    ],
                  ),
                ),
              ),
              if (_searchQuery.isNotEmpty || _filterLevel != -1)
                IconButton(
                  icon: const Icon(Icons.clear_all),
                  onPressed: () {
                    setState(() {
                      _searchQuery = '';
                      _filterLevel = -1;
                      _searchController.clear();
                    });
                  },
                  tooltip: '清除所有筛选',
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildFilterChip(String label, int level) {
    final isSelected = _filterLevel == level;
    Color color;
    
    switch (level) {
      case 0:
        color = Colors.green;
        break;
      case 1:
        color = Colors.orange;
        break;
      case 2:
        color = Colors.red;
        break;
      case 3:
        color = Colors.grey;
        break;
      default:
        color = Colors.blue;
    }

    return FilterChip(
      label: Text(label),
      selected: isSelected,
      onSelected: (selected) {
        setState(() {
          _filterLevel = selected ? level : -1;
        });
      },
      selectedColor: color.withOpacity(0.2),
      checkmarkColor: color,
      labelStyle: TextStyle(
        color: isSelected ? color : Colors.black87,
        fontWeight: isSelected ? FontWeight.bold : FontWeight.normal,
      ),
    );
  }

  // 获取筛选后的数据
  List<MapEntry<String, Map<String, DiagnosticState>>> _getFilteredData(DiagnosticManager diagnosticManager) {
    List<MapEntry<String, Map<String, DiagnosticState>>> allData = [];
    
    // 获取所有硬件数据
    for (var hardwareId in diagnosticManager.hardwareIds) {
      final states = diagnosticManager.getStatesForHardware(hardwareId);
      allData.add(MapEntry(hardwareId, states));
    }
    
    // 应用搜索筛选
      if (_searchQuery.isNotEmpty) {
      allData = allData.where((entry) {
        final hardwareId = entry.key.toLowerCase();
        final states = entry.value;
        
        // 检查硬件ID是否匹配
        if (hardwareId.contains(_searchQuery)) {
          return true;
        }
        
        // 检查组件是否匹配
        for (var componentEntry in states.entries) {
          final componentName = componentEntry.key.toLowerCase();
          final state = componentEntry.value;
          
          if (componentName.contains(_searchQuery) ||
              state.message.toLowerCase().contains(_searchQuery) ||
              state.keyValues.values.any((value) => value.toLowerCase().contains(_searchQuery))) {
            return true;
          }
        }
        
        return false;
      }).toList();
    }
    
    // 应用状态筛选
    if (_filterLevel != -1) {
      allData = allData.where((entry) {
        final states = entry.value;
        
        // 检查是否有匹配状态的组件
        return states.values.any((state) => state.level == _filterLevel);
      }).toList();
    }
    
    return allData;
  }

  // 构建硬件组
  Widget _buildHardwareGroup(String hardwareId, Map<String, DiagnosticState> states) {
    final isExpanded = _expandedHardware[hardwareId] ?? false;
    
    // 如果有筛选条件，只显示匹配的组件
    Map<String, DiagnosticState> filteredStates = states;
    if (_searchQuery.isNotEmpty || _filterLevel != -1) {
      filteredStates = Map.fromEntries(
        states.entries.where((entry) {
          final componentName = entry.key;
          final state = entry.value;
          
      // 搜索筛选
      if (_searchQuery.isNotEmpty) {
            final hardwareIdLower = hardwareId.toLowerCase();
            final componentNameLower = componentName.toLowerCase();
            
            if (!hardwareIdLower.contains(_searchQuery) &&
                !componentNameLower.contains(_searchQuery) &&
                !state.message.toLowerCase().contains(_searchQuery) &&
                !state.keyValues.values.any((value) => value.toLowerCase().contains(_searchQuery))) {
          return false;
        }
      }

      // 状态筛选
          if (_filterLevel != -1 && state.level != _filterLevel) {
        return false;
      }

      return true;
        })
      );
    }
    
    // 如果没有匹配的组件，不显示该硬件组
    if (filteredStates.isEmpty) {
      return const SizedBox.shrink();
    }
    
    // 计算该硬件组的最高状态级别
    int maxLevel = DiagnosticStatus.OK;
    for (var state in filteredStates.values) {
      if (state.level > maxLevel) {
        maxLevel = state.level;
      }
    }
    
    Color groupColor;
    IconData groupIcon;
    String groupStatusText;
    
    switch (maxLevel) {
      case DiagnosticStatus.ERROR:
        groupColor = Colors.red;
        groupIcon = Icons.error;
        groupStatusText = '错误';
        break;
      case DiagnosticStatus.WARN:
        groupColor = Colors.orange;
        groupIcon = Icons.warning;
        groupStatusText = '警告';
        break;
      case DiagnosticStatus.STALE:
        groupColor = Colors.grey;
        groupIcon = Icons.schedule;
        groupStatusText = '过期';
        break;
      default:
        groupColor = Colors.green;
        groupIcon = Icons.check_circle;
        groupStatusText = '正常';
    }

    return Card(
      margin: const EdgeInsets.symmetric(vertical: 4, horizontal: 8),
      child: ExpansionTile(
        leading: Icon(groupIcon, color: groupColor),
        title: Text(
          hardwareId,
          style: const TextStyle(
            fontWeight: FontWeight.bold,
            fontSize: 16,
          ),
        ),
        subtitle: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              '状态: $groupStatusText | 组件数: ${filteredStates.length}',
              style: TextStyle(
                color: groupColor,
                fontWeight: FontWeight.w500,
              ),
            ),
            const SizedBox(height: 2),
            Text(
              '最后更新: ${_formatDateTime(_getLatestUpdateTime(filteredStates))}',
              style: const TextStyle(
                fontSize: 11,
                color: Colors.grey,
                fontStyle: FontStyle.italic,
              ),
            ),
          ],
        ),
        initiallyExpanded: isExpanded,
        onExpansionChanged: (expanded) {
          setState(() {
            _expandedHardware[hardwareId] = expanded;
          });
        },
        children: filteredStates.entries.map((entry) => _buildComponentItem(hardwareId, entry.key, entry.value)).toList(),
      ),
    );
  }

  // 构建组件项
  Widget _buildComponentItem(String hardwareId, String componentName, DiagnosticState state) {
    final isExpanded = _expandedComponents[hardwareId]?[componentName] ?? false;
    
    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 2),
      child: Card(
        elevation: 2,
        child: ExpansionTile(
          leading: Icon(state.levelIcon, color: state.levelColor, size: 20),
          title: Text(
            componentName,
            style: const TextStyle(
              fontWeight: FontWeight.w600,
              fontSize: 14,
            ),
          ),
          subtitle: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                '状态: ${state.levelDisplayName}',
                style: TextStyle(
                  color: state.levelColor,
                  fontWeight: FontWeight.w500,
                  fontSize: 12,
                ),
              ),
              if (state.message.isNotEmpty)
                Text(
                  state.message,
                  style: const TextStyle(
                    fontSize: 11,
                    color: Colors.grey,
                  ),
                  maxLines: 2,
                  overflow: TextOverflow.ellipsis,
                ),
              const SizedBox(height: 4),
              Text(
                '更新时间: ${_formatDateTime(state.lastUpdateTime)}',
                style: const TextStyle(
                  fontSize: 10,
                  color: Colors.grey,
                  fontStyle: FontStyle.italic,
                ),
              ),
            ],
          ),
          initiallyExpanded: isExpanded,
          onExpansionChanged: (expanded) {
            setState(() {
              if (!_expandedComponents.containsKey(hardwareId)) {
                _expandedComponents[hardwareId] = {};
              }
              _expandedComponents[hardwareId]![componentName] = expanded;
            });
          },
          children: [
            if (state.keyValues.isNotEmpty)
              _buildKeyValueTable(state.keyValues, state.lastUpdateTime)
            else
              Padding(
                padding: const EdgeInsets.all(16),
                child: Column(
                  children: [
                    const Text(
                      '暂无详细信息',
                      style: TextStyle(
                        color: Colors.grey,
                        fontStyle: FontStyle.italic,
                      ),
                    ),
                    const SizedBox(height: 8),
                    Text(
                      '最后更新: ${_formatDateTime(state.lastUpdateTime)}',
                      style: const TextStyle(
                        fontSize: 10,
                        color: Colors.grey,
                        fontStyle: FontStyle.italic,
                      ),
                    ),
                  ],
                ),
              ),
          ],
        ),
      ),
    );
  }

  // 构建键值对表格
  Widget _buildKeyValueTable(Map<String, String> keyValues, DateTime lastUpdateTime) {
    return Container(
      margin: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        border: Border.all(color: Colors.grey[300]!),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Column(
        children: [
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
            decoration: BoxDecoration(
              color: Colors.grey[100],
              borderRadius: const BorderRadius.only(
                topLeft: Radius.circular(8),
                topRight: Radius.circular(8),
              ),
            ),
            child: Row(
              children: [
                const Expanded(
                  flex: 2,
                  child: Text(
                    '键',
                    style: TextStyle(
                      fontWeight: FontWeight.bold,
                      fontSize: 12,
                    ),
                  ),
                ),
                const Expanded(
                  flex: 3,
                  child: Text(
                    '值',
                    style: TextStyle(
                      fontWeight: FontWeight.bold,
                      fontSize: 12,
                    ),
                  ),
                ),
                Expanded(
                  flex: 2,
                  child: Text(
                    '更新时间: ${_formatDateTime(lastUpdateTime)}',
                    style: const TextStyle(
                      fontWeight: FontWeight.bold,
                      fontSize: 10,
                      color: Colors.grey,
                    ),
                    textAlign: TextAlign.right,
                  ),
                ),
              ],
            ),
          ),
          ...keyValues.entries.map((entry) => Container(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
            decoration: BoxDecoration(
              border: Border(
                top: BorderSide(color: Colors.grey[200]!),
              ),
            ),
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Expanded(
                  flex: 2,
                  child: Text(
                    entry.key,
                    style: const TextStyle(
                      fontWeight: FontWeight.w500,
                      fontSize: 11,
                    ),
                  ),
                ),
                Expanded(
                  flex: 3,
                  child: Text(
                    entry.value,
                    style: const TextStyle(fontSize: 11),
                  ),
                ),
                const Expanded(
                  flex: 2,
                  child: SizedBox(), // 占位符，保持对齐
                ),
              ],
            ),
          )),
        ],
      ),
    );
  }

  // 构建摘要栏
  Widget _buildSummaryBar() {
    return Consumer<RosChannel>(
      builder: (context, rosChannel, child) {
        return ListenableBuilder(
          listenable: rosChannel.diagnosticManager,
          builder: (context, child) {
            final diagnosticManager = rosChannel.diagnosticManager;
            final statusCounts = diagnosticManager.getStatusCounts();
            
            if (statusCounts.values.every((count) => count == 0)) {
              return const SizedBox.shrink();
            }

            int okCount = statusCounts[DiagnosticStatus.OK] ?? 0;
            int warnCount = statusCounts[DiagnosticStatus.WARN] ?? 0;
            int errorCount = statusCounts[DiagnosticStatus.ERROR] ?? 0;
            int staleCount = statusCounts[DiagnosticStatus.STALE] ?? 0;

            return Container(
              padding: const EdgeInsets.all(16),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceAround,
                children: [
                  _buildSummaryChip('正常', okCount, Colors.green),
                  _buildSummaryChip('警告', warnCount, Colors.orange),
                  _buildSummaryChip('错误', errorCount, Colors.red),
                  _buildSummaryChip('过期', staleCount, Colors.grey),
                ],
              ),
            );
          },
        );
      },
    );
  }

  Widget _buildSummaryChip(String label, int count, Color color) {
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

  // 获取硬件组中所有组件的最新更新时间
  DateTime _getLatestUpdateTime(Map<String, DiagnosticState> states) {
    if (states.isEmpty) return DateTime.now();
    
    DateTime latestTime = states.values.first.lastUpdateTime;
    for (var state in states.values) {
      if (state.lastUpdateTime.isAfter(latestTime)) {
        latestTime = state.lastUpdateTime;
      }
    }
    return latestTime;
  }

  // 格式化时间显示
  String _formatDateTime(DateTime dateTime) {
    final milliseconds = dateTime.millisecond.toString().padLeft(3, '0');
    return '${dateTime.year}-${dateTime.month}-${dateTime.day} ${dateTime.hour.toString().padLeft(2, '0')}:${dateTime.minute.toString().padLeft(2, '0')}:${dateTime.second.toString().padLeft(2, '0')}.$milliseconds';
  }
}
