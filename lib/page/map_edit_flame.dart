import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:flame/events.dart';
import 'package:ros_flutter_gui_app/display/map.dart';
import 'package:ros_flutter_gui_app/display/grid.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';

// 专门的地图编辑Flame组件
class MapEditFlame extends FlameGame with ScrollDetector, ScaleDetector, TapDetector {
  late MapComponent _displayMap;
  late GridComponent _displayGrid;
  final RosChannel? rosChannel;
  
  final double minScale = 0.1;
  final double maxScale = 10.0;
  
  // 地图变换参数
  double mapScale = 1.0;
  Vector2 mapOffset = Vector2.zero();
  
  // 当前选中的编辑工具
  String? selectedTool;
  
  // 回调函数，用于通知外部添加导航点
  Function(double x, double y)? onAddNavPoint;
  
  MapEditFlame({this.rosChannel, this.onAddNavPoint});
  
  // 设置当前选中的工具
  void setSelectedTool(String? tool) {
    selectedTool = tool;
  }
  
  @override
  Future<void> onLoad() async {
    super.onLoad();
    
    // 添加地图组件
    _displayMap = MapComponent(rosChannel: rosChannel);
    world.add(_displayMap);
    _displayMap.priority = 900;
    
    // 添加网格组件
    _displayGrid = GridComponent(
      size: size,
      rosChannel: rosChannel,
    );
    _displayGrid.priority = 910;
    
    // 设置ROS监听器
    _setupRosListeners();
  }
  
  void _setupRosListeners() {
    if (rosChannel != null) {
      // 监听地图数据
      rosChannel!.map_.addListener(() {
        _displayMap.updateMapData(rosChannel!.map_.value);
      });
      
      // 立即更新地图数据
      _displayMap.updateMapData(rosChannel!.map_.value);
    }
  }
  
  // 处理点击事件
  @override
  bool onTapDown(TapDownInfo info) {
    if (selectedTool == 'addNavPoint') {
      // 将屏幕坐标转换为世界坐标
      final worldPoint = camera.globalToLocal(info.eventPosition.global);
      
      // 调用回调函数添加导航点
      if (onAddNavPoint != null) {
        onAddNavPoint!(worldPoint.x, worldPoint.y);
      }
      return true;
    }
    return false;
  }
  
  // 手势和滚轮缩放支持
  double _baseScale = 1.0;
  Vector2? _lastFocalPoint;
  
  @override
  bool onScroll(PointerScrollInfo info) {
    // 滚轮缩放
    const zoomSensitivity = 0.5;
    double zoomChange = -info.scrollDelta.global.y.sign * zoomSensitivity;
    final newZoom = (camera.viewfinder.zoom + zoomChange).clamp(minScale, maxScale);
    
    // 获取鼠标位置在世界坐标中的位置
    final mousePosition = info.eventPosition.global;
    final worldPoint = camera.globalToLocal(mousePosition);
    
    // 应用缩放
    camera.viewfinder.zoom = newZoom;
    mapScale = newZoom;
    
    // 调整相机位置以保持鼠标位置不变
    final newScreenPoint = camera.localToGlobal(worldPoint);
    final offset = mousePosition - newScreenPoint;
    camera.viewfinder.position -= offset / camera.viewfinder.zoom;
    
    return true;
  }
  
  @override
  bool onScaleStart(ScaleStartInfo info) {
    _baseScale = mapScale;
    _lastFocalPoint = info.eventPosition.global;
    return true;
  }
  
  @override
  bool onScaleUpdate(ScaleUpdateInfo info) {
    if (_lastFocalPoint == null) return false;
    
    // 计算新的缩放值
    final newScale = (_baseScale * info.scale.global.x).clamp(minScale, maxScale);
    
    // 应用缩放
    mapScale = newScale;
    camera.viewfinder.zoom = mapScale;
    
    // 计算焦点偏移
    final focalPointDelta = info.eventPosition.global - _lastFocalPoint!;
    camera.viewfinder.position -= focalPointDelta / camera.viewfinder.zoom;
    
    _lastFocalPoint = info.eventPosition.global;
    return true;
  }
  
  @override
  bool onScaleEnd(ScaleEndInfo info) {
    _lastFocalPoint = null;
    return true;
  }
  
  // 缩放控制方法
  void zoomIn() {
    mapScale = (mapScale * 1.2).clamp(minScale, maxScale);
    camera.viewfinder.zoom = mapScale;
  }
  
  void zoomOut() {
    mapScale = (mapScale / 1.2).clamp(minScale, maxScale);
    camera.viewfinder.zoom = mapScale;
  }
  
  void resetZoom() {
    mapScale = 1.0;
    camera.viewfinder.zoom = mapScale;
    camera.viewfinder.position = Vector2.zero();
  }
}
