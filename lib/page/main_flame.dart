import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:flame_svg/flame_svg.dart';
import 'package:flame_svg/svg_component.dart';
import 'package:flutter/material.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/display/map.dart';
import 'package:ros_flutter_gui_app/display/grid.dart';
import 'package:ros_flutter_gui_app/display/pointcloud.dart';
import 'package:ros_flutter_gui_app/display/path.dart';
import 'package:ros_flutter_gui_app/display/laser.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/display/costmap.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';
import 'package:ros_flutter_gui_app/display/pose.dart';
import 'package:ros_flutter_gui_app/display/topology_line.dart';
import 'package:ros_flutter_gui_app/display/polygon.dart' as custom;
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/them_provider.dart';
import 'package:ros_flutter_gui_app/global/setting.dart';
import 'package:vector_math/vector_math_64.dart' as vm;
import 'package:ros_flutter_gui_app/provider/nav_point_manager.dart';
import 'dart:math';
import 'package:ros_flutter_gui_app/display/pose.dart';

class MainFlame extends FlameGame {
  late MapComponent _displayMap;
  late GridComponent _displayGrid;
  final RosChannel? rosChannel;
  final ThemeProvider? themeProvider;
  late PoseComponent _displayRobot;

  List<NavPoint> offLineNavPoints = [];

  // 新的Flame组件
  late LaserComponent _laserComponent;
  late PathComponent _tracePathComponent;
  late PointCloudComponent _pointCloudComponent;
  late PathComponent _globalPathComponent;
  late PathComponent _localPathComponent;
  late CostMapComponent _globalCostMapComponent;
  late CostMapComponent _localCostMapComponent;
  
  // 拓扑图层组件
  late TopologyLine _topologyLineComponent;
  late List<PoseComponent> _wayPointComponents;
  
  // 机器人轮廓组件
  late custom.PolygonComponent _robotFootprintComponent;
  
  // 全局状态引用
  late GlobalState globalState;
  
  // NavPointManager 实例
  late NavPointManager navPointManager;
  
  // 添加右侧信息面板相关变量
  PoseComponent? selectedWayPoint;
  bool _showInfoPanel = false;
  Function(NavPoint?)? onNavPointTap;
  
  final double minScale = 0.01;
  final double maxScale = 10.0;

  bool isDarkMode=true;

  // 地图变换参数（使用camera.viewfinder）
  double mapScale = 1.0;
  Vector2 mapOffset = Vector2.zero();
  
  // 手势相关变量
  double _baseScale = 1.0;
  Vector2? _lastFocalPoint;

  bool isRelocMode = false;
  RobotPose relocRobotPose = RobotPose(0, 0, 0);
  
  MainFlame({
    this.rosChannel, 
    this.themeProvider,
    required GlobalState globalState,
    required NavPointManager navPointManager,
  }) {
    this.globalState = globalState;
    this.navPointManager = navPointManager;
    // 初始化主题模式
    isDarkMode = themeProvider?.themeMode == ThemeMode.dark;
  }
  
  @override
  Color backgroundColor() => isDarkMode ? const Color(0xFF1E1E1E) : Colors.white;
  
  @override
  Future<void> onLoad() async {
    super.onLoad();
    
        // 加载图层设置
       await globalState.loadLayerSettings();
    
       // 添加地图组件
      _displayMap = MapComponent(rosChannel: rosChannel);
      world.add(_displayMap);
      _displayMap.updateThemeMode(isDarkMode);

     _displayGrid = GridComponent(
      size: size,
      rosChannel: rosChannel,
    );
    _displayGrid.updateThemeMode(isDarkMode);

    _globalCostMapComponent = CostMapComponent(
      opacity: 0.3,
      isGlobal: true
    );
    
    _localCostMapComponent = CostMapComponent(
      opacity: 0.5,
      isGlobal: false
    );

    // 初始化新的Flame组件
    _laserComponent = LaserComponent(pointList: []);
    
    _pointCloudComponent = PointCloudComponent(
      pointList: [], 
      map: rosChannel!.map.value
    );
    
    _globalPathComponent = PathComponent(
      pointList: [], 
      color: Colors.blue
    );
     _globalPathComponent.priority = 149;
    
    _localPathComponent = PathComponent(
      pointList: [], 
      color: Colors.green
    );
    _localPathComponent.priority = 151;

    _tracePathComponent = PathComponent(
      pointList: [], 
      color: Colors.yellow
    );
    _tracePathComponent.priority = 150;

    // 初始化拓扑图层组件
    _topologyLineComponent = TopologyLine(
      points: [],
      routes: [],
      rosChannel: rosChannel, // 传入rosChannel以获取地图数据
    );
    
    // 初始化机器人轮廓组件
    _robotFootprintComponent = custom.PolygonComponent(
      pointList: [],
      color: Colors.green.withAlpha(50),
      enableWaterDropAnimation: false, // 启用水波纹动画
    );
    _robotFootprintComponent.priority = 1001;
    
    // 立即添加到world中，确保动画系统能工作
    world.add(_robotFootprintComponent);
    
    //机器人位置 - 放在最上层
    _displayRobot = PoseComponent(
      PoseComponentSize: globalSetting.robotSize,
      color: Colors.blue,
      count: 2,
      isEditMode: false,
      onPoseChanged: (RobotPose pose) {
       relocRobotPose=pose;
      },
      rosChannel: rosChannel,
      poseType: PoseType.robot,
    );
    _displayRobot.priority = 1002;
    world.add(_displayRobot);

    _wayPointComponents = [];
    
    // 监听ROS数据更新
    _setupRosListeners();
    
    // 设置图层状态监听
    _setupLayerListeners();
    
    // 根据初始图层状态添加组件
    _initializeLayerComponents();

  }
 
 

  RobotPose getRelocRobotPose(){
    return relocRobotPose;
  }
  
  void setRelocMode(bool isReloc){
    _displayRobot.setEditMode(isReloc);
    isRelocMode=isReloc;
    if(!isReloc){
      camera..viewfinder..stop();
    }
  }

      // 加载导航点
  Future<void> _loadOfflineNavPoints() async {
    // 使用传入的 NavPointManager 实例
    offLineNavPoints = await navPointManager.loadNavPoints();
    
    // 如果地图已经加载，立即更新拓扑图层
    if (rosChannel?.map_.value != null) {
      _updateTopologyLayers();
    }
  }
  
  // 重新加载导航点和地图数据
  Future<void> reloadNavPointsAndMap() async {
    print('重新加载导航点和地图数据...');
    
    // 重新加载离线导航点
    await _loadOfflineNavPoints();
    
    // 如果地图已经加载，重新更新拓扑图层
    if (rosChannel?.map_.value != null) {
      _updateTopologyLayers();
    }
    
    print('导航点和地图数据重新加载完成');
  }

  void _setupRosListeners() {
    rosChannel!.robotPoseMap.addListener(() {
      // 使用新的updatePose方法更新机器人位置
      if(isRelocMode) return;
      _displayRobot.updatePose(rosChannel!.robotPoseMap.value);
      relocRobotPose=rosChannel!.robotPoseMap.value;
    });
    
    // 监听机器人轮廓数据
    rosChannel!.robotFootprint.addListener(() {
      final footprintPoints = rosChannel!.robotFootprint.value;
        _robotFootprintComponent.updatePointList(footprintPoints);
      
    });
    
    // 监听激光雷达数据
    rosChannel!.laserPointData.addListener(() {
      if(rosChannel!.map_.value == null || rosChannel!.map_.value.mapConfig.resolution <= 0 || rosChannel!.map_.value.height() <= 0) return;
      final laserPoints = rosChannel!.laserPointData.value;
      var robotPose = laserPoints.robotPose;
      if(isRelocMode){
        robotPose=relocRobotPose;
      }
      List<Vector2> vector2Points =[];
      for (int i = 0; i < laserPoints.laserPoseBaseLink.length; i++) {
        final laserPointMap=absoluteSum(robotPose, RobotPose(laserPoints.laserPoseBaseLink[i].x, laserPoints.laserPoseBaseLink[i].y, 0));
        vm.Vector2 laserPointScene=rosChannel!.map_.value.xy2idx(vm.Vector2(laserPointMap.x, laserPointMap.y));
        vector2Points.add(Vector2(laserPointScene.x, laserPointScene.y));
      }
      _laserComponent.updateLaser(vector2Points);
    });
    
    // 监听点云数据
    rosChannel!.pointCloud2Data.addListener(() {
      final pointCloudData = rosChannel!.pointCloud2Data.value;
      _pointCloudComponent.updatePoints(pointCloudData);
    });
    
    // 监听全局路径
    rosChannel!.globalPath.addListener(() {
      final globalPathPoints = rosChannel!.globalPath.value;
      _globalPathComponent.updatePath(globalPathPoints);
    });
    
    // 监听局部路径
    rosChannel!.localPath.addListener(() {
      final localPathPoints = rosChannel!.localPath.value;
      _localPathComponent.updatePath(localPathPoints);
    });

    // 监听轨迹路径
    rosChannel!.tracePath.addListener(() {
      final tracePathPoints = rosChannel!.tracePath.value;
      _tracePathComponent.updatePath(tracePathPoints);
    });
    
    // 监听代价地图
    rosChannel!.globalCostmap.addListener(() {
      _globalCostMapComponent.updateCostMap(rosChannel!.globalCostmap.value);
    });
    
    rosChannel!.localCostmap.addListener(() {
      _localCostMapComponent.updateCostMap(rosChannel!.localCostmap.value);
    });
    
    // 监听地图数据
    rosChannel!.map_.addListener(() {
      _displayMap.updateMapData(rosChannel!.map_.value);
    });
    
    // 监听拓扑地图数据
    rosChannel!.topologyMap_.addListener(() {
      _updateTopologyLayers();
    });
    
    // 立即更新地图数据
    _displayMap.updateMapData(rosChannel!.map_.value);

    _loadOfflineNavPoints();
    
    // 立即更新拓扑图层数据
    _updateTopologyLayers();
  }

  
  // 处理缩放开始
  bool onScaleStart(Vector2 position) {
    _baseScale = mapScale;
    _lastFocalPoint = position;
    
    return true;
  }
  
  // 处理缩放更新（同时处理拖拽和缩放）
  bool onScaleUpdate(double scale, Vector2 position) {
    if (_lastFocalPoint == null) return false;
    
    // 地图缩放/移动
    final newScale = (_baseScale * scale).clamp(minScale, maxScale);
    mapScale = newScale;
    camera.viewfinder.zoom = mapScale;
    
    final focalPointDelta = position - _lastFocalPoint!;
    camera.viewfinder.position -= focalPointDelta / camera.viewfinder.zoom;
    
    _lastFocalPoint = position;
    return true;
  }
  
  // 处理缩放结束
  bool onScaleEnd() {
    _lastFocalPoint = null;
    return true;
  }
  
  // 处理滚轮缩放
  bool onScroll(double delta, Vector2 position) {
    const zoomSensitivity = 0.5;
    double zoomChange = -delta.sign * zoomSensitivity;
    final newZoom = (camera.viewfinder.zoom + zoomChange).clamp(minScale, maxScale);
    
    // 获取鼠标位置在世界坐标中的位置
    final worldPoint = camera.globalToLocal(position);
    
    // 应用缩放
    camera.viewfinder.zoom = newZoom;
    mapScale = newZoom;
    
    // 调整相机位置以保持鼠标位置不变
    final newScreenPoint = camera.localToGlobal(worldPoint);
    final offset = position - newScreenPoint;
    camera.viewfinder.position -= offset / camera.viewfinder.zoom;
    
    return true;
  }

  
  void centerOnRobot(bool isCenterOnRobot) {
    if(isCenterOnRobot){
     camera.follow(_displayRobot);
    }else{
     camera.stop();
    }
    camera.viewfinder.zoom = 6.0;
    mapScale = 6.0;
  }
  
  void zoomIn() {
    mapScale = (mapScale * 1.2).clamp(minScale, maxScale);
    camera.viewfinder.zoom = mapScale;
  }
  
  void zoomOut() {
    mapScale = (mapScale / 1.2).clamp(minScale, maxScale);
    camera.viewfinder.zoom = mapScale;
  }
  
  // 简化实现，只保留基本的点击效果

  @override
  void update(double dt) {
    super.update(dt);
    // 这里可以添加游戏逻辑更新
  }
  
  // 更新拓扑图层
  void _updateTopologyLayers() {

    //有在线导航点时，就不再显示离线导航点
    if (rosChannel?.topologyMap_.value == null) {
      print('拓扑地图数据为空，跳过更新');
      return;
    }
    
    final topologyMap = rosChannel!.topologyMap_.value;
    
    print('更新在线拓扑图层: ${topologyMap.points.length} 个点, ${topologyMap.routes.length} 条路径 离线导航点数量: ${offLineNavPoints.length}');
    
    List<NavPoint> navPoints =offLineNavPoints;
    if(topologyMap.points.isNotEmpty){
      navPoints = List<NavPoint>.from(topologyMap.points);
      print('使用在线拓扑点位');
    }

    // 更新拓扑线组件数据，但不直接添加到world
    _topologyLineComponent.removeFromParent();
    _topologyLineComponent = TopologyLine(
      points: topologyMap.points,
      routes: topologyMap.routes,
      rosChannel: rosChannel, // 传递rosChannel参数
    );
    
    // 清除旧的路径点组件
    for (final waypoint in _wayPointComponents) {
      waypoint.removeFromParent();
    }
    _wayPointComponents.clear();
    
    // 创建新的路径点组件
    for (final point in navPoints) {
      final waypoint = PoseComponent(
        PoseComponentSize: globalSetting.robotSize,
        color: Colors.green,
        count: 2,
        isEditMode: false,
        direction: point.theta,
        navPoint: point,
        rosChannel: rosChannel,
        poseType: PoseType.waypoint,
      );

      // 设置路径点位置（使用地图索引坐标）
      waypoint.updatePose(RobotPose(point.x, point.y, point.theta));
      // print('waypoint: ${waypoint.position} point: $point');
      _wayPointComponents.add(waypoint);
      
    }
    
    // 根据当前图层状态决定是否添加到world
    if (globalState.isLayerVisible('showTopology')) {
      // 添加拓扑线组件
      if (!world.contains(_topologyLineComponent)) {
        world.add(_topologyLineComponent);
      }
      
      // 添加所有导航点组件
      for (final waypoint in _wayPointComponents) {
        if (!world.contains(waypoint)) {
          world.add(waypoint);
        }
      }
      
      print('拓扑图层已更新，显示 ${_wayPointComponents.length} 个导航点');
    } else {
      print('拓扑图层不可见，移除所有导航点组件');
      // 当图层不可见时，移除所有导航点组件
      for (final waypoint in _wayPointComponents) {
        if (world.contains(waypoint)) {
          waypoint.removeFromParent();
        }
      }
    }
  }
  
  // 设置图层状态监听
  void _setupLayerListeners() {
    print('设置图层状态监听器...');
    
    // 定义图层组件映射
    final layerComponentMap = <String, Component>{
      'showGrid': _displayGrid,
      'showGlobalCostmap': _globalCostMapComponent,
      'showLocalCostmap': _localCostMapComponent,
      'showLaser': _laserComponent,
      'showPointCloud': _pointCloudComponent,
      'showGlobalPath': _globalPathComponent,
      'showLocalPath': _localPathComponent,
      'showTracePath': _tracePathComponent,
      'showTopology': _topologyLineComponent,
      'showRobotFootprint': _robotFootprintComponent,
    };
    
    // 为每个图层设置监听器
    for (final entry in layerComponentMap.entries) {
      final layerName = entry.key;
      final component = entry.value;
      
      // print('为图层 $layerName 设置监听器');
      
      globalState.getLayerState(layerName).addListener(() {
        final isVisible = globalState.isLayerVisible(layerName);
        // print('图层 $layerName 状态变化: $isVisible');
        
        if (isVisible) {
          if (!world.contains(component)) {
            // print('添加组件 $layerName 到world');
            world.add(component);
          }
        } else {
          if (world.contains(component)) {
            // print('从world移除组件 $layerName');
            // 添加安全检查，确保组件仍然有效
            if (component.isMounted) {
              component.removeFromParent();
            }
          }
        }
      });
    }
    
    // 特殊处理拓扑图层的路径点
    globalState.getLayerState('showTopology').addListener(() {
      final isVisible = globalState.isLayerVisible('showTopology');
      print('拓扑图层状态变化: $isVisible');
      
      if (isVisible) {
        if (!world.contains(_topologyLineComponent)) {
          world.add(_topologyLineComponent);
        }
        for (final waypoint in _wayPointComponents) {
          if (!world.contains(waypoint)) {
            world.add(waypoint);
          }
        }
      } else {
        if (world.contains(_topologyLineComponent)) {
          if (_topologyLineComponent.isMounted) {
            _topologyLineComponent.removeFromParent();
          }
        }
        for (final waypoint in _wayPointComponents) {
          if (world.contains(waypoint)) {
            if (waypoint.isMounted) {
              waypoint.removeFromParent();
            }
          }
        }
      }
    });
    
    // 根据初始状态设置组件显示
    _updateLayerVisibility();
  }
  
  // 根据图层状态更新组件可见性
  void _updateLayerVisibility() {
    // 定义图层组件映射
    final layerComponentMap = <String, Component>{
      'showGrid': _displayGrid,
      'showGlobalCostmap': _globalCostMapComponent,
      'showLocalCostmap': _localCostMapComponent,
      'showLaser': _laserComponent,
      'showPointCloud': _pointCloudComponent,
      'showGlobalPath': _globalPathComponent,
      'showLocalPath': _localPathComponent,
      'showTracePath': _tracePathComponent,
      'showTopology': _topologyLineComponent,
      'showRobotFootprint': _robotFootprintComponent,
    };
    
    // 检查每个图层的可见性
    for (final entry in layerComponentMap.entries) {
      final layerName = entry.key;
      final component = entry.value;
      
      if (!globalState.isLayerVisible(layerName) && world.contains(component)) {
        component.removeFromParent();
      }
    }
    
    // 特殊处理拓扑图层的路径点
    if (!globalState.isLayerVisible('showTopology')) {
      for (final waypoint in _wayPointComponents) {
        if (world.contains(waypoint)) {
          waypoint.removeFromParent();
        }
      }
    }
  }

  // 根据初始图层状态添加组件
  void _initializeLayerComponents() {
    print('初始化图层组件...');
    
    final layerComponentMap = <String, Component>{
      'showGrid': _displayGrid,
      'showGlobalCostmap': _globalCostMapComponent,
      'showLocalCostmap': _localCostMapComponent,
      'showLaser': _laserComponent,
      'showPointCloud': _pointCloudComponent,
      'showGlobalPath': _globalPathComponent,
      'showLocalPath': _localPathComponent,
      'showTracePath': _tracePathComponent,
      'showTopology': _topologyLineComponent,
      'showRobotFootprint': _robotFootprintComponent,
    };

    for (final entry in layerComponentMap.entries) {
      final layerName = entry.key;
      final component = entry.value;
      final isVisible = globalState.isLayerVisible(layerName);
      
      print('图层 $layerName 初始状态: $isVisible');

      if (isVisible) {
        if (!world.contains(component)) {
          print('初始添加组件 $layerName 到world');
          world.add(component);
        }
      } else {
        if (world.contains(component)) {
          print('初始从world移除组件 $layerName');
          if (component.isMounted) {
            component.removeFromParent();
          }
        }
      }
    }

    // 特殊处理拓扑图层的路径点
    final topologyVisible = globalState.isLayerVisible('showTopology');
    print('拓扑图层初始状态: $topologyVisible');
    
    if (topologyVisible) {
      if (!world.contains(_topologyLineComponent)) {
        world.add(_topologyLineComponent);
      }
      for (final waypoint in _wayPointComponents) {
        if (!world.contains(waypoint)) {
          world.add(waypoint);
        }
      }
    } else {
      if (world.contains(_topologyLineComponent)) {
        _topologyLineComponent.removeFromParent();
      }
      for (final waypoint in _wayPointComponents) {
        if (world.contains(waypoint)) {
          waypoint.removeFromParent();
        }
      }
    }
  }
  

  NavPoint? getSelectedWayPointInfo() {
    final wayPoint = selectedWayPoint;
    if (wayPoint == null) return null;
    var x = wayPoint.position.x;
    var y = wayPoint.position.y;
    var direction = -wayPoint.direction;
    double mapx = 0;
    double mapy = 0;
    if(rosChannel?.map_.value != null){
      vm.Vector2 mapPose = rosChannel!.map_.value.idx2xy(vm.Vector2(x, y));
      mapx = mapPose.x;
      mapy = mapPose.y;
    }
    var pointInfo = wayPoint.getPointInfo();
    pointInfo!.x = mapx;
    pointInfo!.y = mapy;
    pointInfo!.theta = direction;
    return pointInfo;
  }


  
  // 获取信息面板显示状态
  bool get showInfoPanel => _showInfoPanel;
  
  // 隐藏信息面板
  void hideInfoPanel() {
    _showInfoPanel = false;
  }
  
  // 检测点击的waypoint
  void onTap(Offset position) {

    // 使用camera.globalToLocal转换坐标，参考map_edit_flame.dart的实现
    final worldPoint = camera.globalToLocal(Vector2(position.dx, position.dy));

  
    for (int i = 0; i < _wayPointComponents.length; i++) {
      final waypoint = _wayPointComponents[i];
      
      if (waypoint.containsPoint(worldPoint)) {
        if (waypoint.navPoint != null) {
          selectedWayPoint = waypoint;
          _showInfoPanel = true;
          onNavPointTap?.call(getSelectedWayPointInfo()!);
          print('选中的导航点: ${waypoint.navPoint!.name}');
        } 
       return;
      }
    }
  
   _showInfoPanel=false;
    onNavPointTap?.call(null);
  }
  
  // 设置机器人编辑模式
  void setRobotEditMode(bool edit) {
    _displayRobot.setEditMode(edit);
  }
  
 
  

}