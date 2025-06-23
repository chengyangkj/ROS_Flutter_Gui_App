# 多边形绘制功能使用指南

## 概述

本项目提供了完整的多边形绘制功能，包括基本多边形绘制、机器人轮廓显示、以及各种高级绘制效果。

## 主要组件

### 1. DisplayPolygon 类

这是主要的多边形绘制类，支持多种绘制选项：

```dart
DisplayPolygon({
  required List<Offset> pointList,  // 多边形顶点列表
  required Color color,             // 多边形颜色
  bool fill = false,                // 是否填充
  double strokeWidth = 2.0,         // 边框宽度
  bool showVertices = false,        // 是否显示顶点
  Color vertexColor = Colors.blue,  // 顶点颜色
  double vertexRadius = 3.0,        // 顶点半径
})
```

#### 使用示例：

```dart
CustomPaint(
  painter: DisplayPolygon(
    pointList: [
      Offset(100, 100),
      Offset(200, 100),
      Offset(150, 200),
    ],
    color: Colors.blue,
    fill: true,
    showVertices: true,
  ),
)
```

### 2. PolygonDrawer 工具类

提供静态方法用于快速绘制各种样式的多边形：

#### 简单多边形
```dart
PolygonDrawer.drawSimplePolygon(
  canvas, 
  points, 
  Colors.red, 
  fill: true
);
```

#### 渐变多边形
```dart
PolygonDrawer.drawGradientPolygon(
  canvas,
  points,
  [Colors.purple, Colors.pink, Colors.orange]
);
```

#### 虚线边框多边形
```dart
PolygonDrawer.drawDashedPolygon(
  canvas,
  points,
  Colors.orange,
  dashLength: 8.0
);
```

### 3. 机器人轮廓绘制

在 `DisplayRobot` 类中，`drawRobotFootprint` 方法专门用于绘制机器人的多边形轮廓：

```dart
void drawRobotFootprint(Canvas canvas, Size size) {
  // 检查点列表有效性
  if (robotFootprint.isEmpty || robotFootprint.length < 3) {
    return;
  }
  
  // 创建路径并绘制
  Path path = Path();
  path.moveTo(robotFootprint[0].dx, robotFootprint[0].dy);
  
  for (int i = 1; i < robotFootprint.length; i++) {
    path.lineTo(robotFootprint[i].dx, robotFootprint[i].dy);
  }
  path.close();
  
  // 绘制填充和边框
  // ...
}
```

## 数据来源

机器人轮廓数据通过 ROS 话题接收：

- **话题名称**: `/local_costmap/published_footprint`
- **消息类型**: `geometry_msgs/PolygonStamped`
- **处理函数**: `robotFootprintCallback`

数据流程：
1. ROS 话题接收 PolygonStamped 消息
2. 解析多边形顶点坐标
3. 坐标系转换（从机器人坐标系到地图坐标系）
4. 更新 `robotFootprint` 列表
5. UI 自动重绘显示新的轮廓

## 配置选项

在 `global/setting.dart` 中可以配置相关话题：

```dart
String get robotFootprintTopic {
  return prefs.getString("robotFootprintTopic") ?? 
         "/local_costmap/published_footprint";
}
```

## 演示页面

运行 `PolygonDemoPage` 可以查看各种多边形绘制效果的演示：

- 基本多边形绘制
- 带顶点的多边形
- 虚线边框多边形
- 渐变填充多边形
- 交互式多边形编辑

## 最佳实践

1. **数据验证**: 始终检查点列表是否为空或点数不足
2. **性能优化**: 对于频繁更新的多边形，使用 `RepaintBoundary` 包装
3. **坐标系**: 确保正确进行坐标系转换
4. **错误处理**: 添加适当的错误处理和日志记录

## 故障排除

### 多边形不显示
- 检查点列表是否为空
- 确认坐标值是否在可见范围内
- 验证坐标系转换是否正确

### 性能问题
- 减少不必要的重绘
- 使用 `shouldRepaint` 优化重绘逻辑
- 考虑使用 `RepaintBoundary`

### 坐标错误
- 检查 ROS 话题数据格式
- 验证坐标系转换矩阵
- 确认地图分辨率设置

## 扩展功能

可以基于现有代码扩展更多功能：

1. **动画效果**: 添加多边形动画
2. **交互编辑**: 支持拖拽编辑顶点
3. **样式主题**: 支持多种视觉主题
4. **数据导出**: 支持导出多边形数据 