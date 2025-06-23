import 'dart:math';
import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_svg/flutter_svg.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/basic/occupancy_map.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';

class DisplayRobot extends StatefulWidget {
  late double size;
  late Color color = Colors.amberAccent;
  double direction = deg2rad(45); //方向默认0度 车头朝向右侧
  
  DisplayRobot({
    required this.size,
    required this.color
  });

  @override
  _DisplayRobotState createState() => _DisplayRobotState();
}

class _DisplayRobotState extends State<DisplayRobot> {
  @override
  Widget build(BuildContext context) {
    return Container(
      width: widget.size,
      height: widget.size,
      child: _buildRobotSvg(),
    );
  }

  Widget _buildRobotSvg() {
    return Center(
      child: Transform.rotate(
        angle: widget.direction,
        child: SvgPicture.asset(
          'assets/icons/robot/robot.svg',
          width: widget.size * 0.4, // 机器人图标占容器的60%
          height: widget.size * 0.4,
          // colorFilter: ColorFilter.mode(
          //   widget.color,
          //   BlendMode.srcIn,
          // ),
        ),
      ),
    );
  }
}
