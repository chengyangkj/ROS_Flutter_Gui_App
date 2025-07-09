import 'package:flutter/cupertino.dart';

enum Mode {
  normal,
  reloc, //重定位模式
  addNavPoint, //添加导航点模式
  robotFixedCenter, //机器人固定屏幕中心模式
}

class GlobalState extends ChangeNotifier {
   ValueNotifier<Mode> mode = ValueNotifier(Mode.normal);
   ValueNotifier<bool> isManualCtrl = ValueNotifier(false);
   
   // 图层开关状态
   ValueNotifier<bool> showGlobalCostmap = ValueNotifier(false);
   ValueNotifier<bool> showLocalCostmap = ValueNotifier(false);
   ValueNotifier<bool> showLaser = ValueNotifier(false);
   ValueNotifier<bool> showPointCloud = ValueNotifier(false);
}
