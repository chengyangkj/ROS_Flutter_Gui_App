import 'package:flutter/cupertino.dart';

enum Mode {
  noraml,
  reloc, //重定位模式
  addNavPoint, //添加导航点模式
  robotFixedCenter, //机器人固定屏幕中心模式
}

class GlobalState extends ChangeNotifier {
   ValueNotifier<Mode> mode = ValueNotifier(Mode.noraml);
   ValueNotifier<bool> isManualCtrl = ValueNotifier(false);
}
