# ROS Flutter Gui APP

基于 C/S 架构的 ROS 人机交互软件，借助Flutter的跨端特性，可以在 Web Android/IOS Mac 等多平台运行(主要倾向于移动APP侧)

注：本仓库为了提高渲染及通信效率从v2.0版本后改为自定义后端的C/S架构,不再依赖ros bridge server，ros bridge server版本请见 [v1.2.5](https://github.com/chengyangkj/ROS_Flutter_Gui_App/tree/v1.2.5)

本仓库包含两部分内容:
- backend: 后端，部署在机器人侧，与ROS2通信，提供 Http 及 Websocket 接口给 APP，目前仅支持ROS2(ROS1接口已预留欢迎提交MR)
- app: 基于Flutter的客户端及前端，与后端通信，做显示



**详细设计、接口与目录说明请阅：**

| 文档 | 内容 |
| --- | --- |
| [backend/README.md](backend/README.md) | 后端编译与部署、进程结构、数据目录、`config.yaml`、HTTP/ROS 接口摘要 |
| [app/README.md](app/README.md) | 前端依赖与构建、与后端连接方式、功能与设置、使用流程 |


---

## 1. 编译部署

### 1.1 后端编译

```
git clone https://github.com/chengyangkj/ROS_Flutter_Gui_App
cd backend
./build.sh
```
### 1.2 前端编译

```
cd app
flutter pub get
flutter build
```

## 2. 二进制部署

从 Release 下载前端及后端的二进制

## 2. 使用

### 2.1 后端使用

```
cd backend/build/install/bin
sh ./start.sh
```

### 2.2 前端使用

启动2.1的后端后，默认会在 http://127.0.0.1:7684 生成web服务，浏览器访问即可，也能在 Release 界面下载android版本的APP进行连接 

---

## 3. 仓库结构（索引）

| 路径 | 说明 |
| --- | --- |
| `protocol/` | `.proto`，根 `build.sh` 生成 **Dart** 与后端共用约定 |
| `backend/` | C++ 后端，CMake 子工程 |
| `app/` | Flutter 工程 |
| `build.sh` | 根目录一键：protobuf → backend → `flutter build web` |
