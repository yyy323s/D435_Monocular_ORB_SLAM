# D435 Monocular ORB-SLAM3

这是一个基于 ORB-SLAM3 单目架构的 Intel RealSense D435 实时 SLAM 工程。根目录保留完整的 ORB-SLAM3 `include/`、`src/` 和第三方依赖；D435 不带 IMU，因此运行模式固定为 `MONOCULAR`，图像源为左红外相机（IR1）。

相机的 IR1 是全局快门，适合运动中的特征跟踪；单目系统没有绝对尺度，轨迹坐标单位是任意尺度而非米。

## 构建

仅保留 `cmake-build-debug/` 作为构建目录：

```bash
cmake -S . -B cmake-build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build cmake-build-debug -j"$(nproc)"
```

需要发布构建时，仍使用这个目录并把 `CMAKE_BUILD_TYPE` 改为 `Release` 后重新构建。

构建时会从 `Vocabulary/ORBvoc.txt.tar.gz` 自动解压词典到构建目录。所需系统依赖为 C++17、OpenCV、Eigen3、Pangolin、Boost.Serialization、OpenSSL 和 librealsense2。

## 运行

连接 D435 后：

```bash
./cmake-build-debug/bin/mono_d435
```

无显示器或远程终端：

```bash
./cmake-build-debug/bin/mono_d435 --headless --trajectory run.tum
```

首次启动时保持相机静止约一秒；随后对有纹理场景做缓慢、连续的横向平移和小角度转动，避免纯旋转或快速抖动。`Ctrl-C` 会请求有序关闭并输出关键帧 TUM 轨迹。常用选项：

```text
--serial <S/N>          多设备时选择相机
--settings <yaml>       覆盖配置文件
--vocabulary <path>     覆盖 ORB 词典
--trajectory <文件>     覆盖关键帧轨迹输出位置
--pointcloud <文件>     覆盖稀疏 PLY 点云输出位置
--warmup-frames <N>     曝光预热帧数
--max-frames <N>        受控测试后退出
--no-save               不导出轨迹
--no-pointcloud         不导出稀疏点云
--headless              关闭 Pangolin Viewer
```

程序会读取启动后实际协商的 IR1 内参，并在 `cmake-build-debug/runtime/` 写入一份有效配置再创建 SLAM 系统，因此不会因配置中的模板内参和运行分辨率不一致而运行。正常退出后，关键帧 TUM 轨迹默认保存到项目的 `output/KeyFrameTrajectory.txt`，活动地图中的有效稀疏地图点默认保存到 `output/MapPoints.ply`；可分别用 `--trajectory <文件>` 和 `--pointcloud <文件>` 覆盖位置。

## 目录

- `apps/mono_d435.cpp`：RealSense 采集、命令行、运行时标定写入与有序退出。
- `config/D435_Monocular.yaml`：D435 IR1 与 ORB-SLAM3 参数模板。
- `include/`、`src/`：完整 ORB-SLAM3 系统、跟踪、局部建图、回环和 Viewer。
- `Thirdparty/`：DBoW2、g2o、Sophus。
- `docs/ARCHITECTURE.md`：线程和数据流说明。
- `prototype_m1/`：早期简化原型归档，不参与当前构建。

本工程遵循 [GPL-3.0](LICENSE)；来源与本地改动见 `NOTICE.md`。
