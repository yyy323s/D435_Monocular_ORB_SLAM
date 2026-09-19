# 架构与数据流

```
D435 IR1 (Y8, 640x480) -> mono_d435 -> System::TrackMonocular
                                         |        |
                              Tracking (调用线程)  +-> LocalMapping 线程
                                         |        +-> LoopClosing   线程
                                         +-> Viewer 线程（可选）
```

`apps/mono_d435.cpp` 在 RealSense 帧仍有效时按真实 stride 深拷贝灰度图，再把单调时间戳送入 `System::TrackMonocular`。这避免了回调返回后引用相机缓冲区的生命周期问题。

`Tracking` 负责 ORB 提取、初始化、运动模型跟踪、重定位和关键帧决策；它向 `LocalMapping` 传递关键帧。`LocalMapping` 进行三角化、局部 BA 和地图点筛除。`LoopClosing` 用 DBoW2 查询候选、做 Sim3 校正和全局 BA。Viewer 仅可视化，不参与算法正确性。

关闭顺序由 `System::Shutdown()` 统一执行：停止局部建图、回环和 Viewer，请求/等待全局 BA，随后 join 线程。应用在此之后才导出关键帧轨迹，避免未完成优化时写出不一致结果。

系统使用 D435 的单个红外相机，故只能恢复相对尺度；若需要米制尺度，应改为双目（IR1+IR2）或使用带 IMU 的设备并接入相应 ORB-SLAM3 模式。
