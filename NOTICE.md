# Source notice

本项目以 UZ-SLAMLab 的 ORB-SLAM3 为基础，导入的上游快照对应提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`。上游代码、词典和随附第三方代码仍受其各自许可证约束；项目整体按根目录 `LICENSE` 中的 GPL-3.0 分发。

本地改动包括：面向 CMake 的独立构建、D435 IR1 单目入口与配置、运行时实际内参写入、帧缓冲深拷贝、受控 SIGINT 退出，以及 System/LoopClosing 的线程等待和关闭路径修正。第三方目录中的构建输出路径与生成的 g2o 配置头也做了非算法性适配。

原型实现保留在 `prototype_m1/`，不属于当前构建目标。
