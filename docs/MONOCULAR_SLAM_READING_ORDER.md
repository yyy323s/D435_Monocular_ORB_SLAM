# 纯单目 SLAM 代码阅读顺序

本文面向本仓库的 D435 单路红外输入程序，按**自底向上**的依赖关系给出阅读顺序。目标是先理解被调用的基础对象，再看 `Tracking`、`LocalMapping`、`LoopClosing` 和 `System` 等总控模块。

## 0. 先确定本项目的实际范围

本项目运行的是纯视觉 `MONOCULAR`，而不是双目、RGB-D 或 `IMU_MONOCULAR`：

- [`apps/mono_d435.cpp`](../apps/mono_d435.cpp) 启用一条 `RS2_STREAM_INFRARED`、`Y8` 图像流，并以 `System::MONOCULAR` 创建系统。
- [`config/D435_Monocular.yaml`](../config/D435_Monocular.yaml) 使用 `Camera.type: "PinHole"`。
- 配置开启了回环检测（`loopClosing: 1`）。

两个后果很重要：

1. 单目初始化没有真实尺度；初始化后会以中值深度归一化地图。
2. 回环和地图合并必须保留 Sim3 的尺度自由优化，不能把它当作“双目功能”跳过。

每个类建议采用同一种看法：**先扫 `.h` 中的数据成员、所有权和锁；再按本文给出的顺序读 `.cc`。** 序列化和 Viewer 相关内容都可以放到最后。

```text
配置与相机模型
  → ORB 特征与词袋
  → Frame 与两视图几何
  → MapPoint / KeyFrame / Map / Atlas
  → 匹配与视觉优化
  → PnP、Sim3、候选数据库
  → Tracking
  → LocalMapping
  → LoopClosing
  → System
  → mono_d435.cpp
```

## 1. 配置、相机和基础转换

### 1.1 `Settings`

文件：[`include/Settings.h`](../include/Settings.h)、[`src/Settings.cc`](../src/Settings.cc)

按以下顺序读：

```text
readParameter<T>
→ readCamera1
→ readImageInfo
→ readORB
→ readOtherParameters
→ readViewer（可略读）
→ readLoadAndSave（可略读）
→ Settings::Settings
```

跳过 `readCamera2`、`readIMU`、`readRGBD`、`precomputeRectificationMaps`。入口程序会依据 D435 实际激活的 IR profile 写出有效配置，因此 YAML 内的内参是备用快照。

### 1.2 `GeometricCamera` 与 `Pinhole`

文件：[`include/CameraModels/GeometricCamera.h`](../include/CameraModels/GeometricCamera.h)、[`include/CameraModels/Pinhole.h`](../include/CameraModels/Pinhole.h)、[`src/CameraModels/Pinhole.cpp`](../src/CameraModels/Pinhole.cpp)

先把 `GeometricCamera` 当作投影接口；然后按下列顺序读 `Pinhole`：

```text
构造函数：参数 [fx, fy, cx, cy]
→ toK / toK_
→ project 的各重载
→ unproject / unprojectEig
→ projectJac
→ uncertainty2
→ epipolarConstrain
→ ReconstructWithTwoViews
→ IsEqual
```

`project/projectJac` 服务匹配与优化；`epipolarConstrain` 服务关键帧三角化匹配；`ReconstructWithTwoViews` 是单目初始化的相机模型入口。`Pinhole::matchAndtriangulate` 直接返回 `false`，不必深究。

跳过 `KannalaBrandt8.*`。

### 1.3 `Converter`

文件：[`include/Converter.h`](../include/Converter.h)、[`src/Converter.cc`](../src/Converter.cc)

它没有核心算法，只负责 OpenCV、Eigen、Sophus、g2o 的类型转换。关注：

```text
toDescriptorVector
→ toMatrix3f / toVector3f
→ toCvMat 的 Eigen、SE3、Sim3 重载
→ toSophus(g2o::Sim3)
→ toQuaternion（轨迹输出）
```

## 2. ORB 特征与 BoW

### 2.1 `ORBextractor`

文件：[`include/ORBextractor.h`](../include/ORBextractor.h)、[`src/ORBextractor.cc`](../src/ORBextractor.cc)

```text
ORBextractor::ORBextractor
→ ComputePyramid
→ ExtractorNode::DivideNode
→ compareNodes
→ DistributeOctTree
→ IC_Angle
→ computeOrientation
→ ComputeKeyPointsOctTree
→ computeOrbDescriptor
→ computeDescriptors
→ operator()
```

- `ComputePyramid`：建立多尺度图像。
- `DistributeOctTree`：从 FAST 候选中选取空间分布均匀的点。
- `IC_Angle`：灰度质心法计算方向。
- `computeOrbDescriptor`：旋转 BRIEF 的二值描述子。
- `operator()`：完整入口，最后看。

跳过 `ComputeKeyPointsOld`，当前入口使用 OctTree 版本。

### 2.2 `ORBVocabulary` 与 DBoW2

先看 [`include/ORBVocabulary.h`](../include/ORBVocabulary.h)，它只是 `TemplatedVocabulary<FORB>` 的别名。

只读 DBoW2 的运行时部分：

```text
FORB::distance
→ BowVector::addWeight / normalize
→ FeatureVector::addFeature
→ TemplatedVocabulary::transform（单描述子）
→ transform（描述子集合，生成 BowVector 与 FeatureVector）
→ L1Scoring::score
→ loadFromTextFile
```

对应目录：[`Thirdparty/DBoW2/DBoW2`](../Thirdparty/DBoW2/DBoW2)。不要阅读词典训练、聚类和 HKmeans；程序只加载已有 ORB 词典。

## 3. 一帧图像：`Frame`

文件：[`include/Frame.h`](../include/Frame.h)、[`src/Frame.cc`](../src/Frame.cc)

先在头文件认清这些数据：

```text
mvKeys / mvKeysUn / mDescriptors
mvpMapPoints / mvbOutlier
mBowVec / mFeatVec
mpCamera
mTcw / mRcw / mtcw / mRwc / mOw
mGrid
```

类内顺序：

```text
ExtractORB
→ UndistortKeyPoints
→ ComputeImageBounds
→ PosInGrid
→ AssignFeaturesToGrid
→ 单目 Frame 构造函数
→ GetFeaturesInArea
→ UpdatePoseMatrices
→ SetPose
→ isInFrustum
→ ComputeBoW
→ 拷贝构造函数
```

单目构造函数串起：

```text
ORB 提取 → 去畸变 → 初始化无深度数组 → 图像边界 → 特征网格化
```

重点：

- `GetFeaturesInArea` 是各种匹配器查询候选特征的基础。
- `isInFrustum` 会把地图点的预计像素位置、尺度层级等结果写入 `MapPoint`，为投影匹配做准备。
- 当前 YAML 的畸变参数为零，因此 `UndistortKeyPoints` 通常只复制 `mvKeys`。

跳过双目/RGB-D 构造、`ComputeStereoMatches`、`ComputeStereoFromRGBD`、`UnprojectStereo`、鱼眼双目函数、所有 IMU 函数，以及当前没有调用点的 `ProjectPointDistort`、`inRefCoordinates`。

## 4. 单目两视图初始化

### 4.1 `GeometricTools`

文件：[`include/GeometricTools.h`](../include/GeometricTools.h)、[`src/GeometricTools.cc`](../src/GeometricTools.cc)

只看：

```text
Triangulate
```

`ComputeF12` 当前没有调用点，可跳过。

### 4.2 `TwoViewReconstruction`

文件：[`include/TwoViewReconstruction.h`](../include/TwoViewReconstruction.h)、[`src/TwoViewReconstruction.cc`](../src/TwoViewReconstruction.cc)

```text
构造函数
→ Normalize
→ ComputeH21
→ ComputeF21
→ CheckHomography
→ CheckFundamental
→ FindHomography
→ FindFundamental
→ GeometricTools::Triangulate
→ DecomposeE
→ CheckRT
→ ReconstructF
→ ReconstructH
→ Reconstruct
```

完整逻辑是：归一化匹配点，RANSAC 同时估计 H/F，按得分选择模型，分解为候选 R/t，再以三角化、正深度、重投影和视差检验选择唯一解。

## 5. 地图数据结构

`KeyFrame` 与 `MapPoint` 循环引用：前者持有地图点，后者记录观测它的关键帧。因此先快速浏览 `KeyFrame.h` 的字段，再读完整 `MapPoint`，最后回来读 `KeyFrame` 实现。

### 5.1 `MapPoint`

文件：[`include/MapPoint.h`](../include/MapPoint.h)、[`src/MapPoint.cc`](../src/MapPoint.cc)

阅读前先看 `ORBmatcher::DescriptorDistance`。

```text
MapPoint(Pos, KeyFrame*, Map*)
→ SetWorldPos / GetWorldPos / GetNormal
→ AddObservation
→ GetObservations / Observations
→ GetIndexInKeyFrame / IsInKeyFrame
→ ComputeDistinctiveDescriptors / GetDescriptor
→ UpdateNormalAndDepth
→ GetMinDistanceInvariance / GetMaxDistanceInvariance
→ PredictScale(KeyFrame / Frame)
→ IncreaseVisible / IncreaseFound / GetFoundRatio
→ EraseObservation
→ Replace
→ SetBadFlag / isBad
→ GetMap / UpdateMap
```

关键关系：`MapPoint ↔ KeyFrame` 是双向观测；每个点持有代表描述子，并由法向、距离范围和预测尺度层级支持投影匹配。

跳过逆深度构造函数与从 `Frame` 创建临时深度点的构造函数。

### 5.2 `KeyFrame`

文件：[`include/KeyFrame.h`](../include/KeyFrame.h)、[`src/KeyFrame.cc`](../src/KeyFrame.cc)

```text
KeyFrame(Frame&, Map*, KeyFrameDatabase*)
→ SetPose 和纯视觉 pose getter
→ GetFeaturesInArea / IsInImage
→ AddMapPoint / EraseMapPointMatch / ReplaceMapPointMatch
→ GetMapPointMatches / GetMapPoint / TrackedMapPoints
→ ComputeBoW
→ AddConnection
→ UpdateBestCovisibles
→ 共视关键帧 getter
→ UpdateConnections
→ AddChild / EraseChild / ChangeParent / GetParent
→ AddLoopEdge / AddMergeEdge
→ ComputeSceneMedianDepth
→ SetNotErase / SetErase / SetBadFlag
→ GetMap / UpdateMap
```

必须理解三种图：

```text
共视图：共享 MapPoint 数作为边权
生成树：关键帧淘汰、回环矫正和 GBA 传播
Loop/Merge 边：回环和多地图合并约束
```

跳过 IMU、速度、右相机和 `UnprojectStereo`。

### 5.3 `Map`

文件：[`include/Map.h`](../include/Map.h)、[`src/Map.cc`](../src/Map.cc)

```text
构造函数
→ AddKeyFrame / AddMapPoint
→ 获取全部对象和数量接口
→ GetId / 初始化 KF / origin KF
→ EraseMapPoint / EraseKeyFrame
→ SetReferenceMapPoints / GetReferenceMapPoints
→ GetMapChangeIndex / IncreaseChangeIndex
→ InformNewBigChange
→ SetCurrentMap / SetStoredMap / IsInUse
→ SetBad / IsBad
→ clear
```

跳过惯性初始化、尺度旋转和序列化。

### 5.4 `Atlas`

文件：[`include/Atlas.h`](../include/Atlas.h)、[`src/Atlas.cc`](../src/Atlas.cc)

```text
构造函数
→ AddCamera
→ CreateNewMap
→ GetCurrentMap
→ AddKeyFrame / AddMapPoint
→ 当前地图查询代理
→ GetAllMaps / CountMaps
→ ChangeMap
→ SetMapBad
→ RemoveBadMaps
→ clearMap / clearAtlas
```

纯单目也不能跳过 Atlas：跟踪完全丢失时可能创建新 Map，之后由回环模块对齐并合并。

## 6. 特征匹配：`ORBmatcher`

文件：[`include/ORBmatcher.h`](../include/ORBmatcher.h)、[`src/ORBmatcher.cc`](../src/ORBmatcher.cc)

先看基础函数：

```text
DescriptorDistance
→ ComputeThreeMaxima
→ RadiusByViewingCos
→ 构造函数
```

再按运行阶段阅读：

```text
初始化：
SearchForInitialization

正常跟踪：
SearchByBoW(KeyFrame*, Frame*)
→ SearchByProjection(CurrentFrame, LastFrame)
→ SearchByProjection(Frame&, vector<MapPoint*>)
→ SearchByProjection(Frame&, KeyFrame*)  // 重定位补匹配

局部建图：
SearchForTriangulation
→ Fuse(KeyFrame*, vector<MapPoint*>)

回环/地图合并：
SearchByBoW(KeyFrame*, KeyFrame*)
→ 两种 Sim3 SearchByProjection
→ Fuse(KeyFrame*, Sim3, ...)
```

`SearchBySim3` 当前没有调用点，可作为补充。混合函数中跳过右相机网格、右图描述子、左右观测联合处理和 stereo 三维误差块。

## 7. 纯视觉优化

### 7.1 `OptimizableTypes`

文件：[`include/OptimizableTypes.h`](../include/OptimizableTypes.h)、[`src/OptimizableTypes.cpp`](../src/OptimizableTypes.cpp)

```text
EdgeSE3ProjectXYZOnlyPose：computeError → linearizeOplus
→ EdgeSE3ProjectXYZ：computeError → linearizeOplus
→ VertexSim3Expmap：oplusImpl / map / project
→ EdgeSim3ProjectXYZ
→ EdgeInverseSim3ProjectXYZ
```

它们分别支撑当前帧位姿优化、BA 和 Sim3 回环优化。跳过两个 `ToBody` 边，以及 `G2oTypes.*` 的惯性部分。

### 7.2 `Optimizer`

文件：[`include/Optimizer.h`](../include/Optimizer.h)、[`src/Optimizer.cc`](../src/Optimizer.cc)

```text
PoseOptimization
→ BundleAdjustment
→ GlobalBundleAdjustemnt
→ LocalBundleAdjustment(KeyFrame*, Map*)
→ OptimizeSim3
→ OptimizeEssentialGraph(Map*, ...)
→ 地图合并窗口使用的 LocalBundleAdjustment 重载
```

每个优化函数都按同一结构理解：创建优化器，加入位姿顶点与地图点顶点，加入重投影边，执行优化，删除外点观测，并写回 KeyFrame/MapPoint。

跳过 `FullInertialBA`、`LocalInertialBA`、全部 `InertialOptimization`、`MergeInertialBA`、`PoseInertialOptimization*`、`OptimizeEssentialGraph4DoF`，以及 stereo/body edge 分支。

## 8. 候选检索和位姿求解

### 8.1 `KeyFrameDatabase`

文件：[`include/KeyFrameDatabase.h`](../include/KeyFrameDatabase.h)、[`src/KeyFrameDatabase.cc`](../src/KeyFrameDatabase.cc)

```text
构造函数：倒排索引
→ add / erase
→ DetectRelocalizationCandidates
→ DetectNBestCandidates
→ clearMap / clear
```

前者服务 Tracking 重定位；后者服务回环与多地图合并。旧的候选检索函数当前未使用。

### 8.2 `MLPnPsolver`

文件：[`include/MLPnPsolver.h`](../include/MLPnPsolver.h)、[`src/MLPnPsolver.cpp`](../src/MLPnPsolver.cpp)

```text
rodrigues2rot / rot2rodrigues
→ mlpnpJacs
→ mlpnp_residuals_and_jacs
→ mlpnp_gn
→ computePose
→ 构造函数
→ SetRansacParameters
→ CheckInliers
→ Refine
→ iterate
```

这是纯单目重定位的 PnP RANSAC 求解器。`mlpnpJacs` 第一次阅读只需掌握输入输出，不必手推全部临时变量。

### 8.3 `Sim3Solver`

文件：[`include/Sim3Solver.h`](../include/Sim3Solver.h)、[`src/Sim3Solver.cc`](../src/Sim3Solver.cc)

```text
FromCameraToImage
→ Project
→ ComputeCentroid
→ ComputeSim3
→ CheckInliers
→ 构造函数
→ SetRansacParameters
→ 五参数 iterate(..., bConverge)
→ GetEstimatedRotation / Translation / Scale
```

纯单目重点是 `ComputeSim3` 的自由尺度计算。四参数 `iterate`、`find` 与 `GetEstimatedTransformation` 当前没有调用点。

## 9. `Tracking`：主线程状态机

文件：[`include/Tracking.h`](../include/Tracking.h)、[`src/Tracking.cc`](../src/Tracking.cc)

先确认状态机：

```text
NO_IMAGES_YET → NOT_INITIALIZED → OK → RECENTLY_LOST → LOST
```

类内顺序：

```text
newParameterLoader
→ SetLocalMapper / SetLoopClosing / SetViewer
→ Tracking::Tracking

CreateInitialMapMonocular
→ MonocularInitialization

CheckReplacedInLastFrame
→ UpdateLastFrame（单目只看开头）
→ UpdateLocalKeyFrames
→ UpdateLocalPoints
→ UpdateLocalMap
→ SearchLocalPoints

TrackReferenceKeyFrame
→ TrackWithMotionModel
→ Relocalization

TrackLocalMap
→ NeedNewKeyFrame
→ CreateNewKeyFrame

CreateMapInAtlas
→ ResetActiveMap
→ Reset

Track
→ GrabImageMonocular
```

初始化链：

```text
SearchForInitialization
→ ReconstructWithTwoViews
→ 创建两个 KeyFrame 和初始 MapPoint
→ GlobalBundleAdjustment
→ 中值深度尺度归一化
```

`Track` 应最后阅读；它将初始化、常规跟踪、最近丢失、完全丢失、局部地图跟踪和关键帧插入串成完整状态机。

跳过 `GrabImageStereo`、`GrabImageRGBD`、`StereoInitialization`、所有 IMU 函数，以及旧配置解析路径。

## 10. `LocalMapping`：局部建图后台线程

文件：[`include/LocalMapping.h`](../include/LocalMapping.h)、[`src/LocalMapping.cc`](../src/LocalMapping.cc)

```text
构造函数
→ InsertKeyFrame / CheckNewKeyFrames
→ ProcessNewKeyFrame
→ MapPointCulling
→ CreateNewMapPoints
→ SearchInNeighbors
→ KeyFrameCulling
→ InterruptBA
→ stop / accept / reset / finish 控制函数
→ Run
```

纯单目一轮局部建图：

```text
处理新关键帧
→ 删除低质量新点
→ 与共视关键帧做极线匹配、三角化新点
→ 投影融合邻居地图点
→ LocalBundleAdjustment
→ 删除冗余关键帧
→ 将关键帧投入 LoopClosing
```

跳过 `InitializeIMU`、`ScaleRefinement`、IMU temporal 邻居、右相机 Fuse 和 stereo 深度反投影造点。最后读 `Run`。

## 11. `LoopClosing`：回环与地图合并后台线程

文件：[`include/LoopClosing.h`](../include/LoopClosing.h)、[`src/LoopClosing.cc`](../src/LoopClosing.cc)

```text
InsertKeyFrame / CheckNewKeyFrames
→ FindMatchesByProjection
→ DetectCommonRegionsFromLastKF
→ DetectAndReffineSim3FromLastKF
→ DetectCommonRegionsFromBoW
→ NewDetectCommonRegions
→ SearchAndFuse(KeyFrameAndPose)
→ RunGlobalBundleAdjustment
→ CorrectLoop
→ MergeLocal
→ reset / finish 控制函数
→ Run
```

检测链：

```text
KeyFrameDatabase 候选
→ KF-KF BoW 匹配
→ Sim3Solver RANSAC
→ Sim3 投影扩匹配
→ OptimizeSim3
→ 连续关键帧一致性验证
```

闭环矫正链：

```text
停止 LocalMapping
→ Sim3 修正当前共视窗口和地图点
→ 融合重复 MapPoint
→ OptimizeEssentialGraph
→ 添加 loop edge
→ 启动全局 BA
→ 恢复 LocalMapping
```

`MergeLocal` 仍是纯单目主线的一部分：它将跟踪丢失后创建的 Map 和旧 Map 做尺度自由 Sim3 对齐、融合和迁移。

跳过 `MergeLocal2`、`CheckObservations`、第二个 `SearchAndFuse(vector<KeyFrame*>)`，以及所有 IMU/4DoF 分支。

## 12. `System` 与应用入口

### 12.1 `System`

文件：[`include/System.h`](../include/System.h)、[`src/System.cc`](../src/System.cc)

```text
GetTrackingState / isShutDown
→ Reset / ResetActiveMap
→ System::System
→ TrackMonocular
→ Shutdown
→ SaveKeyFrameTrajectoryTUM
→ SaveMapPointsPLY
→ 析构函数
```

构造函数的核心装配顺序：

```text
Settings
→ Vocabulary
→ KeyFrameDatabase
→ Atlas
→ Tracking
→ LocalMapping 线程
→ LoopClosing 线程
→ 三模块互相注入指针
```

Tracking 在调用 `TrackMonocular` 的应用线程中同步运行；LocalMapping 和 LoopClosing 才是后台线程。

跳过 `TrackStereo`、`TrackRGBD`、IMU 接口、KITTI/EuRoC 输出和 Atlas 保存加载。

### 12.2 `mono_d435.cpp`

文件：[`apps/mono_d435.cpp`](../apps/mono_d435.cpp)

最后按此顺序阅读：

```text
CommandLine / CaptureSettings / ImagePacket
→ parseCommandLine
→ readCaptureSettings
→ selectDevice / configureSensors
→ tryGetInfraredFrame
→ writeEffectiveSettings
→ trackingStateName 与输出辅助函数
→ main
```

`main` 的核心只有：配置 D435 的单路 IR1 Y8，获得设备实际内参，生成有效 YAML，构造 `System::MONOCULAR`，循环调用 `TrackMonocular`，最后关闭并导出轨迹、点云。

## 首轮阅读中可以整体跳过的文件

```text
CameraModels/KannalaBrandt8.*
ImuTypes.*
G2oTypes.* 中的惯性内容
FrameDrawer.* / MapDrawer.* / Viewer.*
SerializationUtils.*
Config.*
Thirdparty/g2o 的求解器内部
Thirdparty/Sophus 的李群实现
DBoW2 的词典训练与聚类实现
```

## 读完后应能复述的三条链

```text
单目初始化：
Frame → ORBextractor → SearchForInitialization
→ TwoViewReconstruction → 创建 KeyFrame/MapPoint
→ Global BA → 中值深度尺度归一化
```

```text
正常跟踪与局部建图：
TrackMonocular → Track → 参考关键帧或运动模型
→ PoseOptimization → TrackLocalMap → 新关键帧
→ LocalMapping → 三角化、融合、Local BA
```

```text
丢失、回环与合并：
Relocalization → KeyFrameDatabase → MLPnPsolver → PoseOptimization

LoopClosing → BoW 候选 → Sim3Solver → OptimizeSim3
→ OptimizeEssentialGraph → Global BA / MergeLocal
```
