/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include"sophus/sim3.hpp"

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"


namespace ORB_SLAM3
{

    class ORBmatcher
    {
    public:
        //构造函数创建一个 ORB 特征匹配器。
        //nnratio：最近邻距离比阈值，默认是 0.6。
        //checkOri：是否检查匹配特征点的方向一致性，默认开启。
        ORBmatcher(float nnratio=0.6, bool checkOri=true);

        //1.计算两个 ORB 描述子之间的汉明距离。
        static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

        // 将局部地图中的地图点投影到当前帧 F，然后在投影位置附近寻找匹配特征。
        // 主要用于 Tracking 中的“跟踪局部地图”。
        // F：当前待匹配帧。
        // vpMapPoints：局部地图点集合。
        // th：搜索窗口的放大系数。它通常还会结合特征点的金字塔层级和观察角度决定实际搜索半径。
        // bFarPoints：是否对远距离地图点进行特殊处理或限制。
        // thFarPoints：远点距离阈值，默认 50，通常以地图使用的距离单位为准。
        int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3, const bool bFarPoints = false, const float thFarPoints = 50.0f);

        // 把上一帧中已经关联的地图点投影到当前帧，并在附近搜索匹配。
        // 主要用于帧间跟踪
        // CurrentFrame：当前帧。
       //  LastFrame：上一帧。
       //  th：投影搜索窗口大小系数。
       //  bMono：是否为单目模式。
       //  true：单目；
       //  false：双目或 RGB-D。
        int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

        // Project MapPoints seen in KeyFrame into the Frame and search matches.
        // Used in relocalisation (Tracking)
        int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

        // Project MapPoints using a Similarity Transformation and search matches.
        // Used in loop detection (Loop Closing)
        int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th, float ratioHamming=1.0);

        // Project MapPoints using a Similarity Transformation and search matches.
        // Used in Place Recognition (Loop Closing and Merging)
        int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs, std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming=1.0);

        // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
        // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
        // Used in Relocalisation and Loop Detection
        int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
        int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);

        // 在单目初始化阶段匹配连续两帧中的 ORB 特征 (单目系统刚启动时还没有地图点，也没有可靠的相机位姿，因此不能使用地图点投影，只能直接匹配两个图像中的特征。)
        // F1：初始化参考帧。
        // F2：当前帧。
        // vbPrevMatched：每个 F1 特征点在 F2 中的预测搜索位置。成功匹配后通常会更新为实际匹配位置，便于下一次继续跟踪。
        // vnMatches12：输出索引映射。
        // vnMatches12[i] = j：F1 的第 i 个特征匹配到了 F2 的第 j 个特征；
        // windowSize：围绕预测位置的搜索窗口大小，默认 10 像素左右。
        // 返回值：
        // 初始化匹配数量。
        int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

        // 三角化匹配在两个关键帧之间寻找适合三角化的特征匹配，用来生成新的地图点。
        // 它通常只处理尚未关联地图点的特征，并利用极线约束判断匹配是否合理。
        // bOnlyStereo：是否只考虑具有双目深度信息的特征。
        // bCoarse：是否使用较宽松、较粗略的匹配策略，常用于地图合并或匹配条件较差的情况。
        int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2,
                                   std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse = false);

        // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
        // In the stereo and RGB-D case, s12=1
        // int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);
        int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const Sophus::Sim3f &S12, const float th);

        // Project MapPoints into KeyFrame and search for duplicated MapPoints.
        int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0, const bool bRight = false);

        // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
        int Fuse(KeyFrame* pKF, Sophus::Sim3f &Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

    public:

        static const int TH_LOW;
        static const int TH_HIGH;
        static const int HISTO_LENGTH;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        float RadiusByViewingCos(const float &viewCos);

        void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

        float mfNNratio;
        bool mbCheckOrientation;
    };

}// namespace ORB_SLAM

#endif // ORBMATCHER_H