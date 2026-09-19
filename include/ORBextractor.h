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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>//引入 std::list，用于保存空间划分节点。 使用链表的原因是：划分过程中需要不断插入、删除节点，并保持已有节点迭代器有效。
#include <opencv2/opencv.hpp>


namespace ORB_SLAM3
{
//它表示图像空间中的一个矩形区域，用于把关键点均匀分布到整幅图像。
class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    //把当前矩形区域分成四个子矩形，并通过引用参数返回
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;//保存位于当前矩形区域中的候选关键点。cv::KeyPoint 中常用字段包括：
                                    // keypoint.pt        坐标
                                    // keypoint.response  角点响应强度
                                    // keypoint.angle     方向
                                    // keypoint.octave    所属金字塔层
                                    // keypoint.size      对应的特征区域大小
    cv::Point2i UL, UR, BL, BR;//节点边界
    std::list<ExtractorNode>::iterator lit;//保存该节点在节点链表中的位置。 节点被划分后，可以通过这个迭代器快速找到并删除旧的父节点，而不必重新遍历整个链表。
    bool bNoMore;//停止划分标志
};

class ORBextractor
{
public:
    
    //enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    int operator()( cv::InputArray _image, cv::InputArray _mask,//图像掩膜
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::OutputArray _descriptors, std::vector<int> &vLappingArea);//vLappingArea主要用于鱼眼双目的重叠区域：

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;//保存最近一次输入图像对应的所有金字塔层

protected:

    void ComputePyramid(cv::Mat image);//生成图像金字塔

    // 遍历每一层图像；
    // 把当前层划分成小网格；
    // 每格先用 iniThFAST 检测；
    // 没有点时改用 minThFAST；
    // 汇总候选点；
    // 调用 DistributeOctTree() 做空间均匀化；
    // 设置关键点的 octave 和 size；
    // 使用 umax 计算方向。
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);//完成主要的关键点检测和均匀化工作。

    //从某一层图像金字塔检测到的大量 FAST 候选点中，选出大约 N 个空间分布均匀、响应值较强的关键点。
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    std::vector<cv::Point> pattern;//保存旋转 BRIEF 描述子的固定采样位置

    int nfeatures;//每幅图像希望提取的 ORB 特征点总数
    double scaleFactor;//相邻两层图像金字塔之间的尺度倍率，通常记作 s
    int nlevels;//金字塔总层数
    int iniThFAST;//FAST 角点检测时使用的阈值。FAST 会比较候选像素与其周围圆环像素的灰度差，阈值越大：要求对比度越强，得到的角点通常更稳定，但特征点数量可能更少
    int minThFAST;//FAST 检测的备用低阈值。ComputeKeyPointsOctTree() 实现中，每个图像小网格会：先用 iniThFAST 检测；如果一个点都没有，再用 minThFAST 重新检测。

    std::vector<int> mnFeaturesPerLevel;//记录每一层计划提取多少个特征点。按照几何级数分配：低层较多，高层逐渐减少。

    std::vector<int> umax;//这是计算关键点方向时使用的圆形区域边界查找表。 ORB-SLAM3 使用半径为 15 的圆形 patch。 umax[v] 表示：当垂直偏移为 v 时，圆内允许的最大水平偏移 |u|

    std::vector<float> mvScaleFactor;//保存每个金字塔层相对于原图的尺度倍率。主要用途有两个。第一，把金字塔层坐标还原到原图坐标。第二，设置关键点在原图尺度下的 patch 大小
    std::vector<float> mvInvScaleFactor;//上一项的倒数，它用于创建图像金字塔
    std::vector<float> mvLevelSigma2;//保存每个层级对应的尺度平方
    std::vector<float> mvInvLevelSigma2;//尺度平方的倒数。它常被当作误差权重
};

} //namespace ORB_SLAM

#endif

