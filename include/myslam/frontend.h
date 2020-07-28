#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {

class Backend;
class Viewer;

enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };  //枚举类的应用

/**
 * 前端
 * 估计当前帧Pose，在满足关键帧条件时向地图加入关键帧并触发优化
 */
class Frontend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    /// 外部接口，添加一个帧并计算其定位结果
    bool AddFrame(Frame::Ptr frame);

    /// Set函数
    void SetMap(Map::Ptr map) { map_ = map; }

    void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

    void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

    FrontendStatus GetStatus() const { return status_; }

    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        camera_left_ = left;
        camera_right_ = right;
    }

   private:
    /**
     * 正常模式下的跟踪
     * @return true if success
     */
    bool Track();
    
    /**
     * 丢失目标时进行重置
     * @return true if success
     */
    bool Reset();

    /**
     * 跟踪最后一帧
     * @return 返回跟踪到的特征点
     */
    int TrackLastFrame();

    /**
     * 估计当前帧的位姿
     * @return num of inliers
     */
    int EstimateCurrentPose();

    /**
     * 将当前帧设置为关键帧并加入后端优化
     * @return true if success
     */
    bool InsertKeyframe();

    /**
     * 尝试使用保存在current_frame_中的立体图像来初始化前端
     * @return true if success
     */
    bool StereoInit();

    /**
     * 在当前帧中检测左目图像中的特征点
     * 特征点会被保存在当前帧中
     * @return
     */
 
    int DetectFeatures();

    /**
     * 在当前帧的右目图像中找到相对应的特征点
     * @return 返回找到的特征点的数量
     */
    int FindFeaturesInRight();

    /**
     * 用单个图像构建初始地图
     * @return true if succeed
     */
    bool BuildInitMap();

    /**
     * 对当前帧中的2D特征点进行三角化
     * @return 返回三角化得到的3D点的数量
     */
    int TriangulateNewPoints();

    /**
     * Set the features in keyframe as new observation of the map points 在关键帧中设置特征点作为对地图点的新观察
     */
    void SetObservationsForKeyFrame();

    // 前端的初始化状态是INITING
    FrontendStatus status_ = FrontendStatus::INITING;

    Frame::Ptr current_frame_ = nullptr;  // 当前帧
    Frame::Ptr last_frame_ = nullptr;     // 上一帧
    Camera::Ptr camera_left_ = nullptr;   // 左侧相机
    Camera::Ptr camera_right_ = nullptr;  // 右侧相机

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    SE3 relative_motion_;  // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

    int tracking_inliers_ = 0;  // inliers, used for testing new keyframes 内线，用于测试新的关键帧

    // 参数值 均存在默认值
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt_;  // opencv中的特征检测器
};

}  // namespace myslam

#endif  // MYSLAM_FRONTEND_H
