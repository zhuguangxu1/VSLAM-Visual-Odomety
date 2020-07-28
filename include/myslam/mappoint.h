#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam {

struct Frame;

struct Feature;

/**
 * 路标点类
 * 特征点在三角化之后形成路标点
 */
struct MapPoint {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;
    unsigned long id_ = 0;  // ID
    bool is_outlier_ = false;
    Vec3 pos_ = Vec3::Zero();  // 世界坐标系下的3D位置
    std::mutex data_mutex_;
    int observed_times_ = 0; 				 //被特征匹配算法观察到的次数 也就是被多少特征点持有
    std::list<std::weak_ptr<Feature>> observations_;	 //持有的特征点的序列

    MapPoint() {}

    MapPoint(long id, Vec3 position);

    Vec3 Pos() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return pos_;
    }

    void SetPos(const Vec3 &pos) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        pos_ = pos;
    };

    void AddObservation(std::shared_ptr<Feature> feature) //用的是它的值 至始至终shared_ptr<Feature>只在帧的容器中 只有一份 这个备份值初始化后会被销毁 但是任务已经完成：构造了虚引用指针
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        observations_.push_back(feature);	//相当于用shared_ptr作为参数来构造list中的weak_ptr<Feature>
        observed_times_++;
    }

    void RemoveObservation(std::shared_ptr<Feature> feat);

    std::list<std::weak_ptr<Feature>> GetObs() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return observations_;
    }

    // factory function
    static MapPoint::Ptr CreateNewMappoint();
};
}  // namespace myslam

#endif  // MYSLAM_MAPPOINT_H
