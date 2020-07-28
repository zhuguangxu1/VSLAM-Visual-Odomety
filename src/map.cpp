/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "myslam/map.h"
#include "myslam/feature.h"

namespace myslam {

void Map::InsertKeyFrame(Frame::Ptr frame) 
{
    current_frame_ = frame;
    if (keyframes_.find(frame->keyframe_id_) == keyframes_.end())  		//与下同 添加进哈希表
    {
        keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
    } else 
    {
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    if (active_keyframes_.size() > num_active_keyframes_) 
    {
        RemoveOldKeyframe();							//移走旧的关键帧 并触发一次地图优化 
    }
}

void Map::InsertMapPoint(MapPoint::Ptr map_point) 
{
    if (landmarks_.find(map_point->id_) == landmarks_.end())     //map_point是经过路标点类的工厂函数得到的路标点实例 其id是工厂函数递增分配的 这里满足条件相当于是新添加的
    {
        landmarks_.insert(make_pair(map_point->id_, map_point)); 		//将这个新的路标点添加进所有路标点的哈希表
        active_landmarks_.insert(make_pair(map_point->id_, map_point));		//将这个新的路标点添加进活跃的路标点的哈希表
    } else 
    {
        landmarks_[map_point->id_] = map_point;					//直接赋值
        active_landmarks_[map_point->id_] = map_point;
    }
}

void Map::RemoveOldKeyframe() {
    if (current_frame_ == nullptr) return;
    // 寻找与当前帧最近与最远的两个关键帧
    double max_dis = 0, min_dis = 9999;
    double max_kf_id = 0, min_kf_id = 0;
    auto Twc = current_frame_->Pose().inverse();		//利用当前帧的位姿来进行判断
    for (auto& kf : active_keyframes_) 				//遍历活跃关键帧的哈希表
    {
        if (kf.second == current_frame_) continue;
        auto dis = (kf.second->Pose() * Twc).log().norm();	//找出每一个活跃关键帧与当前帧的位姿矩阵的差值
        if (dis > max_dis) 
	{
            max_dis = dis;
            max_kf_id = kf.first;
        }							//搜索法找到最大的距离和最小的距离以及对应的关键帧编号
        if (dis < min_dis) 
	{
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    const double min_dis_th = 0.2;  // 最近阈值
    Frame::Ptr frame_to_remove = nullptr;
    if (min_dis < min_dis_th) {
        // 如果存在很近的帧，优先删掉最近的
        frame_to_remove = keyframes_.at(min_kf_id);
    } else {
        // 删掉最远的
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;
    // remove keyframe and landmark observation
    active_keyframes_.erase(frame_to_remove->keyframe_id_); 	//从这里看出来 只在活跃关键帧哈希表中删除这个关键帧 但是在所有的关键帧哈希表中是保留的
    for (auto feat : frame_to_remove->features_left_) 		//在删除帧维护的特征点序列中进行遍历
    {
        auto mp = feat->map_point_.lock(); 			//map_point_在frontend.cpp的初始化地图函数中经由shared_ptr构建 通过.lock()函数获得指向该路标点的shared_ptr
        if (mp) 						//如果存在经由该删除帧特征点三角化的路标点 一个特征点只有一个虚引用指针指向三角化后的路标点
	{
            mp->RemoveObservation(feat);			//该路标点持有的特征点序列中删除这个要删除的帧中的特征点 同时将该特征点中的weak_ptr置为空
        }							//路标点同样持有指向特征点的虚引用指针链表  同时特征点只有一个指向路标点的虚引用指针
    }
    for (auto feat : frame_to_remove->features_right_) {
        if (feat == nullptr) continue;
        auto mp = feat->map_point_.lock();
        if (mp) {
            mp->RemoveObservation(feat);			//与上同
        }
    }

    CleanMap();						//清除地图中观测数量为0的路标点  每次需要移除旧的关键帧的时候才会触发这一函数 不加这一句 会出全图 但是很卡
}

void Map::CleanMap() 
{
    int cnt_landmark_removed = 0;
    for (auto iter = active_landmarks_.begin();
         iter != active_landmarks_.end();) 			//在地图维护的活跃的路标点中进行遍历
	 {
        if (iter->second->observed_times_ == 0) 		//iter->second是Map::Ptr
	{
            iter = active_landmarks_.erase(iter);		//在该数据结构中删除 并返回下上一个迭代器的位置
            cnt_landmark_removed++;
        } else 
	{
            ++iter;
        }
    }
    LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
}

}  // namespace myslam
