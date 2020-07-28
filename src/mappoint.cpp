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

#include "myslam/mappoint.h"
#include "myslam/feature.h"

namespace myslam {

MapPoint::MapPoint(long id, Vec3 position) : id_(id), pos_(position) {}

MapPoint::Ptr MapPoint::CreateNewMappoint() {
    static long factory_id = 0;
    MapPoint::Ptr new_mappoint(new MapPoint);
    new_mappoint->id_ = factory_id++;
    return new_mappoint;
}

void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat) 
{
    std::unique_lock<std::mutex> lck(data_mutex_);
    for (auto iter = observations_.begin(); iter != observations_.end();
         iter++) 
	 {
        if (iter->lock() == feat) 	//在路标点维护的shared_ptr(weak_ptr的lock得到)特征点序列中删去 删除帧中的特征点shared_ptr指针
	{
            observations_.erase(iter);
            feat->map_point_.reset();   //feat->map_point_是指向路标点的weak_ptr 将该weak_ptr置为空
            observed_times_--;		//该路标点持有的特征点数量-1
            break;
        }
    }
}

}  // namespace myslam
