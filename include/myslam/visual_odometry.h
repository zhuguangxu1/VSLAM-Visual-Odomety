#pragma once
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "myslam/backend.h"
#include "myslam/common_include.h"
#include "myslam/dataset.h"
#include "myslam/frontend.h"
#include "myslam/viewer.h"

namespace myslam {

/**
 * VO 对外接口
 */
class VisualOdometry {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    /// constructor with config file
    VisualOdometry(std::string &config_path);//利用配置文件的路径进行构造

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init(); //里程计初始化

    /**
     * start vo in the dataset
     */
    void Run();//运行函数

    /**
     * Make a step forward in dataset
     */
    bool Step();//步进函数

    // 获取前端状态
    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

   private:
    bool inited_ = false;
    std::string config_file_path_;

    Frontend::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;                  //这里的指针均为shared_ptr 前端 后端 地图 观测者 数据集持有的均是shared_ptr 通过指针可以直接访问另一个类
    Viewer::Ptr viewer_ = nullptr;

    // dataset
    Dataset::Ptr dataset_ = nullptr;
};
}  // namespace myslam

#endif  // MYSLAM_VISUAL_ODOMETRY_H
/**
 * 静态变量只能被初始化定义一次，之后重复的初始化会被编译器忽略
 * int fun()
 * {
 * 	static int i = 0;
 * 	i++;
 * 	return i;
 * }
 * 当这个函数被反复调用时，i的值会递增，也就是静态变量只能被初始化一次
 * 操作系统在加载程序时会根据程序中的声明部分为程序分配内存空间(这部分数据是由编译器生成的)
 * 程序所支配的内存空间可以分为两大部分：静态区域和动态区域
 * 动态数据用于存储经常会发生变动的数据(栈和堆) 静态数据用于存储不会经常发生变化的区域(程序的指令代码--程序编译后得到的代码、用户类型(结构体、类的声明代码)、全局变量、静态变量)
 * 任何变量都只能进行一次初始化，局部变量在程序块结束时生存期就结束了，下次再调用这个程序块从原理上来说声明的是另一个变量了(分配到的地址不一样)
 */