//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/visual_odometry.h"
#include <chrono>
#include "myslam/config.h"

namespace myslam {

VisualOdometry::VisualOdometry(std::string &config_path)   //采用外部配置的路径进行里程计的初始化
    : config_file_path_(config_path) {}

//这个函数只执行一次 实际上只创建了一个数据集智能指针 也是一种隐式的单例模式
bool VisualOdometry::Init() {
    // 从给定路径中读取数据集
    if (Config::SetParameterFile(config_file_path_) == false)    //调用的是成员函数 在这一行如果还没有配置实例 会唯一创建一个出来
    {   //该函数的含义 以只读模式打开这个config_file路径的文件
        return false;
    }
    //执行到这一步 相当于成功打开了.yaml配置文件
    dataset_ =
        Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir"))); //FIleNode FileStorage::operator[](const string& nodename)const 返回指定的元素的对应数据
	//new Dataset(Config::Get<std::string>("dataset_dir"))返回名称为dataset_dir的节点数据 dataset_dir是在yaml文件中的
	//利用该配置文件中的dataset_dir数据集路径来构造了一个Dataset
	//这个Dataset类是用一个数据集路径来进行构造的，只是实例化了一个Dataset的对象出来 够用了
	
    CHECK_EQ(dataset_->Init(), true);//初始化的情况下数据集的相机序列已经有了四个相机 且通过t进行了初始化
    //EQ即equation 函数判断是否相等 当不相等时，函数打印出不相等的关系

    // 初始化流程
    frontend_ = Frontend::Ptr(new Frontend);   //利用默认构造的方式来创建前端
    backend_ = Backend::Ptr(new Backend);      //利用默认构造的方式来创建后端
    map_ = Map::Ptr(new Map);                  //利用默认构造的方式来创建地图
    viewer_ = Viewer::Ptr(new Viewer);         //利用默认构造的方式来创建观测者

    frontend_->SetBackend(backend_);           //在满足条件时加入后端优化
    frontend_->SetMap(map_);                   //在前端中设置地图
    frontend_->SetViewer(viewer_);             //在前端中设置观测者
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1)); //设置左右目相机 左目相机是初始化相机序列中的相机0 右目相机是初始化相机序列中的相机1 
    //初始化情况下前端已经创建了相机 这个相机是存在位姿的
    backend_->SetMap(map_);                    //在后端中设置地图
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));  //后端相机设置与前端相机设置相同

    viewer_->SetMap(map_);                     //在观测者中创建地图

    return true;
}

void VisualOdometry::Run() {
    while (1) 
    {
        LOG(INFO) << "VO is running";
        if (Step() == false)  //一直循环运行步进函数
	{
            break;
        }
    }

    backend_->Stop();
    viewer_->Close();

    LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step() {
    Frame::Ptr new_frame = dataset_->NextFrame();   //首先读取两张左目和右目的图片 之后利用帧类的工厂函数创建经由两帧立体图像构造得到新的一帧 
    if (new_frame == nullptr) return false;

    auto t1 = std::chrono::steady_clock::now();
    
    bool success = frontend_->AddFrame(new_frame);  //函数作用：将这个利用两幅图像构建的新的一帧加入前端 status_是INITING()初始化创建的Frontend 
    //Addframe 外部接口，添加一个帧并计算其定位结果
    
    auto t2 = std::chrono::steady_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return success;
}

}  // namespace myslam
