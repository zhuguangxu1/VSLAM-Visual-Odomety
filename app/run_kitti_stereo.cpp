//
// Created by gaoxiang on 19-5-4.
//

#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"

DEFINE_string(config_file, "/home/zhuguangxu/SLAM/slambook2-master/ch13//config/default.yaml", "config file path");
//DEFINE_xxxxx(变量名，默认值,help-string)
int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true); //调用函数解析命令行
    //gflags 命令行解析专用
    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(FLAGS_config_file)); //使用“FLAGS_”前缀的宏，来访问命令行标记 访问参数变量 加上FLAGS_ 之前是config_file加上FLAGS 创建一个视觉里程计
    assert(vo->Init() == true);//对视觉里程计进行初始化 数据集维护了初始化的包含4个相机的vector序列 并且初始化前端和后端及其他类
    vo->Run();//视觉里程计开始运行

    return 0;
}

/*
上述的DEFINE宏包含有三个参数
1、flag的命名
2、flag的默认值
3、该flag对应的一些提示性的说明 当用户传入-help标志的时候向用户展示提示性的说明

只可以在一个文件中对某个flags进行定义 一旦在一个文件中给出了定义宏之后，在其他文件中想要使用该flag 需要使用DECLARE宏来进行声明
通常在.cpp文件中给出flag的定义，在.h文件中进行DECLARE声明，然后任何包含该头文件的文件均可以使用该flag
用DEFINE宏定义的flag都可以像普通的变量一样进行调用，定义的变量是以FLAGS_为前缀
google::ParseCommandLineFlags(&argc, &argv, true);
前两个参数通常用户会在Main函数中给出，最后一个参数是remove_flags 如果为true的话，ParseCommandLineFlags会移除相应的flag和对应的参数
并且修改相应的argc，然后argv只会保留命令行参数 如果为false的话，会保持argc不变，但是会调整argv中存储的内容，并且把flags放在命令行参数的前面
*/