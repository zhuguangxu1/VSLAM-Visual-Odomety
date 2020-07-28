#include "myslam/dataset.h"
#include "myslam/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace std;

namespace myslam {

Dataset::Dataset(const std::string& dataset_path)
    : dataset_path_(dataset_path) {}

bool Dataset::Init() {
    // 读取相机的内参和外参
    ifstream fin(dataset_path_ + "/calib.txt"); //dataset_path_是数据集的路径
    if (!fin) {
        LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
        return false;
    }

    for (int i = 0; i < 4; ++i) 
    {
        char camera_name[3];  //P0: P1: P2: P3: 相机的名字是单个字符组成的数组
        for (int k = 0; k < 3; ++k) 
	{
            fin >> camera_name[k]; //前三个字符是相机的名称 PX:  读完之后就丢弃了 
        }
        double projection_data[12]; //投影数据
        for (int k = 0; k < 12; ++k) 
	{
            fin >> projection_data[k];
        }
        Mat33 K; //内参结构
        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];
        Vec3 t;	 //初始化平移
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t; //不是很明白这一步为什么要进行变换
        K = K * 0.5;
        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),   //推测是初始时刻利用平移来进行初始化 前四个参数fx cx fy cy
                                          t.norm(), SE3(SO3(), t)));    //norm是求范数 SE3是通过t构建的T运动矩阵 这是SE3一种构造函数的形式 t.norm是基线baseline的实参
        cameras_.push_back(new_camera); //放入数据集类维护的相机序列 
	//真正的释放是智能指针本身的释放才会引起内存的实际释放,也就是说只要还有对象持有该智能指针（比如数据集的vector序列仍然持有这些智能指针 就不会被释放）
        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();    //extrinsics外参
    }
    fin.close(); //关闭IO读取
    current_image_index_ = 0; //初始化 现有的图像序列为0
    return true;
}
//初始化获得四个相机之后开始一帧一帧的读取 前端和后端均只用前2个相机作为左目和右目进行初始化
//函数作用：首先读取两张左目和右目的图片 之后利用帧类的工厂函数创建经由两帧立体图像构造得到新的一帧 
Frame::Ptr Dataset::NextFrame() {
    boost::format fmt("%s/image_%d/%06d.png");
    //boost::format( "format-string ") % arg1 % arg2 % ... % argN; 
    //format-string代表需要格式化的字符串，后面用重载过的%跟参数
    cv::Mat image_left, image_right;
    // 读取图片
    image_left =
        cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);
    //这里的dataset_path对应着第一个%s  0对应第二个%d  current_image_index_对应第三个%06d
    //使用 %序号% 的方式给出指示符，后面用%连接对应的数据
    image_right =
        cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);
    //分别读左目image_0和右目image_1的数据
    if (image_left.data == nullptr || image_right.data == nullptr) {
        LOG(WARNING) << "cannot find images at index " << current_image_index_;
        return nullptr;
    }

    cv::Mat image_left_resized, image_right_resized;
    cv::resize(image_left, image_left_resized, cv::Size(), 0.5,0.5,
               cv::INTER_NEAREST); //将图片按比例缩小至原来的0.5倍
    cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    //到这一步就有了缩小0.5倍之后的左目和右目图像
    auto new_frame = Frame::CreateFrame(); //工厂函数 这个函数是静态成员函数 只能使用这种作用域的形式来进行调用 创建的时候分配了id号为0 这时候创建的帧只有一个初始化的帧号
    new_frame->left_img_ = image_left_resized;
    new_frame->right_img_ = image_right_resized;
    current_image_index_++;   //current_image_index_是Dataset类的成员变量
    return new_frame;   //返回创建的这个新的一帧
}

}  // namespace myslam