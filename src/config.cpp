#include "myslam/config.h"

namespace myslam {
bool Config::SetParameterFile(const std::string &filename) {
    if (config_ == nullptr) //config_是static指针 因此config是一个单例模式
        config_ = std::shared_ptr<Config>(new Config); //私有函数在类内构造是完全可以的 这是一个静态的单例 通过一个抽象类的调用构造了一个单例的类
    config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ); //以只读模式打开这个文件 这里是之前设置的config_file
    if (config_->file_.isOpened() == false) {
        LOG(ERROR) << "parameter file " << filename << " does not exist.";
        config_->file_.release(); //存储或者读取操作完成之后，需要关闭这个文件并且释放内存
        return false;
    }
    return true;
}

Config::~Config() {
    if (file_.isOpened())   //如果还在开着  就要把这个资源释放
        file_.release();
}

std::shared_ptr<Config> Config::config_ = nullptr;

}
