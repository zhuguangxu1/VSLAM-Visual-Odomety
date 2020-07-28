#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h"

namespace myslam {

/**
 * 配置类，使用SetParameterFile确定配置文件
 * 然后用Get得到对应值
 * 单例模式 !!!因为只有一个配置类!!! 所以构造函数设为私有函数 只用指针来进行访问
 */
class Config 
{
   private:
    static std::shared_ptr<Config> config_; //静态static数据 所有类共享这一个数据 归抽象类所有
    cv::FileStorage file_;  //数据存储与读取的类

    Config() {}  // 私有函数来构造一个单例
   public:
    ~Config();  // close the file when deconstructing

    // 静态函数 只能使用静态数据 因为静态对象是独立于实例出来的类的
    static bool SetParameterFile(const std::string &filename);

    // 访问参数值
    template <typename T>
    static T Get(const std::string &key) {
        return T(Config::config_->file_[key]); //dataset_dir在yaml配置的文件中 返回名称为nodename的节点数据 FIleNode FileStorage::operator[](const string& nodename)const 返回指定的元素
    }
};
}  // namespace myslam

#endif  // MYSLAM_CONFIG_H
/*
 * FileStorage类将各种opencv数据结构的数据存储为XML或者YAML格式 同时 也可以将其他类型的数值数据存储为这两种格式
 * cv::FileStorage(const string& source,int flags,const string& encoding=string() );
 * source  		存储或者读取的文件名字符串 其扩展名(.xml或者.yml/.yaml)决定文件格式
 * flags   		操作模式 包括：
 * FileStorage::READ    打开文件进行读操作
 * FileStorage::WRITE   打开文件进行写操作
 * FileStorage::APPEND  打开文件进行附加操作
 * FileStorage::MEMORY  从source读取数据，或者向内部缓存写入数据(由FileStorage::Release返回)
 * encoding             文件编码方式 
 * FileStorage::release  存储或者读取操作完成后，需要关闭文件并且释放内存
 * FIleNode FileStorage::operator[](const string& nodename)const 返回指定的元素
 */