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

#include "myslam/frame.h"

namespace myslam {

Frame::Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right)
        : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

Frame::Ptr Frame::CreateFrame() {
    static long factory_id = 0; //创建的新的一帧的id号 是静态数据 每次初始化可以直接忽略
    Frame::Ptr new_frame(new Frame);
    new_frame->id_ = factory_id++; //通过工厂函数的唯一id分配帧号
    return new_frame;   //复制这个智能指针 返回的是这个智能指针的拷贝 则该智能指针上的对象的引用数+1 因此在离开该作用域时原智能指针被销毁 
    //重点：引用计数值绑定在管理对象身上而不是指针本体 现在，当p被销毁时，它所指向的对象还有其他的使用者(重点：p指针是会被销毁，但是因为引用计数机制，它指向的内存没有被销毁)
}

void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;
    is_keyframe_ = true;			//更改该帧的标志位
    keyframe_id_ = keyframe_factory_id++;
}

}
//关于智能指针的实际应用
/*
 * void use_factory(T arg)
//   {
//        Foo* p = new Foo(arg);
//        //使用p但不delete它
//   } 
      p离开了它的作用域，但它所指向的内存没有被释放
//   在这个程序中，一旦use_factory被返回，程序就没有办法释放这块内存了
//   根据整个程序的逻辑，修正这个错误的正确方法是在use_factory中记得释放内存delete
//
//   与之相比
//   shared_ptr use_factory(T arg)
//   {
//        shared_ptr<Foo>p = make_shared<Foo>(arg);
//        return p;  //当我们返回p时，引用计数进行了递增操作
//   }p离开了作用域，但是它所指向的内存不会被释放掉
//   在这个智能指针版本中，use_factory中的return语句向此函数的调用者返回一个p的拷贝
//   拷贝一个shard_ptr会增加所管理对象的引用计数值（重点：引用计数值绑定在管理对象身上而不是指针本体）
//   现在，当p被销毁时，它所指向的对象还有其他的使用者(重点：p指针是会被销毁，但是因为引用计数机制，它指向的内存没有被销毁)
//   对于一块内存，shared_ptr类保证只要有任何shared_ptr对象使用它，它就不会被释放掉

//   为什么要使用智能指针而不建议使用new-delete？
//   答案：
//   delete:销毁指针对象，释放对应内存
//   shared_ptr引用计数为0：销毁指针对象，释放对应内存
//  （如果引用计数还不为0 即使销毁了指针对象，但是也不会释放内存，从而可以实现数据共享）
*/