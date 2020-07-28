//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "myslam/common_include.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
/**
总体流程:	SLAM p187 误差关于位姿与关于三维点的雅克比矩阵的推导          		---观测相机方程关于相机位姿与特征点的两个导数矩阵
1、定义优化节点 设置需要更新的变量类型_estimate
2、定义误差边
(1)计算曲线模型误差
    实例化节点类v
    v->estimate返回设置的需要估计的参数 与v中的更新参数update的类型一致
    _error= _measurement - 利用估计参数设置的方程(其中_measurment后续会传入观测值)
(2)计算雅可比矩阵
    实例化节点类
    v->estimate返回设置的需要估计的参数 与v中的更新参数update的类型一致
    利用估计参数设置需要优化的方程
    计算建立的方程对于各个优化参数的雅可比矩阵（也就算偏导数）
 */
namespace myslam {
// vertex and edges used in g2o ba
// 位姿顶点
// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class VertexPose : public g2o::BaseVertex<6, SE3> 
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override { _estimate = SE3(); }   			//估计值存在默认值类型 

    /// SE3上的左乘法
    virtual void oplusImpl(const double *update) override {
        Vec6 update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4],
            update[5];
        _estimate = SE3::exp(update_eigen) * _estimate;
    }

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true; }
};

/// 路标顶点
class VertexXYZ : public g2o::BaseVertex<3, Vec3> 
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void setToOriginImpl() override { _estimate = Vec3::Zero(); }		//存在默认值

    virtual void oplusImpl(const double *update) override {
        _estimate[0] += update[0];
        _estimate[1] += update[1];
        _estimate[2] += update[2];
    }

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true; }
};

/// 仅估计位姿的一元边
// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose> 
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectionPoseOnly(const Vec3 &pos, const Mat33 &K,const SE3& cam_ext)		//这里的K是内参 pos是外参
        : _pos3d(pos), _K(K) ,_cam_ext(cam_ext){}

    virtual void computeError() override 				//(1)计算模型误差
    {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);	//1、实例化节点类v   0代表连接的是自定义的第一种优化节点 
        SE3 T = v->estimate();						//2、v->estimate返回设置的需要估计的参数 与v中的更新参数update的类型一致					
        Vec3 pos_pixel = _K * (_cam_ext * (T * _pos3d) );		
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();			//3、_error= _measurement - 利用估计参数设置的方程(其中_measurment后续会传入观测值)
    }

    virtual void linearizeOplus() override 				//(2)、计算雅可比矩阵
    {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);	//1、实例化节点类v   0代表连接的是自定义的第一种优化节点 
        SE3 T = v->estimate();						//2、v->estimate返回设置的需要估计的参数 与v中的更新参数update的类型一致
        Vec3 pos_cam = _cam_ext * T * _pos3d;				//像素坐标系的坐标
        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Zinv = 1.0 / (Z + 1e-18);
        double Zinv2 = Zinv * Zinv;
        _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
            -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
            fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
            -fy * X * Zinv;
    }		//优化X Y Z 顶点坐标

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true; }

   private:
    Vec3 _pos3d;
    Mat33 _K;
    SE3 _cam_ext;
};

/// 带有地图和位姿的二元边
class EdgeProjection
    : public g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /// 构造时传入相机内外参
    EdgeProjection(const Mat33 &K, const SE3 &cam_ext) : _K(K) {
        _cam_ext = cam_ext;		//左相机的位姿或者右相机的位姿
    }
    //与上面流程相同
    virtual void computeError() override {
        const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
        const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
        SE3 T = v0->estimate();							//同时使用两种估计类型SE3和Vec3
        Vec3 pos_pixel = _K * (_cam_ext * (T * v1->estimate()));		//像素误差  T * v1->estimate()将点转换为相机坐标系 T人为设置成帧相对于世界坐标系的位姿
        pos_pixel /= pos_pixel[2];						//可以看成是一种李代数的扰动的形式 相机位姿是初始位姿cam_ext 帧的扰动是T
        _error = _measurement - pos_pixel.head<2>();				//cam_ext * T相当于在帧的那一时刻的“相机位姿”
    }

    virtual void linearizeOplus() override {
        const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
        const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
        SE3 T = v0->estimate();							//v->estimate返回设置的需要估计的参数类型 与v中的更新参数update的类型一致
        Vec3 pw = v1->estimate();
        Vec3 pos_cam = _cam_ext * T * pw;					//_cam_ext是左相机或者右相机的位姿  _cam_ext*T是合并的一个项 代表转换矩阵？
        double fx = _K(0, 0);
        double fy = _K(1, 1);							//世界坐标系->帧坐标系->相机坐标系
        double X = pos_cam[0];							
        double Y = pos_cam[1];							//相机坐标系下的坐标
        double Z = pos_cam[2];
        double Zinv = 1.0 / (Z + 1e-18);
        double Zinv2 = Zinv * Zinv;
        _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,			//误差相对于位姿的雅克比矩阵
            -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
            fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
            -fy * X * Zinv;

        _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *					//eigen.block 提取一个2*3的块 块起始位置是(0,0)
                           _cam_ext.rotationMatrix() * T.rotationMatrix();			//误差相对于路标点的雅克比矩阵
    }

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true; }

   private:
    Mat33 _K;
    SE3 _cam_ext;
};

}  // namespace myslam

#endif  // MYSLAM_G2O_TYPES_H
