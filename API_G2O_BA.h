#ifndef G2OBA_CPP
#define G2OBA_CPP


#include "API_draw.h"

#include <opencv2/core/core.hpp>

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <chrono>
#include <sophus/se3.hpp>
using namespace std;
using namespace cv;

/// vertex and edges used in g2o ba
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
 
  virtual void setToOriginImpl() override {
    _estimate = Sophus::SE3d();
  }
 
  /// left multiplication on SE3
  virtual void oplusImpl(const double *update) override {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }
 
  virtual bool read(istream &in) override {}
 
  virtual bool write(ostream &out) const override {}
};

 
/// g2o edge  观测值维度3 类型 Vector3d  优化节点第一个  VertexPose
class GNSSConstraintEdge : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
 
  GNSSConstraintEdge(const Eigen::Vector3d &GNSS_ENU) : _GNSS_ENU(GNSS_ENU) {}
 
  virtual void computeError() override {
        const VertexPose* v = static_cast<const VertexPose*>(_vertices[0]);
        Sophus::SE3d cam_pose = v->estimate();
        Vector3d cam_position = cam_pose.translation();
        _error = cam_position - _GNSS_ENU; //_measurement  这个误差通常定义为 VO-enu 转换后的坐标与 gnss-enu 之间的欧几里得距离。
        //cout<< "_error  "<<_error[0]<< " "<< _error[1]<< " "<< _error[2]<<endl;

        //   double error = (p2.inverse() * p1).log().norm();
  
  }
 
// virtual void linearizeOplus() override {
// //   //   VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
// //   //   Sophus::SE3d T = pose->estimate();
// //   //   Eigen::Vector3d xyz_trans = T * _point;
// //   //   _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
// //   //   _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(xyz_trans);
//      _jacobianOplusXi = Eigen::Matrix3d::Identity(); //假设同一个坐标
//  }
 
  bool read(istream &in) {}
 
  bool write(ostream &out) const {}
 
protected:
  Eigen::Vector3d _GNSS_ENU; // 真值 
};




void bundleAdjustment(
  const TrajectoryType &Camera_poses,
  const Point_txyz_List  &Gnss_enus,
  TrajectoryType &optimized_poses
  ) {
  // 构建图优化，先设定g2o
  //typedef g2o::BlockSolverX BlockSolverType;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
  
  // 梯度下降方法，可以从GN, LM, DogLeg 中选
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
  g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

  g2o::SparseOptimizer optimizer;     // 图模型
  optimizer.setAlgorithm(solver);   // 设置求解器
  optimizer.setVerbose(true);       // 打开调试输出
 
  
  int N = Gnss_enus.size();// 总对数
  


  for (size_t i = 0; i < N; i++) {
      Sophus::SE3d Camera_pose_i = Camera_poses[i];

      Eigen::Vector3d  Gnss_enu_i(Gnss_enus[i].x, Gnss_enus[i].y, Gnss_enus[i].z);

      //cout << i <<"===="<<endl;
      //cout << "Camera_pose_i : \n" << Camera_pose_i .translation()[0] <<"  " <<Camera_pose_i .translation()[1] << "  "<< Camera_pose_i .translation()[2] << endl;
      //cout << "Gnss_enu_i : \n" << Gnss_enus[i].x <<"  " <<Gnss_enus[i].y << "  "<< Gnss_enus[i].z << endl;
  
       // vertex
      VertexPose *pose = new VertexPose(); // camera pose R t
      pose->setId(i);
      pose->setEstimate(Camera_pose_i);

      optimizer.addVertex(pose);

      // Edge
      GNSSConstraintEdge *edge = new GNSSConstraintEdge(Gnss_enu_i); //GNSS ENU真值 3D-2D是图像1的3d点
      edge->setId(i);
      edge->setVertex(0, pose); // 获取优化第一个节点位姿 没有第二个节点位姿
      edge->setMeasurement(Gnss_enu_i); //GNSS ENU真值 3D-2D是图像2的像素点
      edge->setInformation(Eigen::Matrix3d::Identity());
      optimizer.addEdge(edge);

  }




 
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();
  cout << "开始优化"<< endl;
  optimizer.optimize(10);
  cout << "BAy优化结束"<< endl;
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "optimization 10次花费时间: " << time_used.count() << " 秒." << endl;

  // Extract optimized camera poses
  for (size_t i = 0; i < N; ++i) {
      VertexPose* v = dynamic_cast<VertexPose*>(optimizer.vertex(i));

    
      Sophus::SE3d optimized_pose = v->estimate();
      //optimized_poses.push_back(optimized_pose_SE3_Rt);
    
      // convert to cv::Mat
      Eigen::Matrix3d R_ = v->estimate().rotationMatrix();
      Eigen::Vector3d t_ = v->estimate().translation();
      //cout<<    i <<"  "<<t_ <<endl;

      
      // R = (Mat_<double>(3, 3) <<
      //   R_(0, 0), R_(0, 1), R_(0, 2),
      //   R_(1, 0), R_(1, 1), R_(1, 2),
      //   R_(2, 0), R_(2, 1), R_(2, 2)
      // );
      //t_ = Vector3d (t_(0)+100, t_(1)+100, t_(2)+10);  

      Sophus::SE3d optimized_pose_SE3_Rt(R_, t_);    
      optimized_poses.push_back(optimized_pose_SE3_Rt);

  
  }
 cout << "数据取出完毕"<<  endl;

}

#endif 