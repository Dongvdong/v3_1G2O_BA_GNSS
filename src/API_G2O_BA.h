#ifndef G2OBA_CPP
#define G2OBA_CPP


#include "API_dataAndFileIO.h"


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
#include <g2o/core/robust_kernel_impl.h>

#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/se3quat.h> // 包含 SE3Quat 类型的头文件
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <chrono>
#include <sophus/se3.hpp>
using namespace std;
using namespace cv;




/// 视觉点 6维度
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

 
// GNSS提供的位置顶点类  没必要使用
class VertexGNSS : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void setToOriginImpl() override
    {
        _estimate.setZero();
    }

    virtual void oplusImpl(const double* update) override
    {
        _estimate += Eigen::Vector3d(update);
    }

    virtual bool read(std::istream& is) override
    {
        // Implement reading from stream if needed
        return false;
    }

    virtual bool write(std::ostream& os) const override
    {
        // Implement writing to stream if needed
        return false;
    }
};


/// g2o edge  观测值维度3 类型 Vector3d  优化节点第一个  VertexPose
class GNSSConstraintEdge : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
 
  GNSSConstraintEdge(const Eigen::Vector3d &GNSS_ENU) : _GNSS_ENU(GNSS_ENU) {}
 
  virtual void computeError() override {
        const VertexPose* v = static_cast<const VertexPose*>(_vertices[0]);
        Sophus::SE3d cam_pose = v->estimate();


        const  Eigen::Vector3d t_cw = cam_pose.translation();
        // cam_pose.matrix() 
        //const  Eigen::Matrix3d R_cw = cam_pose.rotationMatrix(); 
        //const  Eigen::Vector3d t_wc = -R_cw.transpose() * t_cw;
        // 齐次t
        //const Vec4_t t_wc_h(t_wc(0), t_wc(1), t_wc(2), 1);
        //_error = srt * t_wc_h - _measurement; // slam->real ecef - 测量gps->ecef
        _error = t_cw - _GNSS_ENU; //_measurement  这个误差通常定义为 VO-enu 转换后的坐标与 gnss-enu 之间的欧几里得距离。 GNSS_ENU
       
  
  }


  
 
 virtual void linearizeOplus() override {

    g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose>::linearizeOplus();// 继承自动求导

 }
 
  bool read(istream &in) {}
 
  bool write(ostream &out) const {}
 
protected:
  Eigen::Vector3d _GNSS_ENU; // 真值 
};



/// g2o edge  观测值维度3 类型 Vector3d  优化节点第一个  VertexPose

class EdgeRelativeRT : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexPose ,VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //EdgeRelativeRT() : g2o::BaseBinaryEdge<6, Eigen::Isometry3d, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>() {}
    // 构造函数重写
    EdgeRelativeRT(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,double t_var, double R_var)
        : R(R), t(t), t_var(t_var), R_var(R_var)
    {}

    void computeError()
    {
        const VertexPose* vi = static_cast<const VertexPose*>(_vertices[0]);
        const VertexPose* vj = static_cast<const VertexPose*>(_vertices[1]);

        // 计算相对变换 T_ij  局部坐标系
        Sophus::SE3d T_iw = vi->estimate();
        Sophus::SE3d T_jw = vj->estimate();
        Sophus::SE3d T_ij = T_iw.inverse() * T_jw;   // j在 i坐标系下的位姿

        // Vector6d error = T_ij.log(); 
        // _error = error;

        // 提取平移和旋转部分
        Eigen::Vector3d t_ij = T_ij.translation();  // j在 i坐标系下的位姿
        Eigen::Matrix3d R_ij = T_ij.rotationMatrix(); // j在 i坐标系下的位姿

        // 计算残差
        Eigen::Matrix3d R_error = R_ij.transpose() * R; // j在 i坐标系下的位姿
        Eigen::Vector3d t_error = (t - t_ij); // j在 i坐标系下的位姿

        _error[0] = t_error.x() / t_var;
        _error[1] = t_error.y() / t_var;
        _error[2] = t_error.z() / t_var;
         
        // 待定
        Eigen::AngleAxisd aa(R_error); //一个旋转表示为一个轴向量和一个角度。
        Eigen::Vector3d axis = aa.axis(); // axis 是一个单位向量，描述了旋转的轴方向
        double angle = aa.angle();// angle 则是旋转的角度（弧度制）
        //这些值用来衡量旋转矩阵 R_error 相对于期望旋转矩阵的偏差，其中 R_var 是用来加权误差的方差参数。以便后续在优化过程中进行误差最小化的计算。
        _error[3] = axis.x() * angle / R_var;
        _error[4] = axis.y() * angle / R_var;
        _error[5] = axis.z() * angle / R_var;
    }

    virtual void linearizeOplus()
    {
        // g2o会自动使用数值微分来计算雅可比矩阵。
        // 通常情况下，除非需要提高性能，否则无需提供解析雅可比矩阵。
        g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexPose,VertexPose>::linearizeOplus();
    }

    virtual bool read(std::istream& is) {}
    virtual bool write(std::ostream& os) const {}

private:
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    double t_var, R_var;
};


// 定义一个继承自g2o::BaseBinaryEdge的边类
// 定义 EdgeSE3Expmap 类，继承自 g2o::BaseBinaryEdge，表示连接两个位姿节点的边
// class EdgeSE3Expmap : public g2o::BaseBinaryEdge<6, Eigen::Isometry3d, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap> {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//     EdgeSE3Expmap() : g2o::BaseBinaryEdge<6, Eigen::Isometry3d, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>() {}

//     // 计算误差，即当前测量值与估计值之间的差异
//     virtual void computeError() {
//         // 获取两个顶点的估计值
//         const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
//         const g2o::VertexSE3Expmap* v2 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);

//         // 估计的变换
//         Eigen::Isometry3d estimate1 = v1->estimate();
//         Eigen::Isometry3d estimate2 = v2->estimate();

//         // 当前测量值
//         Eigen::Isometry3d measured = _measurement;

//         // 计算误差，这里假设误差为估计值与测量值之间的差异
//         _error = (measured.inverse() * (estimate1.inverse() * estimate2)).log();
//     }

//     // 线性化边，计算误差对优化变量的雅可比矩阵
//     virtual void linearizeOplus() {
//         // 获取两个顶点的估计值
//         g2o::VertexSE3Expmap* v1 = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
//         g2o::VertexSE3Expmap* v2 = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);

//         // 估计的变换
//         Eigen::Isometry3d estimate1 = v1->estimate();
//         Eigen::Isometry3d estimate2 = v2->estimate();

//         // 当前测量值
//         Eigen::Isometry3d measured = _measurement;

//         // 计算误差对节点1的雅可比矩阵
//         Eigen::Matrix<double, 6, 6> J1 = g2o::internal::toCompactQuaternion(estimate2.rotation().inverse() * estimate1.rotation()).toRotationMatrix();

//         // 计算误差对节点2的雅可比矩阵
//         Eigen::Matrix<double, 6, 6> J2 = -g2o::internal::toCompactQuaternion(estimate2.rotation().inverse()).toRotationMatrix();

//         // 设置边的线性化雅可比矩阵
//         _jacobianOplusXi = J1;
//         _jacobianOplusXj = J2;
//     }

//     // 读取和写入边的数据
//     virtual bool read(std::istream& is) { return true; }
//     virtual bool write(std::ostream& os) const { return true; }
// };


// 重载 GNSS不带位姿 只有三维点
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
  auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

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
      pose->setFixed(false);
      //if ( i == 0) pose->setFixed( true ); // 第一个点固定为零
      optimizer.addVertex(pose);

      // Edge - GNSS 绝对位移
      GNSSConstraintEdge *edge = new GNSSConstraintEdge(Gnss_enu_i); //GNSS ENU真值 3D-2D是图像1的3d点
      edge->setId(i);
      edge->setVertex(0, pose); // 获取优化第一个节点位姿 没有第二个节点位姿
      //edge->setVertex(1, dynamic_cast<g2o::VertexSBAPointXYZ*>  );  如果未来使用 第二个顶点
      edge->setMeasurement(Gnss_enu_i); //GNSS ENU真值 3D-2D是图像2的像素点
      edge->setInformation(Eigen::Matrix3d::Identity());

      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      //rk->setDelta(sqrt(5.991));
      edge->setRobustKernel(rk);

      optimizer.addEdge(edge);

  }




 
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now(); // 开始计时
  optimizer.initializeOptimization();
  cout << "开始优化"<< endl;
  optimizer.optimize(10);
  cout << "BAy优化结束"<< endl;
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();// 结束计时
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "optimization 10次花费时间: " << time_used.count() << " 秒." << endl;

  // Extract optimized camera poses
  for (size_t i = 0; i < N; ++i) {
      VertexPose* v = static_cast<VertexPose*>(optimizer.vertex(i));

    
      Sophus::SE3d optimized_pose = v->estimate();
      optimized_poses.push_back(optimized_pose);

      // convert to cv::Mat
      // Eigen::Matrix3d R_ = optimized_pose.rotationMatrix();
      // Eigen::Vector3d t_ = optimized_pose.translation();
      // Sophus::SE3d optimized_pose_SE3_Rt(R_, t_);    
      // optimized_poses.push_back(optimized_pose_SE3_Rt);

  
  }
 cout << "数据取出完毕"<<  endl;

}

// 重载 GNSS带位姿
void bundleAdjustment(
  const TrajectoryType &Camera_poses,
  const TrajectoryType  &Gnss_enus,
  TrajectoryType &optimized_poses
  ) 
{


  // 构建图优化，先设定g2o
  //typedef g2o::BlockSolverX BlockSolverType;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
  
  // 梯度下降方法，可以从GN, LM, DogLeg 中选
  auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

  g2o::SparseOptimizer optimizer;     // 图模型
  optimizer.setAlgorithm(solver);   // 设置求解器
  optimizer.setVerbose(true);       // 打开调试输出
 
 
  int N = Gnss_enus.size();// 总对数
  


  // 添加节点 局部待优化的所有位姿
  for (size_t i = 0; i < N; i++) {
      Sophus::SE3d Camera_pose_i = Camera_poses[i];     
      
       // vertex
      VertexPose *pose = new VertexPose(); // camera pose R t
      pose->setId(i);
      pose->setEstimate(Camera_pose_i);
      pose->setFixed(false);
      optimizer.addVertex(pose);
  }
  
  // 相对误差边
  // for (size_t i = 0; i < N-1; i++) {
      
  //     // GNSS 相对位移
  //     Sophus::SE3d T_iw = Gnss_enus[i] ;
  //     Sophus::SE3d T_jw = Gnss_enus[i+1];
  //     Sophus::SE3d T_ij = T_iw.inverse() *  T_jw ; // j在 i坐标系下的位姿
  //     // 提取平移和旋转部分
  //     Eigen::Vector3d t_ij = T_ij.translation();
  //     //Eigen::Quaterniond q_ij = T_ij.rotation();
  //     Eigen::Matrix3d R_ij = T_ij.rotationMatrix();
  //     double t_var=1;
  //     double R_var=1;
    

  //     // 添加边
  //     EdgeRelativeRT* edge = new EdgeRelativeRT(R_ij, t_ij, t_var, R_var);
  //     VertexPose* vi = static_cast<VertexPose*>(optimizer.vertex(i));   
  //     VertexPose* vj = static_cast<VertexPose*>(optimizer.vertex(i+1));
  //     edge->setVertex(0, vi);
  //     edge->setVertex(1, vj);
  //     edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity()); // 设置信息矩阵（协方差）

  //     // 鲁棒核函数 必须加 避免离群噪声点干扰结果
  //     g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
  //     //rk->setDelta(sqrt(5.991));
  //     edge->setRobustKernel(rk);

  //     optimizer.addEdge(edge);
  // }

  /*
  开启相对边
带位姿的均方根误差
真值和原始 RMSE :34.9827
真值和优化 RMSE :0.0720014
优化和原始 RMSE :34.9861
位移的均方根误差
真值和原始 RMSE :34.9827
真值和优化 RMSE :0.00624998
优化和原始 RMSE :34.9831

不开相对边
带位姿的均方根误差
真值和原始 RMSE :34.868
真值和优化 RMSE :0.240703
优化和原始 RMSE :36.9033
位移的均方根误差
真值和原始 RMSE :34.868
真值和优化 RMSE :1.72056e-40
优化和原始 RMSE :34.868

下一步引入srt自动
切 GNSS输入改为 vo 局部和 vo全局
R替换为q
局部BA优化GNSS
全中考虑问题
间歇式引入GNSS 而不是全部
  
  */

  // 绝对误差边
  for (size_t i = 0; i < N; i++) {
      VertexPose* vi = static_cast<VertexPose*>(optimizer.vertex(i));
      Eigen::Vector3d Gnss_enu_i =  Gnss_enus[i].translation();

      // Edge
      GNSSConstraintEdge *edge = new GNSSConstraintEdge(Gnss_enu_i); //GNSS ENU真值 3D-2D是图像1的3d点
      edge->setId(i);
      edge->setVertex(0, vi); // 获取优化第一个节点位姿 没有第二个节点位姿
      //edge->setMeasurement(Gnss_enu_i); //GNSS ENU真值 3D-2D是图像2的像素点 初始化已经送入
      edge->setInformation(Eigen::Matrix3d::Identity());
      
      // 胡函数 防止利群点误差过大干扰整体结果
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      //rk->setDelta(sqrt(5.991));
      edge->setRobustKernel(rk);

      optimizer.addEdge(edge);

  }





 
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now(); // 开始计时
  optimizer.initializeOptimization();
  cout << "开始优化"<< endl;
  optimizer.optimize(10);
  cout << "BAy优化结束"<< endl;
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();// 结束计时
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "optimization 10次花费时间: " << time_used.count() << " 秒." << endl;

  // Extract optimized camera poses
  for (size_t i = 0; i < N; ++i) {
      VertexPose* v = static_cast<VertexPose*>(optimizer.vertex(i));

    
      Sophus::SE3d optimized_pose = v->estimate();
      optimized_poses.push_back(optimized_pose);

      // convert to cv::Mat
      // Eigen::Matrix3d R_ = optimized_pose.rotationMatrix();
      // Eigen::Vector3d t_ = optimized_pose.translation();
      // Sophus::SE3d optimized_pose_SE3_Rt(R_, t_);    
      // optimized_poses.push_back(optimized_pose_SE3_Rt);

  
  }
 cout << "数据取出完毕"<<  endl;

}
#endif 