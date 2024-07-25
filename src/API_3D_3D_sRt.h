
#ifndef API_3D_3D_sRt_CPP
#define API_3D_3D_sRt_CPP



#include "API_dataAndFileIO.h"

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <random>   
 
  
  
using namespace Eigen;
using namespace std;
 
 


 

  
// 核心公式 3d-3d sbd分解计算
/*
 
输入点云 一般是SLAM vo的enu坐标点  vector<Vector3d>& source_points,
目标点云 GNSS的enu坐标点             const vector<Vector3d>& target_points,
输出
 尺度             double& scale,
 旋转             Matrix3d& R,
 位移             Vector3d& t
 
 结果使用
   target_points=scale*R*source_points+t
 
*/
void API_ICP_3D_3D_sRt(const vector<Vector3d>& source_points,
              const vector<Vector3d>& target_points,
              double& scale, Matrix3d& R, Vector3d& t) {
  
    int N = source_points.size();
  
    // 计算质心
    Vector3d centroid_source = Vector3d::Zero();
    Vector3d centroid_target = Vector3d::Zero();
    for(int i=0;i<N;i++){
        centroid_source += source_points[i];
        centroid_target += target_points[i];
    }
    centroid_source /= N;
    centroid_target /= N;
 
    /// 中心化点云
    vector<Vector3d> centered_source_points (N);
    vector<Vector3d> centered_target_points (N);
    for(int i=0;i<N;i++){
        centered_source_points[i]= source_points[i] - centroid_source;
        centered_target_points[i]= target_points[i] - centroid_target;
 
    }
      
    // 计算尺度因子 s 方法2
    // 计算点云的每个点的范数平方
 
    double sum_source = 0.0;
    double sum_target = 0.0;
    for (size_t i = 0; i < N; ++i) {
        sum_source += centered_source_points[i].norm(); // norm---sqrt(x^2+y^2+z^2)
        sum_target += centered_target_points[i].norm();
    }
    double scale_ = sum_target /sum_source;
    scale = scale_;
 
    // vector<double> source_norm = computeNormSquared(centered_source_points); // 每一行的 source_norm[i]= x^2+y^2+z^2
    // vector<double> target_norm = computeNormSquared(centered_target_points);
    // double sum_source = 0.0;
    // double sum_target = 0.0;
    // for (size_t i = 0; i < N; ++i) {
    //     sum_source += sqrt(source_norm[i]) ;
    //     sum_target += sqrt(target_norm[i]) ;
    // }
    // double scale_ = sum_target /sum_source;
    // scale = scale_;
    // 计算尺度因子 s 方法1
    // double trace = 0.0;
    // for (int i = 0; i < N; ++i) {
    //     trace += centered_target_points[i].transpose() * R * centered_source_points[i];
    // }
    // double src_norm = 0.0;
    // for (int i = 0; i < N; ++i) {
    //     src_norm += centered_source_points[i].squaredNorm();
    // }
    // scale = trace / src_norm;
 
 
 
    // 计算协方差矩阵 H
    Matrix3d H = Matrix3d::Zero();
    
      
    for (int i = 0; i < N; ++i) {
        // 是否乘上scale没有区别 centered_target_points
        H += centered_target_points[i] * centered_source_points[i].transpose();
        //H += scale*centered_source_points[i] * centered_target_points[i].transpose(); 
    }
      
 
    // 使用奇异值分解H SVD
    JacobiSVD<Matrix3d> svd(H, ComputeFullU | ComputeFullV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    
 
    // 计算旋转矩阵R
    R = U*  V.transpose() ; // src到dis点变换关系 对应后面计算t
    
     
 
    if (U.determinant() * V.determinant() < 0) // R的行列式<0 取反
    {
        for (int x = 0; x < 3; ++x)
        {
            U(x, 2) *= -1;
        }
        R = U*  V.transpose() ;
    }
     
    //R = V * U.transpose();
    // 计算平移向量t   两个质心算位移
    t = centroid_target - scale * R * centroid_source;
}
  
 
 
void API_ransac_ICP_3D_3D_sRt_inliner_sR(const vector<Vector3d>& source_points,
                     const vector<Vector3d>& target_points,
                     const int num_iterations,        //ransac随机抽取验证次数
                     const double error_threshold,    // 误差阈值 3         
                     
                     double& best_scale, Matrix3d& best_R, Vector3d& best_t) {
  
   
 
    // 1 数据准备
    int best_inlier_count = 0; // 最大内点数
  
    int N = source_points.size();
 
    //cout<<  "总点数  " << N << endl;
  
    // 全体点计算质心
    Vector3d centroid_source = Vector3d::Zero();
    Vector3d centroid_target = Vector3d::Zero();
    for(int i=0;i<N;i++){
        centroid_source += source_points[i];
        centroid_target += target_points[i];
    }
    centroid_source /= N;
    centroid_target /= N;
 
    /// 全体点中心化点云
    vector<Vector3d> centered_source_points (N);
    vector<Vector3d> centered_target_points (N);
    for(int i=0;i<N;i++){
        centered_source_points[i]= source_points[i] - centroid_source;
        centered_target_points[i]= target_points[i] - centroid_target;
 
    }
      
 
    // 3使用ransc 找出内点剔除外点，计算更加准确的sRt
    // 使用 std::random_device 生成真随机数种子
    std::random_device rd;
    // 使用梅森旋转算法引擎 mt19937，以 rd() 作为种子
    std::mt19937 gen(rd());
    // 定义一个均匀整数分布，范围是[1, 100]
    //std::uniform_int_distribution<int> dist(1, 100);
    uniform_int_distribution<int> dist(0, N - 1);
    // 生成随机数
    //int random_number = dist(gen);
  
      
    // RANSAC loop num_iterations 迭代次数
    for (int iteration = 0; iteration < num_iterations; ++iteration) {
        // Randomly select 3 points to form a minimal sample
  
        vector<Vector3d> source_points_temp;
        vector<Vector3d> target_points_temp;
  
        for (int num=0;num<4;num++){ // 最少3个点  取4个点
  
            int idx = dist(gen);
            source_points_temp.push_back(source_points[idx]);
            target_points_temp.push_back(target_points[idx]);
  
        }
  
        // 本次随机计算结果  Perform ICP with these initial parameters
        double scale;
        Matrix3d R;
        Vector3d t;
        API_ICP_3D_3D_sRt(source_points_temp,target_points_temp, scale, R, t);
  
        // Compute error with all points and count inliers
        int inlier_count = 0;
          
 
        vector<Vector3d> source_points_inliner;
        vector<Vector3d> target_points_inliner;
 
         
        
        for (int i = 0; i < N; ++i) {  
           Vector3d centered_source2target_points_i = scale*R*centered_source_points[i];//  使用本次随机抽取样本计算的sRt结果 ，对所有原始点的去中心点变换到目标点的中心点 尺度和旋转位置
           double source_norm_i=centered_source2target_points_i.norm(); //去中心化的点长2范数度 norm---sqrt(x^2+y^2+z^2)
           double target_norm_i=centered_target_points[i].norm(); //去中心化的点长2范数度  norm---sqrt(x^2+y^2+z^2)
 
           double error = abs(source_norm_i  - target_norm_i);//计算误差 如果目标点是GNSS的enu坐标 单位是米
 
           if (error < error_threshold) {// 取3米
                inlier_count++; // 记录本次内点数目
                // 找出内点集合
                source_points_inliner.push_back(source_points[i]);
                target_points_inliner.push_back(target_points[i]);
           }
 
        }
 
         
        // 如果本次内点多余历史最好内点数目 本次结果作为最好结果
        if (inlier_count > best_inlier_count) {
 
            API_ICP_3D_3D_sRt(source_points_inliner,target_points_inliner, scale, R, t);// 使用内点再次计算最优的srt
 
            best_inlier_count = inlier_count;
            best_scale = scale;
            best_R = R;
            best_t = t;
 
            if(iteration==num_iterations-1) cout<<"最大内点数：" <<inlier_count <<endl;
        }
    }
 
     
 
 
 
}




#endif 
  
 