#ifndef error_calculate_CPP
#define error_calculate_CPP


#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <random>   
 
#include "API_dataAndFileIO.h"  
  
using namespace Eigen;
using namespace std;
 
double cal_rmse( TrajectoryType &groundtruth,TrajectoryType &estimated){


 // // compute rmse
  double rmse = 0;
  for (size_t i = 0; i < estimated.size(); i++) {
    Sophus::SE3d p1 = estimated[i], p2 = groundtruth[i];
    double error = (p2.inverse() * p1).log().norm();
    rmse += error * error;

  }
  rmse = rmse / double(estimated.size());
  rmse = sqrt(rmse);
  
  //cout << "RMSE = " << rmse << endl;
  return rmse;
}


double cal_rmse_t_se3_se3( TrajectoryType &groundtruth,TrajectoryType &estimated){


 // // compute rmse
  double rmse = 0;
  for (size_t i = 0; i < estimated.size(); i++) {
    Sophus::SE3d p1 = estimated[i], p2 = groundtruth[i];
    Eigen::Vector3d t_1 = p1.translation();
    Eigen::Vector3d t_2 = p2.translation();

    Eigen::Vector3d error  = (t_2- t_1);
    rmse += error.squaredNorm();//用 squaredNorm() 计算误差平方和 norm平方根

  }
  rmse = rmse / double(estimated.size());
  rmse = sqrt(rmse);
  
  //cout << "RMSE = " << rmse << endl;
  return rmse;
}

double cal_rmse_t_v3d_se3(  vector<Vector3d>  &groundtruth,TrajectoryType &estimated){


 // // compute rmse
  double rmse = 0;
  for (size_t i = 0; i < estimated.size(); i++) {
    //Sophus::SE3d p1 = estimated[i];
    //Eigen::Vector3d p2 = groundtruth[i];
    Eigen::Vector3d t_1 = estimated[i].translation();
    Eigen::Vector3d t_2 = groundtruth[i];

    Eigen::Vector3d error  = (t_2- t_1);
    rmse += error.squaredNorm();//用 squaredNorm() 计算误差平方和 norm平方根

  }
  rmse = rmse / double(estimated.size());
  rmse = sqrt(rmse);
  
  //cout << "RMSE = " << rmse << endl;
  return rmse;
}

 
 //  变换点和目标点计算误差
double computeError(const vector<Vector3d>& source_points,
                    const vector<Vector3d>& target_points,
                    const double scale, const Matrix3d& R, const Vector3d& t) {
    // Compute transformation error using point-to-point distances
    double total_error = 0.0;
    double Mean_error = 0.0;
    double max_error = 0.0;
    double min_error = 9999;
  
    int num_points = source_points.size();
 
    vector<double> error_;
    for (int i = 0; i < num_points; ++i) {
        Vector3d transformed_point = scale * R * source_points[i] + t;
 
        double cur_error =(transformed_point - target_points[i]).norm();
        error_.push_back(cur_error);
 
    }
 
 
    for(auto cur_error : error_){
            
        if(cur_error>max_error){max_error=cur_error;}
 
        if(cur_error<min_error){min_error=cur_error;}
 
        total_error = total_error+cur_error;  //向量的二范数
 
    }
 
    Mean_error=total_error / num_points;
 
    cout<<"Mean_error  " <<Mean_error << "   max_error   "<< max_error<< "   min_error   "<< min_error<< " 总点数  " << num_points <<endl;
    return Mean_error;  // Mean error
}
 

  
// 计算点云的每个点的范数平方 -- 用于计算S尺度
vector<double> computeNormSquared(const vector<Vector3d>& cloud) {
    vector<double> norms;
 
    for (const auto& point : cloud) {
        double normSquared = 0.0;
        for (int i = 0; i < point.size(); ++i)
        {
            normSquared += point[i] * point[i];
        }
        norms.push_back(normSquared);
    }
     
     
    return norms;
}


#endif 