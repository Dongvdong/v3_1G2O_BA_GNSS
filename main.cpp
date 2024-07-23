#ifndef MAIN_CPP
#define MAIN_CPP


#include "API_draw.h"
#include "API_G2O_BA.h"

#include "API_3D_3D.h"

using namespace std;



void Test1(){



  string groundtruth_file = "../data/groundtruth.txt";
  string estimated_file = "../data/estimated.txt";

  TrajectoryType groundtruth_se3d = ReadTrajectory(groundtruth_file);
  Point_txyz_List groundtruth = Read_Point_txyz(groundtruth_file);

  TrajectoryType estimated = ReadTrajectory(estimated_file);
  
 
  assert(!groundtruth.empty() && !estimated.empty());
  assert(groundtruth.size() == estimated.size());

  cout<<"GNSS 数据总数  " <<groundtruth.size()  << endl;
  
  cout<<"estimated 数据总数  " <<estimated.size()  << endl;

  TrajectoryType optimized_pose(groundtruth.size());

  //bundleAdjustment(estimated,groundtruth,optimized_pose);

  bundleAdjustment(estimated,groundtruth_se3d,optimized_pose);


  
  // double error_1 =cal_rmse(groundtruth_se3d,estimated);
  // double error_2 =cal_rmse(groundtruth_se3d,optimized_pose);
  // double error_3 = cal_rmse(optimized_pose,estimated);

  double error_1 =cal_rmse(estimated,groundtruth_se3d);
  double error_2 =cal_rmse(optimized_pose,groundtruth_se3d);
  double error_3 = cal_rmse(optimized_pose,estimated);
  cout << "真值和原始 RMSE :" << error_1 << endl;
  cout << "真值和优化 RMSE :" << error_2 << endl;
  cout << "优化和原始 RMSE :" << error_3 << endl;

  DrawTrajectory3(groundtruth_se3d, estimated, optimized_pose);

  

}


int main(int argc, char **argv) {



  //1-1 创建测试点 调用点代替
  vector<Vector3d> source_points;
  vector<Vector3d> target_points;


  // 1-2 设置变换矩阵
  Matrix3d currentR;
  Vector3d currentT;
  double currentScale=1;
  currentR<<1,0,0,0,1,0,0,0,1;// 单位阵
  //currentR<<0,-1,0,1,0,0,0,0,1; // 旋转90度
  currentT<<0,0,0;
  // 1-3 获取随机测试点
  Get_TestValue(source_points,target_points,currentScale,currentR,currentT);



  TrajectoryType source_points_T,target_points_T;
  TrajectoryType optimized_pose;

  for(int i=0; i<source_points.size();i++ ){

    source_points_T.push_back(Sophus::SE3d(currentR,source_points[i]));
    target_points_T.push_back(Sophus::SE3d(currentR,target_points[i]));
 
  }


  cout<<"GNSS 数据总数  " <<source_points_T.size()  << endl;
  
  cout<<"estimated 数据总数  " <<target_points_T.size()  << endl;

  bundleAdjustment(target_points_T,source_points_T,optimized_pose);


  double error_1 =cal_rmse(source_points_T,target_points_T);
  double error_2 =cal_rmse(source_points_T,optimized_pose);
  double error_3 = cal_rmse(optimized_pose,target_points_T);
  double error_4 = cal_rmse(target_points_T,optimized_pose);
  cout << "带位姿的均方根误差" <<endl;
  cout << "真值和原始 RMSE :" << error_1 << endl;
  cout << "真值和优化 RMSE :" << error_2 << endl;
  cout << "优化和原始 RMSE :" << error_3 << endl;
  cout << "原始和优化 RMSE :" << error_4 << endl;

  DrawTrajectory3(source_points_T, target_points_T, optimized_pose);

  

  // DrawTrajectory(groundtruth, estimated);
  return 0;
}




#endif 
