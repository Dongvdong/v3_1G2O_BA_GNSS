#ifndef MAIN_CPP
#define MAIN_CPP

#include "src/API_dataAndFileIO.h"  // 数据结构定义 文件数据读取 测试数据产生
#include "src/API_draw.h"           // 画图轨迹
#include "src/API_G2O_BA.h"         // 使用G2O优化 GNSS和vo位姿
#include "src/API_3D_3D_sRt.h"      // 使用3D-3D svd分解求解 sRt
#include "src/API_ error_calculate.h"  // 计算均方根误差等


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



  TrajectoryType source_points_T; // 真值
  TrajectoryType target_points_T; // 噪声值 噪声+超级离群点模拟
  TrajectoryType optimized_pose;  // 优化后的值

  for(int i=0; i<source_points.size();i++ ){

    Matrix3d currentR2;
    //currentR2<<0,-1,0,1,0,0,0,0,1;// 旋转90度
    currentR2<<1,0,0,0,1,0,0,0,1;// 旋转90度

    source_points_T.push_back(Sophus::SE3d(currentR,source_points[i]));
    target_points_T.push_back(Sophus::SE3d(currentR2,target_points[i]));
 
  }

  /*

  误差函数没有变换矩阵导致计算过大 
  currentR<<1,0,0,0,1,0,0,0,1;// 单位阵
  currentR2<<0,-1,0,1,0,0,0,0,1;// 旋转90度
  带位姿的均方根误差
  真值和原始 RMSE :37.9145
  真值和优化 RMSE :1.60159  
  优化和原始 RMSE :37.1613
  位移的均方根误差
  真值和原始 RMSE :35.1885
  真值和优化 RMSE :3.75842e-40
  优化和原始 RMSE :35.1885

  没有 相对误差优化 只有绝对误差
  currentR<<1,0,0,0,1,0,0,0,1;// 单位阵
  currentR2<<1,0,0,0,1,0,0,0,1;// 旋转90度
  带位姿的均方根误差
  真值和原始 RMSE :35.3401
  真值和优化 RMSE :0.234842
  优化和原始 RMSE :37.2973
  位移的均方根误差
  真值和原始 RMSE :35.3401
  真值和优化 RMSE :4.49181e-40
  优化和原始 RMSE :35.3401


  增加了 相对误差优化
  currentR<<1,0,0,0,1,0,0,0,1;// 单位阵
  currentR2<<1,0,0,0,1,0,0,0,1;// 旋转90度
  数据取出完毕
  带位姿的均方根误差
  真值和原始 RMSE :35.2128
  真值和优化 RMSE :0.0829628
  优化和原始 RMSE :35.214
  位移的均方根误差
  真值和原始 RMSE :35.2128
  真值和优化 RMSE :0.00526571
  优化和原始 RMSE :35.2129

  */


  cout<<"GNSS 数据总数  " <<source_points_T.size()  << endl;
  cout<<"estimated 数据总数  " <<target_points_T.size()  << endl;

  bundleAdjustment(target_points_T,source_points_T,optimized_pose);


  double error_1 =cal_rmse(source_points_T,target_points_T);
  double error_2 =cal_rmse(source_points_T,optimized_pose);
  double error_3 = cal_rmse(optimized_pose,target_points_T);
  cout << "带位姿的均方根误差" <<endl;
  cout << "真值和原始 RMSE :" << error_1 << endl;
  cout << "真值和优化 RMSE :" << error_2 << endl;
  cout << "优化和原始 RMSE :" << error_3 << endl;

   error_1 =cal_rmse_t(source_points_T,target_points_T);
   error_2 =cal_rmse_t(source_points_T,optimized_pose);
   error_3 = cal_rmse_t(optimized_pose,target_points_T);
  cout << "位移的均方根误差" <<endl;
  cout << "真值和原始 RMSE :" << error_1 << endl;
  cout << "真值和优化 RMSE :" << error_2 << endl;
  cout << "优化和原始 RMSE :" << error_3 << endl;


  DrawTrajectory3(source_points_T, target_points_T, optimized_pose);

  

  // DrawTrajectory(groundtruth, estimated);
  return 0;
}




#endif 
