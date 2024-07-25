#ifndef MAIN_CPP
#define MAIN_CPP


#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <unistd.h> // For sleep function
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <vector>
using namespace std;
using namespace Eigen;




#include "src/API_dataAndFileIO.h"  // 数据结构定义 文件数据读取 测试数据产生
#include "src/API_draw.h"           // 画图轨迹
#include "src/API_G2O_BA.h"         // 使用G2O优化 GNSS和vo位姿
#include "src/API_3D_3D_sRt.h"      // 使用3D-3D svd分解求解 sRt
#include "src/API_ error_calculate.h"  // 计算均方根误差等




// 一次性读取画图
void Test1(){


  // string groundtruth_file = "../data/groundtruth.txt";
  // string estimated_file = "../data/estimated.txt";

  // TrajectoryType groundtruth_se3d = ReadTrajectory(groundtruth_file);
  // Point_txyz_List groundtruth = Read_Point_txyz(groundtruth_file);

  // TrajectoryType estimated = ReadTrajectory(estimated_file);
  
 
  // assert(!groundtruth.empty() && !estimated.empty());
  // assert(groundtruth.size() == estimated.size());


  //1-1 创建测试点 调用点代替
  vector<Vector3d> source_points;
  vector<Vector3d> target_points;



    // 1-2 设置变换矩阵
  Matrix3d RelativeR;
  Vector3d Relativet;
  double currentScale=1;
  RelativeR<<1,0,0,0,1,0,0,0,1;// 单位阵
  //RelativeR<<0,-1,0,1,0,0,0,0,1; // 旋转90度
  Relativet<<0,0,0;
  // 1-3 获取随机测试点
  Get_TestValue(source_points,target_points,currentScale,RelativeR,Relativet);



  TrajectoryType source_points_T; // 真值
  TrajectoryType target_points_T; // 噪声值 噪声+超级离群点模拟
  TrajectoryType optimized_pose;  // 优化后的值

  for(int i=0; i<source_points.size();i++ ){

    Matrix3d RelativeR2;
    //RelativeR2<<0,-1,0,1,0,0,0,0,1;// 旋转90度
    RelativeR2<<1,0,0,0,1,0,0,0,1;// 旋转90度

    source_points_T.push_back(Sophus::SE3d(RelativeR,source_points[i]));
    target_points_T.push_back(Sophus::SE3d(RelativeR2,target_points[i]));
 
  }

 


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

  

}





// 主线程生成3D点的函数
void Pub_data_GNSS_VO(vector<Vector3d> &GNSS_points,TrajectoryType &VO_points_T,mutex &queueMutex) {


    vector<Vector3d> source_points;
    vector<Vector3d> target_points;


    generatePoints(source_points,target_points); //产生测试点

    cout<<"GNSS(真值)       数据总数  " <<source_points.size()  << endl;
    cout<<"VO (真值+噪声）   数据总数  " <<target_points.size()  << endl;

    // 使用测试点产生 位姿
    for(int i=0; i<source_points.size();i++ ){

      double yaw = 0;    // 绕Z轴的角度
      double pitch = 0;  // 绕Y轴的角度
      double roll = 0;   // 绕X轴的角度

      Eigen::Matrix3d VO_points_R;
      VO_points_R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

      queueMutex.lock();
      GNSS_points.push_back(source_points[i]);
      VO_points_T.push_back(Sophus::SE3d(VO_points_R,target_points[i]));
      queueMutex.unlock();

      cout<<"发布位姿对 " << i  << endl;


      sleep(1); // 每秒生成一个点
  
    }


}




int main(int argc, char **argv) {

 
  // 用于存储3D点的队列和互斥锁
  mutex queueMutex;// 锁
  vector<Vector3d> GNSS_points; // 真值
  TrajectoryType VO_points_T; // 噪声值 噪声+超级离群点模拟
  TrajectoryType VO_optimized_T;  // 优化后的值



  // 初始化VO-GNSS变换矩阵单位阵
  Eigen::Matrix4d T_vo_to_GNSS_ENU = Eigen::Matrix4d::Identity();
  T_vo_to_GNSS_ENU.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // 设置旋转部分为单位矩阵
  T_vo_to_GNSS_ENU.block<3, 1>(0, 3) << 0, 0, 0; // 设置平移部分为 (0, 0, 0)


  
  std::thread Pub_data_thread([&GNSS_points, &VO_points_T, &queueMutex]() {Pub_data_GNSS_VO(GNSS_points,VO_points_T,queueMutex);});
  std::thread Display_thread([&GNSS_points, &VO_points_T,&VO_optimized_T,&queueMutex]() {displayline_3_thread(GNSS_points,VO_points_T,VO_optimized_T,queueMutex);});
  
  // 等待线程结束
  Pub_data_thread.join();
  Display_thread.join();




  // bundleAdjustment(target_points_T,source_points_T,optimized_pose);


  // double error_1 =cal_rmse(source_points_T,target_points_T);
  // double error_2 =cal_rmse(source_points_T,optimized_pose);
  // double error_3 = cal_rmse(optimized_pose,target_points_T);
  // cout << "带位姿的均方根误差" <<endl;
  // cout << "真值和原始 RMSE :" << error_1 << endl;
  // cout << "真值和优化 RMSE :" << error_2 << endl;
  // cout << "优化和原始 RMSE :" << error_3 << endl;

  //  error_1 =cal_rmse_t(source_points_T,target_points_T);
  //  error_2 =cal_rmse_t(source_points_T,optimized_pose);
  //  error_3 = cal_rmse_t(optimized_pose,target_points_T);
  // cout << "位移的均方根误差" <<endl;
  // cout << "真值和原始 RMSE :" << error_1 << endl;
  // cout << "真值和优化 RMSE :" << error_2 << endl;
  // cout << "优化和原始 RMSE :" << error_3 << endl;


  // DrawTrajectory3(source_points_T, target_points_T, optimized_pose);

  

  // DrawTrajectory(groundtruth, estimated);
  return 0;
}




#endif 
