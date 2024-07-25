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







// 主线程生成3D点的函数
void Pub_data_GNSS_VO(
  vector<Vector3d> &GNSS_points,
  TrajectoryType &VO_points_T,
  mutex &queueMutex
  ) {


    vector<Vector3d> source_points;
    vector<Vector3d> target_points;


    //generatePoints(source_points,target_points); //产生测试点

   // 1-2 设置source_points到 target_points变换矩阵
    Matrix3d RelativeR;
    Vector3d Relativet;
    Relativet<<0,0,10;
    double currentScale=2;
    //RelativeR<<1,0,0,0,1,0,0,0,1;// 单位阵
    //RelativeR<<0,-1,0,1,0,0,0,0,1; // 旋转90度
    double yaw = 0;    // 绕Z轴的角度
    double pitch = 90;  // 绕Y轴的角度
    double roll = 0;   // 绕X轴的角度
    RelativeR = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    cout<< "RelativeR "<<RelativeR <<endl;


    
    // 1-3 获取随机测试点
    Get_TestValue(source_points,target_points,currentScale,RelativeR,Relativet);



    cout<<"GNSS(真值)       数据总数  " <<source_points.size()  << endl;
    cout<<"VO (真值+噪声）   数据总数  " <<target_points.size()  << endl;

    // 使用测试点产生 位姿
    for(int i=0; i<source_points.size();i++ ){
      
      // 设置自身的旋转
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
      
      cout<<"发布位姿对 " << i   <<endl;


      sleep(1); // 每秒生成一个点
  
    }


}

void bundleAdjustment_GNSS_thread(
  const vector<Vector3d>  &Gnss_enus,
  const TrajectoryType &Camera_poses,
  TrajectoryType &optimized_poses,
  mutex &queueMutex,
  Eigen::Matrix4d &T_vo_to_GNSS_ENU
  
){

cout<< "优化线程开启"<<endl;
while (1){
 
    cout<< "GNSS新数据到来"<<endl;
    if(Gnss_enus.size()>=3){

      double s;
      Matrix3d R;
      Vector3d t;  
      vector<Vector3d> source_points;
      vector<Vector3d> target_points;
      for(int i=0; i<Gnss_enus.size();i++){
          source_points.push_back(Camera_poses[i].translation());
          target_points.push_back(Gnss_enus[i]);
      }
      
      int N=Gnss_enus.size();
      int num_combinations = N * (N - 1) / 2; // 随机挑选2个点验证 一共多少次
      if(num_combinations>1000) num_combinations=1000;
      

      API_ransac_ICP_3D_3D_sRt_inliner_sR(source_points,target_points,
                            num_combinations,        //ransac随机抽取验证次数
                            3,          // 误差阈值 3         
                            s, R, t) ;
          
    
        T_vo_to_GNSS_ENU.block<3, 3>(0, 0) = s * R;
        T_vo_to_GNSS_ENU.block<3, 1>(0, 3) = t;

        bundleAdjustment_GNSS(Gnss_enus,Camera_poses,optimized_poses,queueMutex,T_vo_to_GNSS_ENU);
      
    }

    else if(Gnss_enus.size()>=99){

     break;

    }
    sleep(3); // 每秒生成一个点
  }



}


int main(int argc, char **argv) {

 
  // 用于存储3D点的队列和互斥锁
  mutex queueMutex;// 锁
  vector<Vector3d> GNSS_points; // 真值
  TrajectoryType VO_points_T; // 噪声值 噪声+超级离群点模拟
  TrajectoryType VO_optimized_T;  // 优化后的值
  bool newGnss=0;




 // 初始化VO-GNSS变换矩阵单位阵  和数据构造矩阵是反着的  构造矩阵是用GNSS真值构造的vo
  Eigen::Matrix4d T_vo_to_GNSS_ENU = Eigen::Matrix4d::Identity();


  
  std::thread Pub_data_thread([&GNSS_points, &VO_points_T, &queueMutex]() {
    Pub_data_GNSS_VO(GNSS_points,VO_points_T,queueMutex);});
  std::thread Display_thread([&GNSS_points, &VO_points_T,&VO_optimized_T,&queueMutex]() {
    displayline_3_thread(GNSS_points,VO_points_T,VO_optimized_T,queueMutex);});
  std::thread optimized_Gnss_thread([&GNSS_points, &VO_points_T,&VO_optimized_T,&queueMutex,&T_vo_to_GNSS_ENU]() {
    bundleAdjustment_GNSS_thread(GNSS_points,VO_points_T,VO_optimized_T,queueMutex,T_vo_to_GNSS_ENU);});

  // 等待线程结束
  Pub_data_thread.join();
  Display_thread.join();
  optimized_Gnss_thread.join();

 




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
