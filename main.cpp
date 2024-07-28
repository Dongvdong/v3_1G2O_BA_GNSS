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


//https://www.cnblogs.com/hardjet/p/11708202.html




// 线程1 发布数据
void Pub_data_GNSS_VO(
  vector<Vector3d> &GNSS_points,
  TrajectoryType &VO_points_T,
  bool &newGnss,
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

      newGnss=1;
   
      cout<<"线程1 发布位姿对 " << i  << " newGnss发送  "<<newGnss <<endl;


      sleep(1); // 每秒生成一个点
  
    }
    
   


}



void bundleAdjustment_GNSS_thread_base(

  vector<Vector3d>  &Gnss_enus, // 真值
  TrajectoryType &Camera_poses,       // 等待优化数值
  TrajectoryType &optimized_poses,    // 优化数值
  bool &newGnss, // 新GNSS到达标志位
  Eigen::Matrix4d &T_vo_to_GNSS_ENU, // 变换关系
  mutex &queueMutex
  
  
){

cout<< "优化线程开启"<<endl;


int intail_srt=0;
while (1){


    
    //cout<< "线程2  newGnss "<<  newGnss <<endl;
    sleep(0.0000001); // 每秒生成一个点
    
    if( newGnss==1 && Gnss_enus.size()>=3){

      cout<< "GNSS新数据到来且大于3"<<endl;

      queueMutex.lock(); 
      
      newGnss=0;// 

      // 使用上次优化数值 + 本次原始数值 和 真数值 更新sRt 没有相对边 RTK足够准确
      // 上次优化数值 拉低误差
      // 本次原始数值 假设前期sRt预测有误差 新的原始值来回来 新原始数值本身误差被 被函数和sRt 3m误差剔除
      vector<Vector3d> source_points;
      vector<Vector3d> target_points;
      for(int i=0; i<Gnss_enus.size();i++){
          source_points.push_back(Camera_poses[i].translation());
          target_points.push_back(Gnss_enus[i]);
      }
      


      double s;
      Matrix3d R;
      Vector3d t;  
      API_ransac_ICP_3D_3D_sRt_inliner_sR(source_points,target_points,
                            1000,        //ransac随机抽取验证次数
                            3,          // 误差阈值 3         
                            s, R, t) ;
      T_vo_to_GNSS_ENU.block<3, 3>(0, 0) = s * R;
      T_vo_to_GNSS_ENU.block<3, 1>(0, 3) = t;
    

      bundleAdjustment_GNSS(Gnss_enus,Camera_poses,optimized_poses,queueMutex,T_vo_to_GNSS_ENU);

      double error_1,error_2,error_3;
      error_1 =cal_rmse_t_se3_se3(optimized_poses,Camera_poses);
      cout << "===========" <<endl;
      cout << "优化和原始 RMSE :" << error_1 << endl;

      for(int i=0; i<Gnss_enus.size();i++){
          Camera_poses[i]=optimized_poses[i];
      }

      


      for(int i=0;i<optimized_poses.size();i++){

  
            Eigen::Matrix4d optimized_poses_ = optimized_poses[i].matrix();     //  optimized_poses 如果是错的 后面srt也是错的
            Matrix3d R_ = Eigen::Matrix3d::Identity();
            Vector3d t_ = optimized_poses_.block<3, 1>(0, 3) ;

            Eigen::Vector4d t_cw_4(t_(0), t_(1), t_(2), 1);
            Eigen::Vector4d t_cw_4_out= T_vo_to_GNSS_ENU*t_cw_4;
            Eigen::Vector3d t_cw_3(t_cw_4_out(0)/t_cw_4_out(3), t_cw_4_out(1)/t_cw_4_out(3), t_cw_4_out(2)/t_cw_4_out(3));

            Sophus::SE3d SE3_Rt(R_, t_cw_3);     

            optimized_poses[i]=SE3_Rt;
        }

      queueMutex.unlock(); 


      // double error_1,error_2,error_3;
      // error_1 =cal_rmse_t(optimized_poses,Camera_poses);
      error_2 =cal_rmse_t_v3d_se3(Gnss_enus,optimized_poses);
      //error_3 = cal_rmse_t(Gnss_enus,Camera_poses);
      //cout << "位移的均方根误差" <<endl;
      //cout << "真值和原始 RMSE :" << error_1 << endl;
      cout << "真值和优化 RMSE :" << error_2 << endl;
      //cout << "优化和原始 RMSE :" << error_3 << endl;
      
      //  固定GNSS优化的节点  优化其他边
    }

    else if(Gnss_enus.size()>=100){

      break;

    }
     // sleep(3); // 每秒生成一个点
  }// while



}
/*
=========== 1-2 优化不更新原始 初始化：3原始点初始化算srt  更新：1 优化点不更新原始点  2上原始点和真值计算 srt
优化和原始 RMSE :35.5493
真值和优化 RMSE :0.427039
不开启相对边
优化和原始 RMSE :35.2253
真值和优化 RMSE :3.00064e-15


=========== 1-1 最好 初始化：3原始点初始化算srt  更新：1 优化点更新原始点  2上次数值和真值计算 srt
开启相对边  (VO 相对不准 RTK数据  除非 VO准 GNSS局部不准)
优化和原始 RMSE :2.66917
真值和优化 RMSE :0.0340471

不开启相对边 + 原始数值和真数值更新srt
优化和原始 RMSE :0.75204
真值和优化 RMSE :3.51862e-15
优化和原始 RMSE :0.478324
真值和优化 RMSE :2.75219e-15
优化和原始 RMSE :0.645991
真值和优化 RMSE :2.86205e-15

不开启相对边 + 上次优化数值+本次原始值和真数值更新srt
优化和原始 RMSE :0.545418
真值和优化 RMSE :3.84633e-14
优化和原始 RMSE :0.366477
真值和优化 RMSE :1.05313e-13

不开启相对边 + 上次优化数值+真数值更新srt
优化和原始 RMSE :2.9509
真值和优化 RMSE :7.44218e-14
优化和原始 RMSE :4.07444
真值和优化 RMSE :6.99925e-14

优化和原始 RMSE :0.423359
真值和优化 RMSE :5.08589e-15


=========== 3-1 最好 初始化：3原始点初始化算srt  更新：1 优化点更新原始点  2优化数值和真值计算 srt
优化和原始 RMSE :0.716698
真值和优化 RMSE :0.00214652
优化和原始 RMSE :1.22067
真值和优化 RMSE :0.00226501
不开启相对边
优化和原始 RMSE :0.699512
真值和优化 RMSE :1.30706e-13
优化和原始 RMSE :1.94147
真值和优化 RMSE :4.56362e-14
优化和原始 RMSE :1.94147
真值和优化 RMSE :4.56362e-14

=========== 3-2 中等  初始化：3原始点初始化算srt  更新：1优化点不更新原始点 2 优化数值和真值计算 srt
优化和原始 RMSE :197.623
真值和优化 RMSE :0.00383256

=========== 3-3 最糟糕  初始化：3原始点初始化算srt  更新：1 优化点不更新原始点  2原始点和真值计算 srt
优化和原始 RMSE :35.2427
真值和优化 RMSE :0.426462

      
        */

void bundleAdjustment_GNSS_thread(
  vector<Vector3d>  &Gnss_enus, // 真值
  TrajectoryType &Camera_poses,       // 等待优化数值
  TrajectoryType &optimized_poses,    // 优化数值
  bool &newGnss, // 新GNSS到达标志位
  Eigen::Matrix4d &T_vo_to_GNSS_ENU, // 变换关系
  mutex &queueMutex
  
  
){

cout<< "优化线程开启"<<endl;


int intail_srt=0;
while (1){


    
    //cout<< "线程2  newGnss "<<  newGnss <<endl;
    sleep(0.0000001); // 每秒生成一个点
    
    if( newGnss==1 && Gnss_enus.size()>=3){
        newGnss=0;// 

      cout<< "线程2 GNSS新数据到来且大于3 初始化:"<< intail_srt<<endl;

      queueMutex.lock(); 


      //  初始化计算srt
      if(!intail_srt && Gnss_enus.size()>=3){
          intail_srt=1;
          cout<< "第一次初始化"<<endl;

          double s;
          Matrix3d R;
          Vector3d t;  
          vector<Vector3d> source_points;
          vector<Vector3d> target_points;
          for(int i=0; i<Gnss_enus.size();i++){
              source_points.push_back(Camera_poses[i].translation());
              target_points.push_back(Gnss_enus[i]);
          }
          


          API_ransac_ICP_3D_3D_sRt_inliner_sR(source_points,target_points,
                                1000,        //ransac随机抽取验证次数
                                3,          // 误差阈值 3         
                                s, R, t) ;
          T_vo_to_GNSS_ENU.block<3, 3>(0, 0) = s * R;
          T_vo_to_GNSS_ENU.block<3, 1>(0, 3) = t;
          cout<< "第一次初始化结束"<<endl;
      }
      
    
      bundleAdjustment_GNSS(Gnss_enus,Camera_poses,optimized_poses,queueMutex,T_vo_to_GNSS_ENU);

      double error_1,error_2,error_3;
      error_1 =cal_rmse_t_se3_se3(optimized_poses,Camera_poses);
      cout << "===========" <<endl;
      cout << "优化和原始 RMSE :" << error_1 << endl;
      
      // 优化数值 更新 旧姿态
      for(int i=0; i<Gnss_enus.size();i++){
          Camera_poses[i]=optimized_poses[i];
      }

      // svd分解计算 srt
      double s;
      Matrix3d R;
      Vector3d t;  
      vector<Vector3d> source_points;
      vector<Vector3d> target_points;

      for(int i=0; i<Gnss_enus.size();i++){
          source_points.push_back(optimized_poses[i].translation());
          target_points.push_back(Gnss_enus[i]);
      }
      
      API_ransac_ICP_3D_3D_sRt_inliner_sR(source_points,target_points,
                            1000,        //ransac随机抽取验证次数
                            3,          // 误差阈值 3         
                            s, R, t) ;
      T_vo_to_GNSS_ENU.block<3, 3>(0, 0) = s * R;
      T_vo_to_GNSS_ENU.block<3, 1>(0, 3) = t;


      for(int i=0;i<optimized_poses.size();i++){

  
            Eigen::Matrix4d optimized_poses_ = optimized_poses[i].matrix();     //  optimized_poses 如果是错的 后面srt也是错的
            Matrix3d R_ = Eigen::Matrix3d::Identity();
            Vector3d t_ = optimized_poses_.block<3, 1>(0, 3) ;

            Eigen::Vector4d t_cw_4(t_(0), t_(1), t_(2), 1);
            Eigen::Vector4d t_cw_4_out= T_vo_to_GNSS_ENU*t_cw_4;
            Eigen::Vector3d t_cw_3(t_cw_4_out(0)/t_cw_4_out(3), t_cw_4_out(1)/t_cw_4_out(3), t_cw_4_out(2)/t_cw_4_out(3));

            Sophus::SE3d SE3_Rt(R_, t_cw_3);     

            optimized_poses[i]=SE3_Rt;
        }

      queueMutex.unlock(); 


      // double error_1,error_2,error_3;
      // error_1 =cal_rmse_t(optimized_poses,Camera_poses);
      error_2 =cal_rmse_t_v3d_se3(Gnss_enus,optimized_poses);
      //error_3 = cal_rmse_t(Gnss_enus,Camera_poses);
      //cout << "位移的均方根误差" <<endl;
      //cout << "真值和原始 RMSE :" << error_1 << endl;
      cout << "真值和优化 RMSE :" << error_2 << endl;
      //cout << "优化和原始 RMSE :" << error_3 << endl;

    }

    else if(Gnss_enus.size()>=100){

      break;

    }
     // sleep(3); // 每秒生成一个点
  }// while



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
  
  // 数据生成线程1
  std::thread Pub_data_thread( Pub_data_GNSS_VO,std::ref(GNSS_points),std::ref(VO_points_T),std::ref(newGnss),std::ref(queueMutex));
  
  // 优化线程2
  std::thread optimized_Gnss_thread( bundleAdjustment_GNSS_thread_base,std::ref(GNSS_points),std::ref(VO_points_T),std::ref(VO_optimized_T),std::ref(newGnss),std::ref(T_vo_to_GNSS_ENU),std::ref(queueMutex));
  
  // 画图线程3
  std::thread Display_thread( displayline_3_thread,std::ref(GNSS_points),std::ref(VO_points_T),std::ref(VO_optimized_T),std::ref(queueMutex));

  // 等待线程结束
  Pub_data_thread.join();
  Display_thread.join();
  optimized_Gnss_thread.join();

 



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
