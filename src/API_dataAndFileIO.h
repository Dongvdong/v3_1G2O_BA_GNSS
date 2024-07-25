
#ifndef API_dataAndIO_CPP
#define API_dataAndIO_CPP



#include <iostream>
#include <fstream>
#include <unistd.h>

#include <cmath>
#include <random>   

#include <Eigen/Dense>
#include <vector>

#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

using namespace Sophus;
using namespace std;
using namespace Eigen;




// 点云类型定义
typedef vector<Vector3d> PointCloud;
 
 
struct  Point4txyz{
    double timestamp;
    double x;
    double y;
    double z;
} ;

typedef vector<Point4txyz,Eigen::aligned_allocator<Point4txyz>> Point_txyz_List;
typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;






TrajectoryType ReadTrajectory(const string &path) {
  ifstream fin(path);
  TrajectoryType trajectory;
  if (!fin) {
    cerr << "trajectory " << path << " not found." << endl;
    return trajectory;
  }

  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    Sophus::SE3d p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
    trajectory.push_back(p1);
  }
  return trajectory;
}



Point_txyz_List Read_Point_txyz(const string &path) {
  ifstream fin(path);
  Point_txyz_List trajectory1;

  if (!fin) {
    cerr << "trajectory " << path << " not found." << endl;
    return trajectory1;
  }

  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    //point.timestamp=std::stod(timestamp);
    Point4txyz point_;
    point_.timestamp=time;
    point_.x=tx;
    point_.y=ty;
    point_.z=tz;
    trajectory1.push_back(point_);
  }
  //cout<<"trajectory读取完毕 数据总数  " <<trajectory.size()  << endl;

  return trajectory1;
}





 
 std::vector<Eigen::Vector3d> generateCircle3D(double radius, int numPoints) {
    std::vector<Eigen::Vector3d> circlePoints;

    for (int i = 0; i < numPoints; ++i) {
        double theta = 2.0 * M_PI * i / numPoints;
        double x = radius * std::cos(theta);
        double y = radius * std::sin(theta);
        double z = 0.0;  // 在平面内，z 始终为 0
        circlePoints.emplace_back(x, y, z);
    }

    return circlePoints;
}


void Get_TestValue(vector<Vector3d>& source_points ,vector<Vector3d>& target_points,double currentScale,Matrix3d currentR,Vector3d currentT){
 
    
    //1-1 随机数产生原始点
    std::random_device rd; // 使用 std::random_device 生成真随机数种子
    std::mt19937 gen(rd());// 使用梅森旋转算法引擎 mt19937，以 rd() 作为种子
    uniform_int_distribution<int> randanm_once1(0, 100);//定义一个均匀整数分布
    int random_number = randanm_once1(gen);// 生成随机数
     
    int N = 100; // 100个点
    float per=0.01;// 假设10%点是对的
    double noise_error=10;// 噪声范围
 
 
    // //1-2 随机创建原始点
    // for (int i = 0; i < N; ++i) {



    //     source_points.push_back(Vector3d(randanm_once1(gen),
    //                                      randanm_once1(gen),
    //                                      randanm_once1(gen)));
    // }
     
    double radius = 60;  // 圆的半径
    int numPoints = N; // 生成的点的数量
    source_points = generateCircle3D(radius, numPoints);

    //2 设置目标点 = 原始点+噪声
 
    //2-1 设置随机噪声
     
    std::uniform_int_distribution<int> randanm_once2(-noise_error/2, noise_error/2);
    //int random_number = randanm_once2(gen);
  
     
    for(int i=0;i<N;i++){
        
        Vector3d dst_i = currentScale * currentR * source_points[i] + currentT;// 目标点=s*R*原始点+t
        Vector3d noise(randanm_once2(gen) ,randanm_once2(gen) ,randanm_once2(gen));// 噪声
         
        // 目标点组成
        if (i > int(N*(1-per))){target_points.push_back(dst_i );} // 目标点真值
        else{target_points.push_back(dst_i + noise);} // 目标点真值+噪声 模拟视觉slam 和GNSS（民用10米 rtk 1-10cm）本身误差
         
        // cout<< i << " "                        
        //         <<"  source_points: "<< source_points[i][0]<< " "<<source_points[i][1]<<" "<<source_points[i][2]
        //         <<"  noise: "<< noise[0]<< " "<<noise[1]<<" "<<noise[2]
        //         <<"  target_points: "<< dst_i[0]<< " "<<dst_i[1]<<" "<<dst_i[2]
        //         <<"  target_points_niose: "<< target_points[i][0]<< " "<<target_points[i][1]<<" "<<target_points[i][2]
        //         << endl;
                
    }
  
  
    // 目标点真值+超级噪声 （模拟视觉SLAM某次错误的位姿）加上几个特别的离群来捣乱 验证内点的筛选重要性
    target_points[50]+=(Vector3d){100,100,100};
 
    target_points[60]+=(Vector3d){-100,-100,100};
 
    target_points[70]+=(Vector3d){-100,-100,-100};
 
    target_points[80]+=(Vector3d){-100,100,-100};
 
}



// 主线程生成3D点的函数
void generatePoints( vector<Vector3d> &source_points,vector<Vector3d> &target_points) {
 
  // 1-2 设置source_points到 target_points变换矩阵
    Matrix3d RelativeR;
    Vector3d Relativet;
    double currentScale=1;
    //RelativeR<<1,0,0,0,1,0,0,0,1;// 单位阵
    //RelativeR<<0,-1,0,1,0,0,0,0,1; // 旋转90度
    double yaw = 0;    // 绕Z轴的角度
    double pitch = 0;  // 绕Y轴的角度
    double roll = 0;   // 绕X轴的角度
    RelativeR = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    cout<< "RelativeR "<<RelativeR <<endl;
    Relativet<<10,10,10;
    // 1-3 获取随机测试点
    Get_TestValue(source_points,target_points,currentScale,RelativeR,Relativet);

 
}





#endif 