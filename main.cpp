#ifndef MAIN_CPP
#define MAIN_CPP


#include "API_draw.h"
#include "API_G2O_BA.h"


using namespace std;

double cal_rmse(TrajectoryType &estimated, TrajectoryType &groundtruth){


 // // compute rmse
  double rmse = 0;
  for (size_t i = 0; i < estimated.size(); i++) {
    Sophus::SE3d p1 = estimated[i], p2 = groundtruth[i];
    double error = (p2.inverse() * p1).log().norm();
    rmse += error * error;

  }
  rmse = rmse / double(estimated.size());
  rmse = sqrt(rmse);
  
  cout << "RMSE = " << rmse << endl;
  return rmse;
}


int main(int argc, char **argv) {


  string groundtruth_file = "../data/groundtruth.txt";
  string estimated_file = "../data/estimated.txt";


 
  Point_txyz_List groundtruth = Read_Point_txyz(groundtruth_file);

  TrajectoryType estimated = ReadTrajectory(estimated_file);
  TrajectoryType groundtruth_se3d = ReadTrajectory(groundtruth_file);
 
  assert(!groundtruth.empty() && !estimated.empty());
  assert(groundtruth.size() == estimated.size());

  cout<<"GNSS 数据总数  " <<groundtruth.size()  << endl;
  
  cout<<"estimated 数据总数  " <<estimated.size()  << endl;

  TrajectoryType optimized_pose(groundtruth.size());
  bundleAdjustment(estimated,groundtruth,optimized_pose);


  cal_rmse(estimated,optimized_pose);
  cal_rmse(estimated,groundtruth_se3d);

  cal_rmse(groundtruth_se3d,estimated);
  cal_rmse(groundtruth_se3d,optimized_pose);

  //cal_rmse(optimized_pose,groundtruth_se3d);
  //cal_rmse(optimized_pose,estimated);



  //DrawTrajectory(groundtruth_se3d, optimized_pose);
  DrawTrajectory(optimized_pose, estimated);
  //DrawTrajectory3(optimized_pose, optimized_pose, optimized_pose);

  

  // DrawTrajectory(groundtruth, estimated);
  return 0;
}




#endif 
