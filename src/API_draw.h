#ifndef DRAW_CPP
#define DRAW_CPP

#include "API_dataAndFileIO.h"


#include <iostream>

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

using namespace Sophus;
using namespace std;

using namespace Eigen;




void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));


  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glLineWidth(2);
    for (size_t i = 0; i < gt.size() - 1; i++) {
      glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
      glBegin(GL_LINES);  //GL_POINTS
      //glBegin(GL_POINTS)
      auto p1 = gt[i], p2 = gt[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }

    for (size_t i = 0; i < esti.size() - 1; i++) {
      glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
      glBegin(GL_LINES);
      //glBegin(GL_POINTS);
      auto p1 = esti[i], p2 = esti[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }

}

//  y一次性画完 3个轨迹图
void DrawTrajectory3(const TrajectoryType &groundtruth, const TrajectoryType &estimated,const TrajectoryType &optimized_pose) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));


  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glLineWidth(2);
    //cout<<"groundtruth总点数  "  << groundtruth.size()<< endl;
    for (size_t i = 0; i < groundtruth.size() - 1; i++) {
      glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
      glBegin(GL_LINES);
      //glBegin(GL_POINTS);
      auto p1 = groundtruth[i], p2 = groundtruth[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }

    //cout<<"estimated总点数  "  << estimated.size()<< endl;
    for (size_t i = 0; i < estimated.size() - 1; i++) {
      glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
      glBegin(GL_LINES);
      auto p1 = estimated[i], p2 = estimated[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }

        
    //cout<<"optimized_pose总点数  "  << optimized_pose.size()<< endl;
    for (size_t i = 0; i < optimized_pose.size() - 1; i++) {
      glColor3f(0.0f, 1.0f, 0.0f);  // green for optimized_pose
      glBegin(GL_LINES);
      //glBegin(GL_POINTS);
      int distance_=0;
      auto p1 = optimized_pose[i], p2 = optimized_pose[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]+distance_);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]+distance_);
      glEnd();
    }

    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }

}



// // 用于存储3D点的队列和互斥锁
// vector<Eigen::Vector3d> pointQueue;
// mutex queueMutex;

// 显示线程使用Pangolin显示和处理3D点
void displayPoints(vector<Eigen::Vector3d> &pointQueue , mutex &queueMutex) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );
 
  pangolin::View &d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));
 
    
  while (!pangolin::ShouldQuit()) {
        // 清空缓冲区
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
 
        // 处理队列中的所有点并绘制
        queueMutex.lock();
 
 
         
        if(pointQueue.size()>=2){ 
             
            for (size_t i = 0; i < pointQueue.size() - 1; i++) {
                    Eigen::Vector3d point1 = pointQueue[i];
                    
                    Eigen::Vector3d point2 =  pointQueue[i+1];
 
                    cout<<point1<<"    "<< point2 <<endl;
 
                    glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
                    glBegin(GL_LINES);
             
                    glVertex3d(point1[0], point1[1], point1[2]);
                    glVertex3d(point2[0], point2[1], point2[2]);
                    glEnd();
 
                 
           }
        }
         
        queueMutex.unlock();
         
     
        // 完成帧并交换缓冲区
        pangolin::FinishFrame();
        usleep(1000);   // sleep 5 ms
    }
 
    // 关闭Pangolin窗口
    pangolin::DestroyWindow("3D Points Display");
}




// 显示线程使用Pangolin显示和处理3D点
void displayline_3_thread(vector<Vector3d> &GNSS_points,TrajectoryType &VO_points_T,TrajectoryType &VO_optimized_T,mutex &queueMutex) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );
 
  pangolin::View &d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));
 
  
  while (!pangolin::ShouldQuit()) {
        // 清空缓冲区
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
 
        // 处理队列中的所有点并绘制
        queueMutex.lock();
 


         
        if(GNSS_points.size()>=2){ 
             
            for (size_t i = 0; i < GNSS_points.size() - 1; i++) {
                    Eigen::Vector3d point1 = GNSS_points[i];
                    
                    Eigen::Vector3d point2 =  GNSS_points[i+1];
 
                    //cout<<point1<<"    "<< point2 <<endl;
 
                    glColor3f(0, 0, 255);  // blue for ground truth
                    glBegin(GL_LINES);
             
                    glVertex3d(point1[0], point1[1], point1[2]);
                    glVertex3d(point2[0], point2[1], point2[2]);
                    glEnd();
 
                 
           }
        }
         
                 
        if(VO_points_T.size()>=2){ 
             
            for (size_t i = 0; i < VO_points_T.size() - 1; i++) {


                    Eigen::Vector3d point1 = VO_points_T[i].translation();
                    
                    Eigen::Vector3d point2 =  VO_points_T[i+1].translation();
 
                    //cout<<point1<<"    "<< point2 <<endl;
 
                    glColor3f(255,0, 0);  // 红色 原始未优化位姿
                    glBegin(GL_LINES);
             
                    glVertex3d(point1[0], point1[1], point1[2]);
                    glVertex3d(point2[0], point2[1], point2[2]);
                    glEnd();
 
                 
           }
        }

                  
        if(VO_optimized_T.size()>=2){ 
             
            for (size_t i = 0; i < VO_optimized_T.size() - 1; i++) {


                    Eigen::Vector3d point1 = VO_optimized_T[i].translation();
                    
                    Eigen::Vector3d point2 =  VO_optimized_T[i+1].translation();
 
                    //cout<<point1<<"    "<< point2 <<endl;
 
                    glColor3f(0,255, 0);  // 绿色 优化后的位姿
                    glBegin(GL_LINES);
                    int out=1;
                    glVertex3d(point1[0]+out, point1[1], point1[2]);
                    glVertex3d(point2[0]+out, point2[1], point2[2]);
                    glEnd();
 
                 
           }
        }
        queueMutex.unlock();
         
     
        // 完成帧并交换缓冲区
        pangolin::FinishFrame();
        usleep(1000);   // sleep 5 ms
    }
 
    // 关闭Pangolin窗口
    pangolin::DestroyWindow("3D Points Display");
}



#endif 