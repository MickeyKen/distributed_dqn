#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_datatypes.h>
#include <chrono>
#include <thread>

int data_base = 0;
void Callback(const std_msgs::Int16& msg)
{
  //std::cout << msg.data << std::endl;
  data_base=msg.data;
  ros::NodeHandle n;

  int exp_num = 0;
  int fin_switch = 1;

  // n.getParam("/exp_num", exp_num);
  n.setParam("exp_miki_img/switch", 1);

  exp_num = 91;


  if (exp_num == 91 || exp_num == 92 || exp_num == 93) {
    ///// decide image size in real world
    float size = 800 / 2;
    int ran = rand() % 10;
    if (ran % 2 == 0) {

    } else {
      ran = ran + 1;

    }
    ///// get image and resize projectr size
    std::string input_file_path = "/home/mikintu183/bird_catched.png";
    ///std::string input_file_path = file_dir + std::to_string(ran) + ".png";
    cv::Mat source_img = cv::imread(input_file_path, cv::IMREAD_UNCHANGED);
    int ColumnOfNewImage = 1024;
    int RowsOfNewImage = 768;
    resize(source_img, source_img, cv::Size(ColumnOfNewImage,RowsOfNewImage));

    ///// BEFORE homography
    const cv::Point2f src_pt[]={
             cv::Point2f(0.0, 0.0),
             cv::Point2f(1023.0 , 0.0),
             cv::Point2f(1023.0 , 767.0),
             cv::Point2f(0.0, 767.0)};
    // AFTER homography
    cv::Point2f dst_pt[]={
             cv::Point2f(0.0, 0.0),
             cv::Point2f(0.0 , 0.0),
             cv::Point2f(0.0 , 0.0),
             cv::Point2f(0.0, 0.0)};


    ///// rojector inner parameter
    const cv::Mat Ap = (cv::Mat_<float>(3, 3) << 2145.37932 ,  0.00000000 ,  495.015557,
                                                       0.00000000 ,  2055.54230 ,  800.250515,   //457.250515
                                                      0.00000000 ,  0.00000000 ,  1.00000000);
    // std::cout << "M = "<< std::endl << " "  << Ap << std::endl << std::endl;

    ///// camera inner parameter
    const cv::Mat Ac = (cv::Mat_<float>(3, 3) << 1.0464088296606685e+03 ,  0.00000000 ,  9.6962285013582118e+02,
                                                       0.00000000 ,  1.0473601981442353e+03 ,  5.3418043955010319e+02,
                                                      0.00000000 ,  0.00000000 ,  1.00000000);

    ///// projector center point 512.0 384.0
    const cv::Mat uv_center = (cv::Mat_<float>(3,1) << 512.0, 384.0, 1.0);


    ///// prepare roatation matrix for x,y,z
    //CV_32F is float
    cv::Mat rot_x(3, 3, CV_32F, cv::Scalar::all(0.0));
    cv::Mat rot_y(3, 3, CV_32F, cv::Scalar::all(0.0));
    cv::Mat rot_z(3, 3, CV_32F, cv::Scalar::all(0.0));
    rot_x.at<float>(0, 0) = 1.0;
    rot_y.at<float>(1, 1) = 1.0;
    rot_z.at<float>(2, 2) = 1.0;
    // for rot_x * rot_y * rot_z
    cv::Mat Rotation(3, 3, CV_32F);

    cv::Mat target = (cv::Mat_<float>(4,3) << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);

    ///// prepare image matrix for display
    cv::Mat warp_img(cv::Size(1024, 768), CV_8U, cv::Scalar::all(0));


    ///// for calcurate
    cv::Mat calc = (cv::Mat_<float>(3,1) << 1.0, 1.0, 1.0);

    ///// prepare rate and tf
    ros::Rate rate(20);
    tf::TransformListener listener;


    ///// main function
    while (ros::ok()) {

      ///// switch
      n.getParam("exp_miki_img/switch", fin_switch);
      if (fin_switch == 0) {
        break;
      }

      ///// get Rotation and Translation
      tf::StampedTransform transform;
      try {
        listener.waitForTransform("/projector_optical_frame","/base_footprint", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/projector_optical_frame","/base_footprint", ros::Time(0), transform);

      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      tf::Quaternion q(transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      // std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;


      ///// calcurate Rotation Matrix
      // insert Rotation matrix for X
      rot_x.at<float>(1, 1) = cos(roll);
      rot_x.at<float>(1, 2) = -sin(roll);
      rot_x.at<float>(2, 1) = sin(roll);
      rot_x.at<float>(2, 2) = cos(roll);

      //insert Ritation matrix for y
      rot_y.at<float>(0, 0) = cos(pitch);
      rot_y.at<float>(0, 2) = sin(pitch);
      rot_y.at<float>(2, 0) = -sin(pitch);
      rot_y.at<float>(2, 2) = cos(pitch);

      //insert Rotation matrix for z
      rot_z.at<float>(0, 0) = cos(yaw);
      rot_z.at<float>(0, 1) = -sin(yaw);
      rot_z.at<float>(1, 0) = sin(yaw);
      rot_z.at<float>(1, 1) = cos(yaw);

      Rotation = rot_z * rot_y * rot_x;
      std::cout << "Rot pre:" << Rotation << std::endl;

      Rotation.at<float>(0, 2) = transform.getOrigin().x()*1000;
      Rotation.at<float>(1, 2) = transform.getOrigin().y()*1000;
      Rotation.at<float>(2, 2) = transform.getOrigin().z()*1000;
      std::cout << "Rot post:" << Rotation << std::endl;

      ///// calcurate center x-y-z point in real world
      calc = (Ap * Rotation).inv() * uv_center;

      calc = calc / calc.at<float>(2,0);
      std::cout << "cmoplete:" << calc << std::endl;

      target.at<float>(0,0) = calc.at<float>(0,0) - size;
      target.at<float>(0,1) = calc.at<float>(1,0) + size;
      target.at<float>(0,2) = 1.>0;

      target.at<float>(1,0) = calc.at<float>(0,0) + size;
      target.at<float>(1,1) = calc.at<float>(1,0) + size;
      target.at<float>(1,2) = 1.0;

      target.at<float>(2,0) = calc.at<float>(0,0) + size;
      target.at<float>(2,1) = calc.at<float>(1,0) - size;
      target.at<float>(2,2) = 1.0;

      target.at<float>(3,0) = calc.at<float>(0,0) - size;
      target.at<float>(3,1) = calc.at<float>(1,0) - size;
      target.at<float>(3,2) = 1.0;

      for (int i = 0; i < 4; i++) {
        calc =  Ap * Rotation * target.row(i).t();
        dst_pt[i].x = calc.at<float>(0,0) / calc.at<float>(2,0);
        dst_pt[i].y = calc.at<float>(1,0) / calc.at<float>(2,0);
        // printf("x: %f , y: %f", dst_pt[i].x, dst_pt[i].y);
        // std::cout << "g = "<< std::endl << " "  << Ap * Rotation * target.row(i).t() << std::endl << std::endl;
      }
      cv::Mat M = cv::getPerspectiveTransform(src_pt,dst_pt);
      cv::warpPerspective( source_img, warp_img, M, source_img.size());
      // std::cout << "g = "<< std::endl << " "  << M << std::endl << std::endl;
      ///// set window fullscreen
      cv::namedWindow( "screen", CV_WINDOW_NORMAL );
      cv::setWindowProperty("screen",CV_WND_PROP_FULLSCREEN,CV_WINDOW_FULLSCREEN);
      cv::imshow("screen", warp_img);
      cv::waitKey(1);

      rate.sleep();
    }


    cv::destroyAllWindows();
  }


}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "for_rec");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("finish_pantilt", 1000, &Callback);

  ros::Rate rate(20);

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
    }


  return 0;
}
