////////////////////////////////////////////////////////////////////////////////
//
// Filename:      CameraProcessor_node.cpp
// Authors:       Yu-Han, Hsueh
//
//////////////////////////////// FILE INFO /////////////////////////////////
//
// This node create class ImageConverter to estimate correspondence from
// image
//
/////////////////////////////////// LICENSE ////////////////////////////////////
//
// Copyright (C) 2020 Yu-Han, Hsueh <zero000.ece07g@nctu.edu.tw>
//
// This file is part of {aruco_test}.
//
//////////////////////////////////// NOTES /////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

static const char* OPENCV_WINDOW = "Image window";

class ImageConverter{
  ros::NodeHandle* nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  double fx, fy, cx, cy;
  double edge_length;
  std::vector<float> k;
  std::vector<double> p;
  std::string topic_name_camera;
  std::fstream topic_config;
  std::ofstream outfile_c;

 public:
  explicit ImageConverter(ros::NodeHandle* nh) : nh_(nh), it_(*nh_) {
    // Subscribe image topic and publish output video
    std::string path = ros::package::getPath("aruco_test");
    nh->getParam("/CameraProcessor_node/topic", topic_name_camera);
    image_sub_ = it_.subscribe(topic_name_camera,
                               1,
                               &ImageConverter::cb_camera,
                               this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    nh->getParam("/CameraProcessor_node/intrinsic_parameter/fx", fx);
    nh->getParam("/CameraProcessor_node/intrinsic_parameter/fy", fy);
    nh->getParam("/CameraProcessor_node/intrinsic_parameter/cx", cx);
    nh->getParam("/CameraProcessor_node/intrinsic_parameter/cy", cy);
    nh->getParam("/CameraProcessor_node/distortion_parameter/k", k);
    nh->getParam("/CameraProcessor_node/distortion_parameter/p", p);
    nh->getParam("/CameraProcessor_node/edge_length", edge_length);

    // open file and save the processed target point
    outfile_c.open(path + "/data/camera_data_raw.csv");
    outfile_c << "time_stamp, id, x, y, z" << std::endl;

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter() {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void cb_camera(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      std::cerr << "cv_bridge error." << std::endl;
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat inputImage;
    cv_ptr->image.copyTo(inputImage);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::DetectorParameters parameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary
        = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    cv::aruco::detectMarkers(inputImage, dictionary, corners, ids);
    cv::aruco::drawDetectedMarkers(inputImage, corners, ids);

    // intrinsic parameters and distortion coefficients
    cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat1d(1, 5) << k[0], k[1], p[0], p[1], k[2]);

    // ArUco marker detection
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners,
                                         edge_length,
                                         cameraMatrix,
                                         distCoeffs,
                                         rvecs,
                                         tvecs);

    std::vector<cv::Point3d> points_3D;
    std::vector<cv::Point2d> points_2D;
    for (int i = 0; i < ids.size(); i++) {
      cv::aruco::drawAxis(inputImage,
                          cameraMatrix,
                          distCoeffs,
                          rvecs[i],
                          tvecs[i],
                          0.1);
      outfile_c << msg->header.stamp <<", "
                << ids[i] << ", "
                << tvecs[i][0] << ", "
                << tvecs[i][1] << ", "
                << tvecs[i][2] << std::endl;
      points_3D.push_back(cv::Point3d(tvecs[i][0], tvecs[i][1], tvecs[i][2]));
    }

    // Testing (project 3D points to 2D image plane)
    cv::Vec3d rVec {0, 0, 0};
    cv::Vec3d tVec {0, 0, 0};
    cv::projectPoints(points_3D,
                      rVec,
                      tVec,
                      cameraMatrix,
                      distCoeffs,
                      points_2D);

    for (int i = 0; i < points_2D.size(); i++) {
      cv::circle(inputImage,
                 cvPoint(points_2D[i].x, points_2D[i].y),
                 15./cv::norm(tvecs[i]),
                 CV_RGB(255, 255, 255),
                 CV_FILLED,
                 CV_AA,
                 0);
    }

    cv::imshow(OPENCV_WINDOW, inputImage);
    cv::waitKey(3);

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;

    img_bridge = cv_bridge::CvImage(msg->header,
                                    sensor_msgs::image_encodings::BGR8,
                                    inputImage);
    img_bridge.toImageMsg(img_msg);

    // Output modified video stream
    image_pub_.publish(img_msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "CameraProcessor_node");
  ros::NodeHandle nh("~");
  ImageConverter ic(&nh);
  ros::spin();
  return 0;
}
