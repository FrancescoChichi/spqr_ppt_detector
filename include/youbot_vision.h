#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <unordered_map>

#include "spqr_find_patches/bounding_box_list.h"
// #include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

//#include "HalconCpp.h"
//#include "HDevEngineCpp.h"

using namespace std;
using namespace cv;

class YoubotObjectLocalizer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  YoubotObjectLocalizer()
  {
    lowThreshold = 70;
    ratio = 3;
    kernel_size = 3;
    scale = 4;
    thresh = 200;
    max_thresh = 255;
    rng=RNG(12345);
    size=Size(100, 100);
    perc=.6;
    overlap_pixels=10;
    K<<614.3051147460938, 0.0, 309.6654052734375, 0.0, 614.3052368164062, 230.21109008789062, 0.0, 0.0, 1.0;

    //HalconCpp::HTuple wh;
//     window_hal_.OpenWindow((Hlong)0, (Hlong)0, (Hlong)640, (Hlong)480, HalconCpp::HString("root"),HalconCpp::HString("visible"),HalconCpp::HString(""));
    
    //readShapeModels();

    w_created=false;
  }
  
  //static void create3DShapeModels(float im_scale);
  
  void performSimpleDetection( const Mat& src, const Mat& src_gray, const std_msgs::Header& img_header, spqr_find_patches::bounding_box_list& rbb_array);
  //void performHalconLocalization(const string obj_name, const Mat& src, const std_msgs::Header& img_header, spqr_find_patches::bounding_box_list& rbb_array);
  
private:
  
  //void readShapeModels();
  //void readShapeModel(string name);
  
  void getBBOrientationWilsonMarco(const vector<Point> &pts, RotatedRect& rbb);
  
  void computeBB(const vector<Point>& contour, Rect& bb, Point2f& center);
  
  void getContourCentroid(const vector<Point>& contour, Point& centroid);
  
  inline bool isInside(const Rect& boxBig, const Rect& boxSmall, const int& margin)
  {
    return boxSmall.tl().x + margin >= boxBig.tl().x && boxSmall.tl().y + margin >= boxBig.tl().y &&
            boxSmall.br().x - margin <= boxBig.br().x && boxSmall.br().y - margin <= boxBig.br().y;
  }
  
  void rotateContour(const vector<Point>& contour, const float theta, const Point& centroid, vector<Point>& out);
  
  static void clearProc(void* ptr) {
    return;
  }
  
  //void projectPoint(const Eigen::Vector3d& p, Eigen::Vector2d& im_p);
  
  // CANNY Parameters
  int lowThreshold;
  int ratio;
  int kernel_size;
  int scale;
  
  // Other Global Parameters
  int thresh;
  int max_thresh;
  RNG rng;
  Size size;
  
  double perc;           //  bounding box < image_size_max * max_box
  int   overlap_pixels;  //  delete boxes that are not perfectly one inside each other 
  
  
  //Halcon
  //unordered_map<string, vector<HalconCpp::HShapeModel3D> > models_;
  //HalconCpp::HWindow* window_hal_;
  Eigen::Matrix3d K;
  bool w_created;
};
