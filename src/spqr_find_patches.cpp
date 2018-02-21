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
//#include "opencv2/core/types.hpp"
#include <iosfwd>
#include "spqr_find_patches/rotated_bounding_box.h"
#include "spqr_find_patches/bounding_box_list.h"

#include "youbot_vision.h"


using namespace cv;
using namespace std;

static const string OPENCV_WINDOW = "Image window";

void print_boxes(const vector <Rect >& bounding_boxes, Mat& drawing);
void print_rotated_boxes(const vector <RotatedRect >& rot_boxes, Mat& drawing);

int find_patch_flag;
string find_patch_obj_name;

//Boundingboxes array
spqr_find_patches::bounding_box_list rbb_array;


class ImageConverter {

public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher rbb_pub_;
    int filter1;
    int filter2;

    YoubotObjectLocalizer* object_localizer;

  
    string input_topic;

    ImageConverter() : it_(nh_) {
        // Subscrive to input video feed and publish output video feed
        filter1 = 9;
        filter2 = 9;

        nh_.param("find_patch_flag",find_patch_flag,0);
        nh_.param("find_patch_obj_name",find_patch_obj_name,string(""));
        nh_.param<std::string>("input_topic", input_topic, "/arm_camera/rgb/image_color");

        image_sub_ = it_.subscribe(input_topic, 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/found_patches/output", 1);
        //bb_pub_ = nh_.advertise<geometry_msgs::PolygonStamped >("/bounding_boxes", 1);

        rbb_pub_ = nh_.advertise<spqr_find_patches::bounding_box_list >("/rotated_bounding_boxes", 1);

        cv::namedWindow(OPENCV_WINDOW);
        
        object_localizer = new YoubotObjectLocalizer();
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {

      rbb_array.boxes.clear();
        nh_.getParam("find_patch_flag",find_patch_flag);
        if(find_patch_flag==1){
            nh_.getParam("find_patch_obj_name",find_patch_obj_name);
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            if (cv_ptr) {
                // Put here the code for manipulating the img
                int spqr_or_slaw = 1;
                Mat src_gray;
                if(spqr_or_slaw==0)
                {     
                    pyrDown(cv_ptr->image, cv_ptr->image);
                    pyrDown(cv_ptr->image, cv_ptr->image);
//                     pyrDown(cv_ptr->image, cv_ptr->image);

                    cvtColor(cv_ptr->image, src_gray, CV_BGR2GRAY);
                    blur(src_gray, src_gray, Size(3, 3));
                    imshow("ciao", src_gray);
                    waitKey(10);
                    object_localizer->performSimpleDetection(cv_ptr->image, src_gray, cv_ptr->header, rbb_array);
                }
                else // HALCON
                {
                  /*cvtColor(cv_ptr->image, src_gray, CV_BGR2GRAY);
                  imshow("ciao", src_gray);
                  waitKey(10);
                  object_localizer->performHalconLocalization(find_patch_obj_name, src_gray, cv_ptr->header, rbb_array);
                  */
                }
                rbb_pub_.publish(rbb_array);
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
//     YoubotObjectLocalizer::create3DShapeModels(1);
    ImageConverter ic;
    ros::spin();
    return 0;
}

void print_boxes(const vector <Rect >& bounding_boxes, Mat& drawing) {

    Scalar color = Scalar(0, 0, 255);
    int num_boxes = bounding_boxes.size();
    
    for (int i = 0; i < num_boxes; i++) {

        Point2f tl = bounding_boxes[i].tl();
        Point2f br = bounding_boxes[i].br();

        rectangle(drawing, tl, br, color, 2, 8, 0);

        /*
        //Save boxes into images
        string num;
        ostringstream convert;
        convert << i;
        num = convert.str();
        //extract the part of the image inside the bounding box 
        //and create a new image
        Rect myRoi(tl, br);
        Mat new_image = drawing(myRoi);
        resize(new_image, new_image, size);
        imwrite(num + ".jpg", new_image);
        */

    }
}

void print_rotated_boxes(const vector <RotatedRect >& rot_rect, Mat& drawing) {

    Scalar color = Scalar(0, 255, 255);
    int num_boxes = rot_rect.size();

    for (int i = 0; i < num_boxes; i++)
    {
        Point2f vertices[4];
        rot_rect[i].points(vertices);
        for (int i = 0; i < 4; i++)
            line(drawing, vertices[i], vertices[(i+1)%4], color);

    }
}
