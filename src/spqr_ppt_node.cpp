#include <ros/ros.h>
#include <std_msgs/String.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include "spqr_find_patches/bounding_box_list.h"

using namespace cv;
using namespace std;

//***********************************
int iLowH = 0;
int iHighH = 175;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 100;


int threshold_line = 30;
int min_line_length = 45;
int max_line_gap = 3;


double orientation;
ros::Publisher rbb_pub_;
spqr_find_patches::bounding_box_list rbb_array;
std::vector<Point> centers;
int areaTreshold = 10000;
int distanceTreshold = 1000000;
int pointsInsideTreshold = 1;
int th1 = 95;
int th2 = 200;
int kernel = 3;

float distanceBetweenTwoPoints(Point a, Point b){
  return (sqrt(pow((a.x-b.x),2.0)+pow((a.y-b.y),2.0)));
}


void getCenter(vector<Point> &pts, Mat &img)
{
    //Construct a buffer used by the pca analysis
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

    //Store the position of the object
    Point pos = Point(pca_analysis.mean.at<double>(0, 0),
                      pca_analysis.mean.at<double>(0, 1));

    centers.push_back(pos);

    circle(img, pos, 3, CV_RGB(255, 0, 255), 2);
}

double getOrientation(vector<Point> &pts,std::vector<cv::Point_<int> > edges, Mat &img, string text)
{
  if(text!="quad"){
    //Construct a buffer used by the pca analysis
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

    //Store the position of the object
    //Point pos = Point(pca_analysis.mean.at<double>(0, 0),
      //                pca_analysis.mean.at<double>(0, 1));


    Point pos = Point((edges[0].x+edges[1].x+edges[2].x+edges[3].x)/4,
                      (edges[0].y+edges[1].y+edges[2].y+edges[3].y)/4);

    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));

        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    putText(img, text ,pos , FONT_HERSHEY_SIMPLEX, 1,Scalar(0,0,255),2,LINE_AA);    

    // Draw the principal components
    circle(img, pos, 3, CV_RGB(255, 0, 255), 2);
    line(img, pos, pos + 0.02 * Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]) , CV_RGB(255, 255, 0));
    line(img, pos, pos + 0.02 * Point(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]) , CV_RGB(0, 255, 255));

    return atan2(eigen_vecs[0].y, eigen_vecs[0].x);
  }
  
  Point pos = Point((edges[0].x+edges[1].x+edges[2].x+edges[3].x)/4,
                      (edges[0].y+edges[1].y+edges[2].y+edges[3].y)/4);

  putText(img, text ,pos , FONT_HERSHEY_SIMPLEX, 1,Scalar(0,0,255),2,LINE_AA);    

  // Draw the principal components
  circle(img, pos, 3, CV_RGB(255, 0, 255), 2);
  line(img, edges[0], edges[2] , CV_RGB(255, 255, 0));
  line(img, edges[1], edges[3] , CV_RGB(0, 255, 255));

  return atan2(edges[2].y, edges[2].x) - 0.785398f; //45% = 0.785398f rad
}

void getShape(std::vector<cv::Point_<int> > corners, vector<Point> &pts, Mat &img){
  string label = "";
  if(corners.size()==4){

    vector<float> edges;
    for (int i=0;i<4;i++)
      edges.push_back(distanceBetweenTwoPoints(corners[i],corners[(i+1)%4]));

    float min = *std::min_element(edges.begin(),edges.end());
    float max = *std::max_element(edges.begin(),edges.end());

    if (max/min <= 1.7f)
      label = "quad";
    else
      label = "rect";

    //cout<<label<<" min "<<min<<" max "<<max<<endl;
    //cout<<endl<<" "<<max/min<<endl<<endl<<endl;
    
    getOrientation(pts, corners, img, label);
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  rbb_array.boxes.clear();

  cv_bridge::CvImagePtr cv_ptr;
  bool found = false;
  try
  {
    centers.clear();
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image_mat = cv_ptr->image;

    Mat canny_output;

    /// Detect edges using canny
    //Mat gray = image_mat.clone();
    //cvtColor(image_mat, gray, COLOR_BGR2HSV); //Convert the captu frame from BGR to HSV


    Mat imgHSV;

    cvtColor(image_mat, imgHSV, COLOR_BGR2HSV); //Convert the captu frame from BGR to HSV
    
    Mat gray;

    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), gray); //Threshold the image 



    Size size = Size(10,10);

    //morphological opening (remove small objects from the foreground)
    erode(gray, gray, getStructuringElement(MORPH_ELLIPSE, size) );
    dilate( gray, gray, getStructuringElement(MORPH_ELLIPSE, size) );

    //morphological closing (fill small holes in the foreground)
    dilate( gray, gray, getStructuringElement(MORPH_ELLIPSE, size) );
    erode(gray, gray, getStructuringElement(MORPH_ELLIPSE, size) );


    Canny( gray, canny_output, th1, th2, kernel );

    vector<vector<Point> > contours;

    vector<Vec4i> hierarchy;


    /// Find contours 
    findContours( canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    vector<Rect> boundRect( contours.size() );
    vector<vector<Point> > contours_poly_( contours.size() );
    vector<RotatedRect> minRect( contours.size() );

    rbb_array.header = msg->header;
    rbb_array.header.stamp=ros::Time::now();
    std::vector<cv::Point> approx;  
    
    Mat cont = image_mat.clone();
    Scalar color = Scalar( 0, 0, 255 );
    


    for( int i = 0; i < contours.size(); i++ )
      minRect[i] = minAreaRect( Mat(contours[i]) );

    for( int i = 0; i< contours.size(); i++ )
    {
      //drawContours( cont, contours, i, color, 2, 8, hierarchy, 0, Point() );
      // Approximate contour with accuracy proportional
      // to the contour perimeter
      cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

      
      // Skip small or non-convex objects 
      if (std::fabs(cv::contourArea(contours[i])) < 10000 || !cv::isContourConvex(approx))
        continue;


      //for(int j=0;j<approx.size();j++)
        //circle(image_mat, approx[j], 3, Scalar(255,0,0), 5, 8, 0);

      if (approx.size() == 4)
      { 
        Point2f rect_points[4]; minRect[i].points( rect_points );
        for( int j = 0; j < 4; j++ )
          line( image_mat, rect_points[j], rect_points[(j+1)%4], color, 2, 8 );

        drawContours( cont, contours, i, color, 2, 8, hierarchy, 0, Point() );
        getShape(approx, contours[i], image_mat);

        spqr_find_patches::rotated_bounding_box rbb_msg;

        rbb_msg.center.x = minRect[i].center.x;
        rbb_msg.center.y = minRect[i].center.y;
        rbb_msg.width = minRect[i].size.width;
        rbb_msg.height = minRect[i].size.height;
        rbb_msg.angle = minRect[i].angle;
        rbb_array.boxes.push_back(rbb_msg);

      }

      /*else if (approx.size() == 6)
      { 
        Point2f rect_points[6]; minRect[i].points( rect_points );
        for( int j = 0; j < 6; j++ )
          line( image_mat, rect_points[j], rect_points[(j+1)%6], color, 2, 8 );

        
        drawContours( image_mat, contours, i, Scalar(0,255,0), 2, 8, hierarchy, 0, Point() );
        /*getShape(approx, contours[i], image_mat);

        spqr_find_patches::rotated_bounding_box rbb_msg;

        rbb_msg.center.x = minRect[i].center.x;
        rbb_msg.center.y = minRect[i].center.y;
        rbb_msg.width = minRect[i].size.width;
        rbb_msg.height = minRect[i].size.height;
        rbb_msg.angle = minRect[i].angle;
        rbb_array.boxes.push_back(rbb_msg);

      }*/
    }

    imshow("original", image_mat);
    //imshow("gray", gray);

    rbb_pub_.publish(rbb_array);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  cv::waitKey(30);

}

void trackbar(){

  
    namedWindow("Control ", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    cvCreateTrackbar("first threshod", "Control ", &th1, 500);
    cvCreateTrackbar("second threshod", "Control ", &th2, 500);
    cvCreateTrackbar("LowH", "Control ", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control ", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control ", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control ", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control ", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control ", &iHighV, 255);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh_;

  //trackbar();

  cv::startWindowThread();
  image_transport::ImageTransport it(nh_);
  image_transport::Subscriber sub = it.subscribe("/softkinetic_camera/rgb/image_color", 1, imageCallback);
  //image_transport::Subscriber sub = it.subscribe("/spqr_camera/rgb", 1, imageCallback);

  rbb_pub_ = nh_.advertise<spqr_find_patches::bounding_box_list >("/rotated_bounding_boxes", 1);

  ros::spin();
}


