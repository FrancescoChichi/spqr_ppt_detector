#include <ros/ros.h>
#include <std_msgs/String.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>

using namespace cv;
using namespace std;

//***********************************

int threshold_line = 30;
int min_line_length = 45;
int max_line_gap = 3;


double orientation;
ros::Publisher chatter_pub;
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

double getOrientation(vector<Point> &pts, Mat &img, string text)
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

    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));

        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    putText(img, text ,pos , FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,LINE_AA);    

    // Draw the principal components
    circle(img, pos, 3, CV_RGB(255, 0, 255), 2);
    line(img, pos, pos + 0.02 * Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]) , CV_RGB(255, 255, 0));
    line(img, pos, pos + 0.02 * Point(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]) , CV_RGB(0, 255, 255));

    return atan2(eigen_vecs[0].y, eigen_vecs[0].x);
}

struct sortX_class {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x < pt2.x);}
} sortX;

struct sortY_class {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.y < pt2.y);}
} sortY;

void getShape(std::vector<cv::Point_<int> > edges, vector<Point> &pts, Mat &img){
  string label = "";
  if(edges.size()==4){

    std::sort(edges.begin(), edges.end(), sortX);
    float xDiff = abs(edges[0].x - edges[1].x);

    std::sort(edges.begin(), edges.end(), sortY);
    float yDiff = abs(edges[0].y - edges[1].y);

    if (xDiff/yDiff >= 0.8f)
      label = "quad";
    else
      label = "rect";
    getOrientation(pts, img, label);

  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  bool found = false;
  try
  {
    centers.clear();
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image_mat = cv_ptr->image;

    Mat canny_output;

    /// Detect edges using canny
    Mat gray = image_mat.clone();
    cvtColor(image_mat, gray, COLOR_BGR2HSV); //Convert the captu frame from BGR to HSV

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

    std::vector<cv::Point> approx;  
    for( int i = 0; i< contours.size(); i++ )
    {

      // Approximate contour with accuracy proportional
      // to the contour perimeter
      cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

      // Skip small or non-convex objects 
      if (std::fabs(cv::contourArea(contours[i])) < 60 || !cv::isContourConvex(approx))
        continue;

      if (approx.size() == 4)
      {
        Scalar color = Scalar( 0, 0, 255 );
        
        drawContours( image_mat, contours, i, color, 2, 8, hierarchy, 0, Point() );
    
        getShape(approx, contours[i], image_mat);

      }
          
    }


    imshow("original", image_mat);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  cv::waitKey(30);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;


  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  //image_transport::Subscriber sub = it.subscribe("/softkinetic_camera/rgb/image_color", 1, imageCallback);
  image_transport::Subscriber sub = it.subscribe("/spqr_camera/rgb", 1, imageCallback);

  chatter_pub = nh.advertise<geometry_msgs::PoseArray>("boxes", 1000);

  ros::spin();
}


