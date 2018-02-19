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

bool insideCircle(int p, Mat img){
  int pointsCounter = 0;
  circle(img, centers[p], distanceTreshold, CV_RGB(255, 0, 255), 2);

  for (int i = 0; i < centers.size(); ++i)
  {

    if(i!=p){
      if ( sqrt(pow((centers[i].x-centers[p].x),2) + pow((centers[i].y-centers[p].y),2)) <= distanceTreshold  )            //(x - center_x)^2 + (y - center_y)^2 < radius^2.
        {
         pointsCounter++;
        }

      }
  }
      if (pointsCounter>pointsInsideTreshold)
      return true;
    else 
      return false;
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

    Size size = Size(30,30);
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

    
    RNG rng;


    int best=0;
    Point center;
    /// Draw contours 
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    //Mat new_img = image_mat.clone();
    Mat new_img = Mat::zeros( canny_output.size(), CV_8UC3 );
  
    std::vector<cv::Point> approx;  
    for( int i = 0; i< contours.size(); i++ )
    {



      // Approximate contour with accuracy proportional
      // to the contour perimeter
      cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

      // Skip small or non-convex objects 
      if (std::fabs(cv::contourArea(contours[i])) < 60 || !cv::isContourConvex(approx))
        continue;

      //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );      
      Scalar color = Scalar( 0, 0, 255 );
      if (approx.size() == 4)
        drawContours( image_mat, contours, i, color, 2, 8, hierarchy, 0, Point() );

      /*
      vector<Vec4i> lines;

      HoughLinesP(canny_output, lines, 1, CV_PI/180, threshold_line, min_line_length, max_line_gap);

      for( size_t i = 0; i < lines.size(); i++ )
      {
        Vec4i l = lines[i];
        line( new_img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
      }

      approxPolyDP( Mat(contours[i]), contours_poly_[i], 3, true );
      boundRect[i] = boundingRect( Mat(contours_poly_[i]) );
      */
      /*
      if(boundRect[i].area() > areaTreshold){
            // Find the orientation of each shape
            getCenter(contours[i], image_mat);
            if( insideCircle(i,image_mat) )
              rectangle( image_mat, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );

          }*/
          
    }


    imshow("original", image_mat);
    //imshow("detect", new_img);
    //imshow("canny", canny_output);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  cv::waitKey(30);
}




void colorModifier(){

  
    namedWindow("Control ", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    
    cvCreateTrackbar("areatreshold", "Control ", &areaTreshold, 10000);
    cvCreateTrackbar("distance treshold", "Control ", &distanceTreshold, 100000);
    cvCreateTrackbar("points treshold", "Control ", &pointsInsideTreshold, 20);
    cvCreateTrackbar("threshold_line", "Control ", &threshold_line, 500);    
    cvCreateTrackbar("min_line_length", "Control ", &min_line_length, 500);
    cvCreateTrackbar("max_line_gap", "Control ", &max_line_gap, 500);
    cvCreateTrackbar("th1", "Control ", &th1, 500);
    cvCreateTrackbar("th2", "Control ", &th2, 500);
    cvCreateTrackbar("kernel", "Control ", &kernel, 15);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  colorModifier(); //decommentare per usare la finestra per modificare i valori

  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  //image_transport::Subscriber sub = it.subscribe("/softkinetic_camera/rgb/image_color", 1, imageCallback);
  image_transport::Subscriber sub = it.subscribe("/spqr_camera/rgb", 1, imageCallback);

  chatter_pub = nh.advertise<geometry_msgs::PoseArray>("boxes", 1000);

  ros::spin();
}


