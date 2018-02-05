#include <ros/ros.h>
#include <std_msgs/String.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>

using namespace cv;
using namespace std;

//***********************************
int iLowH = 0;
int iHighH = 46;

int iLowS = 64;
int iHighS = 255;

int iLowV = 88;
int iHighV = 255;

int threshold_line = 50;
int min_line_length = 80;
int max_line_gap = 5;


double orientation;
ros::Publisher chatter_pub;
std::vector<Point> centers;
int areaTreshold = 10000;
int distanceTreshold = 1000000;
int pointsInsideTreshold = 1;

float distanceBetweenTwoPoints(Point a, Point b){
  return (sqrt(pow((a.x-b.x),2.0)+pow((a.y-b.y),2.0)));
}


void drawAllTriangles(Mat& img, const vector< vector<Point> >& contours){
    vector<Point> approxTriangle;
    for(size_t i = 0; i < contours.size(); i++){
        approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.05, true);
        if(approxTriangle.size() == 3){
            drawContours(img, contours, i, Scalar(0, 255, 255), CV_FILLED); // fill GREEN
            vector<Point>::iterator vertex;
            for(vertex = approxTriangle.begin(); vertex != approxTriangle.end(); ++vertex){
                circle(img, *vertex, 3, Scalar(0, 0, 255), 1);
            }
        }
    }
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
  cout<< " p "<<centers[p]<<endl;
  circle(img, centers[p], distanceTreshold, CV_RGB(255, 0, 255), 2);
cout<<"centers "<<centers.size()<<endl;
  for (int i = 0; i < centers.size(); ++i)
  {

    if(i!=p){
      if ( sqrt(pow((centers[i].x-centers[p].x),2) + pow((centers[i].y-centers[p].y),2)) <= distanceTreshold  )            //(x - center_x)^2 + (y - center_y)^2 < radius^2.
        {
         pointsCounter++;
        }

      }
   // cout<<"centers"<<centers.size()<<" point a "<<centers[i]<< " point p "<<centers[p]<<endl;
  //  cout<<" distanceTreshold "<<distanceTreshold<<" dist  "<<(sqrt(pow((centers[i].x-centers[p].x),2) + pow((centers[i].y-centers[p].y),2)))<< endl;

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

    Mat imgHSV;
    Mat grayScale;

    cvtColor(image_mat, imgHSV, COLOR_BGR2HSV); //Convert the captu frame from BGR to HSV
    cvtColor(image_mat, grayScale, COLOR_BGR2GRAY); //Convert the captu frame from BGR to gray

    
    Mat imgThresholded;

    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image 

    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //imshow("Thresholded Image ", imgThresholded); //show the thresholded image


    Mat canny_output;

    /// Detect edges using canny
    Canny( imgThresholded, canny_output, 95, 200, 3 );
    //Canny( imgThresholded, canny_output, 95, 100 );

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
    for( int i = 0; i< contours.size(); i++ )
    {
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      drawContours( image_mat, contours, i, color, 2, 8, hierarchy, 0, Point() );
      color = Scalar( 0, 0, 255 );
      approxPolyDP( Mat(contours[i]), contours_poly_[i], 3, true );
      boundRect[i] = boundingRect( Mat(contours_poly_[i]) );
        //if(boundRect[i].area()>best) {
           // best = boundRect[i].area();
      if(boundRect[i].area() > areaTreshold){
            // Find the orientation of each shape
            getCenter(contours[i], image_mat);
            if( insideCircle(i,image_mat) )
              rectangle( image_mat, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
            

          }
       //     found = true;
      //  }
    }

    vector<Vec4i> lines;
    Mat cdst;

    cvtColor(canny_output, cdst, CV_GRAY2BGR);
    HoughLinesP(canny_output, lines, 1, CV_PI/180, threshold_line, min_line_length, max_line_gap);

    for( size_t i = 0; i < lines.size(); i++ )
    {
      Vec4i l = lines[i];
      line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }
    

    //drawAllTriangles(test,contours);

    //CERCHIO INTORNO AI CENTRI, SE PER N FRAME CI STA ANCORA QUALCOSA CI METTO UNA BOUNDING BOX


   /* Size sizeTr(imgThresholded.cols * 0.4, imgThresholded.rows * 0.4);
    Size sizeIm(image_mat.cols * 0.4, image_mat.rows * 0.4);

    Mat tr;
    Mat img;

    resize(image_mat, img, sizeIm);
    resize(imgThresholded, tr, sizeTr);*/

        // Showing the result
    //imshow("canny_output ",imgThresholded);
    //imshow("original", image_mat);
    imshow("cdst", cdst);
    //imshow("grayScale", grayScale);

      


    /*if(found){
      geometry_msgs::PoseArray posArray;
      geometry_msgs::Pose ;


      .position.x=center.x;
      .position.y=center.y;
      .orientation.z=orientation;


      posArray.poses.push_back();
      cout<<"number of boxes: "<<posArray.poses.size()<<endl;
      //std::cout<<"pose  " << posArray.poses[0<.position<<endl;
      //std::cout<<"orientation  " << posArray.poses[0].orientation<<endl;

      std::cout<<" "<<posArray.poses[0]<<std::endl;
      chatter_pub.publish(posArray);
    }*/


  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  cv::waitKey(30);
  //  cv::waitKey(0);

}




void colorModifier(){

  
    namedWindow("Control ", CV_WINDOW_AUTOSIZE); //create a window called "Control"

//***********color mod**********
 /*   cvCreateTrackbar("LowH", "Control ", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control ", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control ", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control ", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control ", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control ", &iHighV, 255);/*/
//******************************
    
    cvCreateTrackbar("areatreshold", "Control ", &areaTreshold, 10000);
    cvCreateTrackbar("distance treshold", "Control ", &distanceTreshold, 100000);
    cvCreateTrackbar("points treshold", "Control ", &pointsInsideTreshold, 20);
    cvCreateTrackbar("threshold_line", "Control ", &threshold_line, 500);    
    cvCreateTrackbar("min_line_length", "Control ", &min_line_length, 500);
    cvCreateTrackbar("max_line_gap", "Control ", &max_line_gap, 500);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  colorModifier(); //decommentare per usare la finestra per modificare i valori

  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/softkinetic_camera/rgb/image_color", 1, imageCallback);
///camera/rgb/image_color

  chatter_pub = nh.advertise<geometry_msgs::PoseArray>("boxes", 1000);

  ros::spin();
}


