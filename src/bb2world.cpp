#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include "utils.h"
#include <message_filters/subscriber.h>
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point32.h"
#include "spqr_find_patches/bounding_box_list.h"

class BBReader
{
public:

    //Variables
    ros::NodeHandle _nh;
    tf::TransformListener tf_;
    tf::MessageFilter<spqr_find_patches::bounding_box_list> * tf_filter_;
    message_filters::Subscriber<spqr_find_patches::bounding_box_list> bb_sub_;
    ros::Publisher pub_;
    std::string _arm_frame, _camera_frame, _ee_frame;
    double shelf_height_;
    Eigen::Matrix3f K;

    //initialisation
    BBReader()
    {
        _arm_frame="/arm_base";
        _camera_frame="/arm_camera_rgb_optical_frame";
        _ee_frame="/end_effector_edge";
        
        K<<960.687141, 0, 655.97077, 0, 962.020992, 378.270252, 0, 0, 1;

        bb_sub_.subscribe(_nh, "/rotated_bounding_boxes", 1);
        tf_filter_ = new tf::MessageFilter<spqr_find_patches::bounding_box_list>(bb_sub_, tf_, _arm_frame, 10);
        tf_filter_->registerCallback( boost::bind(&BBReader::callback, this, _1) );
        pub_=_nh.advertise<geometry_msgs::PoseArray>("/bb_to_3d_points",1);
        _nh.getParam("/bb2world/shelf_height", shelf_height_);
    }

    //callback
    void callback(const spqr_find_patches::bounding_box_list::ConstPtr& msg)
    {   
      _nh.getParam("/bb2world/shelf_height", shelf_height_);
      geometry_msgs::PoseArray pub_msg;
      pub_msg.header=msg->header;
      pub_msg.header.frame_id=_arm_frame;

      std::vector<Eigen::Vector3f> center_points;
      std::vector<float> center_angles;
      
      Eigen::Affine3f T, T_inv;
      tf::StampedTransform transform;
      try
      {
        //ros::Time now=ros::Time::now();
        //tf_.waitForTransform(_arm_frame, _camera_frame, msg->header.stamp, ros::Duration(.515));
        tf_.lookupTransform( _arm_frame, _camera_frame, msg->header.stamp, transform);
        //tf::transformTFToEigen(transform, T);
        tf2Affine(transform, T);
        T_inv=T.inverse();
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }

      float fx=K(0,0);
      float fy=K(1,1);
      float cx=K(0,2);
      float cy=K(1,2);
      float Z=T.translation()(2)-shelf_height_;
      float X, Y;
      
      for(int i=0; i<msg->boxes.size(); i++)
      {   

        cv::RotatedRect rbb(cv::Point2f(msg->boxes[i].center.x, msg->boxes[i].center.y),
                            cv::Size2f(msg->boxes[i].width, msg->boxes[i].height),
                            msg->boxes[i].angle);

        float angle=rbb.angle;
        float angle_rad=rbb.angle*M_PI/180.f;
        std::vector<Eigen::Vector3f> region_points(4);
        
        Eigen::Matrix3f R_angle=Eigen::Matrix3f(Eigen::AngleAxisf(angle_rad, Eigen::Vector3f(0,0,1)));
        R_angle=T_inv.linear()*R_angle;
        Eigen::Vector3f rpy=R_angle.eulerAngles(0,1,2);

        float angle_transf=rpy(2);
        center_angles.push_back(angle_transf);

        float x=msg->boxes[i].center.x;
        float y=msg->boxes[i].center.y;
        X=(x-cx)*Z/fx;   
        Y=(y-cy)*Z/fy;
        center_points.push_back(Eigen::Vector3f(X,Y,Z));
      }

      for(int i=0; i<center_points.size(); i++)
      {
        center_points[i]=T*center_points[i];
        geometry_msgs::Pose pose;
        pose.position.x=center_points[i](0);
        pose.position.y=center_points[i](1);
        pose.position.z=shelf_height_;
        pose.orientation.z=center_angles[i]+1.57;
        pub_msg.poses.push_back(pose);

        std::cout<<shelf_height_<<std::endl;
      }
      
      pub_.publish(pub_msg);   
    }
    
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bb2world");
  BBReader bb_reader;
  ros::spin();

  return 0;
}
