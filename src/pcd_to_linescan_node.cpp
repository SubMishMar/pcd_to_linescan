#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"

std::string target_frame_;
double tolerance_, min_height_, max_height_;
double angle_min_, angle_max_, angle_increment_;
double scan_time_, range_min_, range_max_;
double inf_epsilon_;
bool use_inf_;

void readParams(ros::NodeHandle &nh) {
  nh.param<std::string>("target_frame", target_frame_, "");
  nh.param<double>("transform_tolerance", tolerance_, 0.01);
  nh.param<double>("min_height", min_height_, std::numeric_limits<double>::min());
  nh.param<double>("max_height", max_height_, std::numeric_limits<double>::max());

  nh.param<double>("angle_min", angle_min_, - M_PI);
  nh.param<double>("angle_max", angle_max_, M_PI);
  nh.param<double>("angle_increment", angle_increment_, M_PI / 180.0);
  nh.param<double>("scan_time", scan_time_, 1.0 / 30.0);
  nh.param<double>("range_min", range_min_, 0.0);
  nh.param<double>("range_max", range_max_, std::numeric_limits<double>::max());
  nh.param<double>("inf_epsilon", inf_epsilon_, 1.0);
  nh.param<bool>("use_inf", use_inf_, true);
}
void pointcloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

}
int main(int argc, char **argv){
  ros::init(argc, argv, "pcd_to_linescan_node");
  ros::NodeHandle n;
  readParams(n);
  ros::Subscriber sub = n.subscribe("cloud_in", 1000, pointcloudCb);
  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan_out", 1000);

  ros::Rate loop_rate(10);

  while(ros::ok()) {

  }
  return 0;
}