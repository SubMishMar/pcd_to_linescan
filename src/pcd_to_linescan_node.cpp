#include "ros/ros.h"
#include "ros/console.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

class scan_publisher {
private:
  double tolerance_, min_height_, max_height_;
  double angle_min_, angle_max_, angle_increment_;
  double scan_time_, range_min_, range_max_;
  double inf_epsilon_;
  bool use_inf_;

  sensor_msgs::LaserScan output;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

public:
  scan_publisher(ros::NodeHandle &n);
  void readParams(ros::NodeHandle &n);
  void pointcloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
};

scan_publisher::scan_publisher(ros::NodeHandle &n) {
  nh_ = n;
  sub_ = nh_.subscribe("cloud_in", 100, &scan_publisher::pointcloudCb, this);
  pub_ = nh_.advertise<sensor_msgs::LaserScan>("camera_link", 100);
  readParams(nh_);  
}

void scan_publisher::readParams(ros::NodeHandle &nh) {
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

void scan_publisher::pointcloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  output.header = cloud_msg->header;
  output.header.frame_id = "camera_de";
  output.angle_min = angle_min_;
  output.angle_max = angle_max_;
  output.angle_increment = angle_increment_;
  output.time_increment = 0.0;
  output.scan_time = scan_time_;
  output.range_min = range_min_;
  output.range_max = range_max_;

  //determine amount of rays to create
  uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

  //determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  }
  else {
    output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
  }

  sensor_msgs::PointCloud2ConstPtr cloud_out;
  cloud_out = cloud_msg;

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float>
            iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"), iter_z(*cloud_out, "z");
            iter_x != iter_x.end();
            ++iter_x, ++iter_y, ++iter_z) {

    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      printf("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_)
    {
      printf("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_)
    {
      printf("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x, *iter_y,
                    *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < output.angle_min || angle > output.angle_max)
    {
      printf("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
      continue;
    }

      //overwrite range at laserscan ray if new range is smaller
    int index = (angle - output.angle_min) / output.angle_increment;
    if (range < output.ranges[index])
    {
      output.ranges[index] = range;
    }
  }
  pub_.publish(output);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pcd_to_linescan_node");
  ros::NodeHandle nh;
  scan_publisher scan_pub(nh);
  ros::spin();
  return 0;
}