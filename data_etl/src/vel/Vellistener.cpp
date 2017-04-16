
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  std::ostringstream os;
  os << std::setiosflags(std::ios_base::fixed) <<std::setprecision(3) << input->header.stamp.toSec();
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
  ROS_INFO("Saved pcd %s", (os.str() + ".pcd").c_str());
  pcl::io::savePCDFileASCII(os.str() + ".pcd",cloud);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pcl_image_saver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  std::string topic = nh.resolveName("pcl");
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (topic, 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

  // Spin
  ros::spin ();
}
