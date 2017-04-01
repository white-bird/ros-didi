/* \author Bastian Steder */

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

// --------------
// -----Help-----
// --------------
void 
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
            << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
            << "-m           Treat all unseen points to max range\n"
            << "-h           this help\n"
            << "\n\n";
}

void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}
// --------------
// -----Main-----
// --------------
int 
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  if (pcl::console::find_argument (argc, argv, "-m") >= 0)
  {
    setUnseenToMaxRange = true;
    cout << "Setting unseen values in range image to maximum range readings.\n";
  }
  int tmp_coordinate_frame;
  if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
  {
    coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
    cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
  }
  if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
    cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
  angular_resolution = pcl::deg2rad (angular_resolution);
  
  // ------------------------------------------------------------------
  // -----Read pcd file or create example point cloud if not given-----
  // ------------------------------------------------------------------
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  if (!pcd_filename_indices.empty ())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
    {
      cout << "Was not able to open file \""<<filename<<"\".\n";
      printUsage (argv[0]);
      return 0;
    }
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);
  //                       cout << scene_sensor_pose.translation () << "|-1-|" << scene_sensor_pose.linear ().col (0) << "|-2-|" << scene_sensor_pose.linear ().col(1) << "|-3-|" 
  //     << scene_sensor_pose.linear ().col (2) << "|-4-|";
  // scene_sensor_pose.translation () = Eigen::Vector3d(0,0,1).cast<float> ();

    std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
    if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
      std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
  }
  else
  {
    cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
    for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f)
      {
        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud.points.push_back (point);
      }
    }
    point_cloud.width = (int) point_cloud.points.size ();  point_cloud.height = 1;
  }
  
  //filter

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud (point_cloud_ptr);
  // pass.setFilterFieldName ("x");
  // pass.setFilterLimits (-0.2, 0.2);
  // pass.setFilterLimitsNegative (true);
  // pass.filter (*cloud_filtered1);

  // pcl::PassThrough<pcl::PointXYZ> pass2;
  // pass2.setInputCloud (cloud_filtered1);
  // pass2.setFilterFieldName ("y");
  // pass2.setFilterLimits (-0.2, 0.2);
  // pass2.setFilterLimitsNegative (true);
  // pass2.filter (*cloud_filtered2);

  // pcl::PassThrough<pcl::PointXYZ> pass3;
  // pass3.setInputCloud (cloud_filtered2);
  // pass3.setFilterFieldName ("z");
  // pass3.setFilterLimits (0.2, 40.0);
  // pass3.filter (*cloud_filtered3);
  
  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;   
  range_image.createFromPointCloud (*point_cloud_ptr, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges (far_ranges);

  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();
  std::string filename = argv[pcd_filename_indices[0]];
  std::string ranges_filename = "range_"+filename;

  // boost::shared_ptr<pcl::RangeImage> range_image2_ptr (new pcl::RangeImage);
  // pcl::RangeImage& range_image2 = *range_image2_ptr; 
  // std::vector<int> indices;
  // pcl::removeNaNFromPointCloud(range_image, range_image2, indices);
  // range_image = range_image2;
  pcl::io::savePCDFileASCII (ranges_filename.c_str(), range_image);


  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (1, 1, 1);
  viewer.addCoordinateSystem (1.0f, "global");
  pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 0, 0, 0);
  viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 150, 150, 150);
  // viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
  // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "range image");
  
// -------------------------
  // -----Extract borders-----
  // -------------------------
  // pcl::RangeImageBorderExtractor border_extractor (&range_image);
  // pcl::PointCloud<pcl::BorderDescription> border_descriptions;
  // border_extractor.compute (border_descriptions);
  
  // // ----------------------------------
  // // -----Show points in 3D viewer-----
  // // ----------------------------------
  // pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
  //                                           veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
  //                                           shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
  // pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
  //                                     & veil_points = * veil_points_ptr,
  //                                     & shadow_points = *shadow_points_ptr;
  // for (int y=0; y< (int)range_image.height; ++y)
  // {
  //   for (int x=0; x< (int)range_image.width; ++x)
  //   {
  //     if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
  //       border_points.points.push_back (range_image.points[y*range_image.width + x]);
  //     if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
  //       veil_points.points.push_back (range_image.points[y*range_image.width + x]);
  //     if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
  //       shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
  //   }
  // }
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler (border_points_ptr, 0, 255, 0);
  // viewer.addPointCloud<pcl::PointWithRange> (border_points_ptr, border_points_color_handler, "border points");
  // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler (veil_points_ptr, 255, 0, 0);
  // viewer.addPointCloud<pcl::PointWithRange> (veil_points_ptr, veil_points_color_handler, "veil points");
  // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler (shadow_points_ptr, 0, 255, 255);
  // viewer.addPointCloud<pcl::PointWithRange> (shadow_points_ptr, shadow_points_color_handler, "shadow points");
  // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");
  
  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  
  range_image_widget.showRangeImage (range_image);

  range_image_widget.setSize(1200,560);
  // -------------------------------------

  
  // --------------------
  // -----Main loop-----
  // --------------------

  while (!viewer.wasStopped ())
  {
    range_image_widget.spinOnce ();
    viewer.spinOnce ();
    
    scene_sensor_pose = viewer.getViewerPose(); //获取观测姿势
    

    // range_image.createFromPointCloud (*cloud_filtered3, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
    //                                scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size); //重新生成新的深度图
    
    // range_image_widget.showRangeImage (range_image);
    // range_image_widget.setSize(1200,560);
    pcl_sleep(0.01);

  }
}
  
 