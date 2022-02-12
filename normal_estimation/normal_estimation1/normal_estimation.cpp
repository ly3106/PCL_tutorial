#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

// !!!!!!!!!!!!!!!!!!!
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

//!!!!!!!!!!!!!!
int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // ... read, pass in or create a point cloud ...
  pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()
  //!!!!!!!!!!!!!!!!!
  cout << cloud_normals->points.size () <<", "<< cloud->points.size () << endl;

  // visualize normals  // 可视化法线
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);

  while (!viewer.wasStopped ())
  {
      viewer.spinOnce ();
  }
  return 0;
}

