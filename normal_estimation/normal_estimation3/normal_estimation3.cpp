#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

// !!!!!!!!!!!!!!!!!!!
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

//!!!!!!!!!!!!!!
int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);

  // ... read, pass in or create a point cloud ...!!!!!!!!!!!!
  pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);

  // ... create a downsampled version of it ...!!!!!!!!!!!!!!!
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*cloud_downsampled);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_downsampled);

  // Pass the original data (before downsampling) as the search surface
  ne.setSearchSurface (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given surface dataset.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud_downsampled->points.size ()
  //!!!!!!!!!!!!!!!!!
  cout << cloud_normals->points.size () <<", "<< cloud_downsampled->points.size () << endl;

  // visualize normals  // 可视化法线
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_downsampled, cloud_normals);//!!!!!!!!!!

  while (!viewer.wasStopped ())
  {
      viewer.spinOnce ();
  }
  return 0;
}

