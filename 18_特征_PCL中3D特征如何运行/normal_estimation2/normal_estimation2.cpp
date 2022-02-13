#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

// #include <pcl/point_cloud.h> //pcl_base.h> //common/boost.h>


int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // ... read, pass in or create a point cloud ...
  pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);

  // Create a set of indices to be used. For simplicity, we're going to be using the first 10% of the points in cloud
  std::vector<int> indices (std::floor (cloud->points.size () / 10));
  for (std::size_t i = 0; i < indices.size (); ++i) indices[i] = i;

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Pass the indices
  // pcl !!!!!!!!!!!!!
  boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));
  ne.setIndices (indicesptr);

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

  // cloud_normals->points.size () should have the same size as the input indicesptr->size ()
  cout << cloud_normals->points.size () <<", "<< cloud->points.size () << endl;

  // !!!!!!!!!!!!!!!!!
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_query (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::size_t i = 0; i < indices.size (); ++i) cloud_query->push_back(cloud->points[i]);

  // visualize normals  // 可视化法线
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_query, cloud_normals);

  while (!viewer.wasStopped ())
  {
      viewer.spinOnce ();
  }
  return 0;
}

