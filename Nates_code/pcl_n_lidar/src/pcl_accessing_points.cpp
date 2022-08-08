/* FOR GETTING COORDS IN PCL VISUALIZER
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
   std::cout << "Picking event active" << std::endl;
   if(event.getPointIndex() != -1)
   {
       float x, y, z;
       event.getPoint(x, y, z);
       std::cout << x << "; " << y << "; " << z << std::endl;
   }
}
*/

/* FOR ESTIMATING SURFACE NORMALS OF A DOWNSAMPLE
#include <pcl/point_types.h>
 2#include <pcl/features/normal_3d.h>
 3
 4{
 5  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 6  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
 7
 8  ... read, pass in or create a point cloud ...
 9
10  ... create a downsampled version of it ...
11
12  // Create the normal estimation class, and pass the input dataset to it
13  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
14  ne.setInputCloud (cloud_downsampled);
15
16  // Pass the original data (before downsampling) as the search surface
17  ne.setSearchSurface (cloud);
18
19  // Create an empty kdtree representation, and pass it to the normal estimation object.
20  // Its content will be filled inside the object, based on the given surface dataset.
21  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
22  ne.setSearchMethod (tree);
23
24  // Output datasets
25  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
26
27  // Use all neighbors in a sphere of radius 3cm
28  ne.setRadiusSearch (0.03);
29
30  // Compute the features
31  ne.compute (*cloud_normals);
32
33  // cloud_normals->size () should have the same size as the input cloud_downsampled->size ()
34}
*/

//GUESSWORK
//cloud_name.points.x 
//cloud_name.points.ranges?