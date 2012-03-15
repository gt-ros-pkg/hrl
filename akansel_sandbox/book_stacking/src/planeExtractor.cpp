#include <plane_extractor.h>

geometry_msgs::Point32 calcCentroid(const pcl::PointCloud<Point>& cloud)
{
  geometry_msgs::Point32 centroid;
  centroid.x = 0.0;
  centroid.y = 0.0;
  centroid.z = 0.0;
  
  for(size_t i = 0; i < cloud.points.size(); i++){
    centroid.x += cloud.points[i].x;
    centroid.y += cloud.points[i].y;
    centroid.z += cloud.points[i].z;
  }
  
  centroid.x /= cloud.points.size();
  centroid.y /= cloud.points.size();
  centroid.z /= cloud.points.size(); 
  
  return centroid;
}


book_stacking_msgs::ObjectInfos getObjectsOverPlane(const book_stacking_msgs::PlaneInfo& plane, const pcl::PointCloud<Point>& cloud,double prism_min_height, double prism_max_height)
{
  book_stacking_msgs::ObjectInfos objects_msg;

  
  //Pull points from the plane's prism out
  pcl::PointIndices prism_indices;
  pcl::ExtractPolygonalPrismData<Point> prism_extract;
  prism_extract.setHeightLimits(prism_min_height,prism_max_height);
  prism_extract.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(cloud));
  pcl::PointCloud<Point> plane_hull;
  pcl::fromROSMsg(plane.hull,plane_hull);
  prism_extract.setInputPlanarHull(boost::make_shared<pcl::PointCloud<Point> >(plane_hull));
  prism_extract.segment(prism_indices);

  pcl::PointCloud<Point> prism_cloud;
  pcl::ExtractIndices<Point> extract_prism_indices;
  extract_prism_indices.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(cloud));
  extract_prism_indices.setIndices(boost::make_shared<const pcl::PointIndices>(prism_indices));
  extract_prism_indices.filter(prism_cloud);
  
  if(prism_cloud.points.size() < 20){
    ROS_WARN("getObjectsOverPlane: Not enough points in prism.");
    return objects_msg;
  }

  //cluster the resulting points from the plane's prism
  pcl::EuclideanClusterExtraction<Point> clusterer;
  clusterer.setClusterTolerance(0.05);
  clusterer.setMinClusterSize(100);
  clusterer.setMaxClusterSize(8000);
/*
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (boost::make_shared<pcl::PointCloud<Point> >(prism_cloud));
	clusterer.setSearchMethod (tree);
*/
  std::vector<pcl::PointIndices> clusters;
  clusterer.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(prism_cloud));
  clusterer.extract(clusters);
  
  for(size_t i = 0; i < clusters.size(); i++){
   book_stacking_msgs::ObjectInfo object_msg;
    pcl::PointCloud<Point> object_cloud;
    pcl::copyPointCloud(prism_cloud,clusters[i],object_cloud);
    
    Eigen::Vector4f min_point;
    Eigen::Vector4f max_point;
    pcl::getMinMax3D(prism_cloud,clusters[i],min_point,max_point);
    //double width = max_point.x() - min_point.x();
    //double length = max_point.y() - min_point.y();
    
    sensor_msgs::PointCloud2 object_cloud_msg;
    pcl::toROSMsg(object_cloud,object_cloud_msg);
    object_cloud_msg.header = cloud.header;
    object_msg.header = cloud.header;
    object_msg.cloud = object_cloud_msg;
    geometry_msgs::Point32 min_pt;
    min_pt.x = min_point.x();
    min_pt.y = min_point.y();
    min_pt.z = min_point.z();
    geometry_msgs::Point32 max_pt;
    max_pt.x = max_point.x();
    max_pt.y = max_point.y();
    max_pt.z = max_point.z();
    object_msg.bbox_min = min_pt;
    object_msg.bbox_max = max_pt;
    object_msg.centroid = calcCentroid(object_cloud);
    object_msg.grasp_width = max_point.y() - min_point.y();
    objects_msg.objects.push_back(object_msg);
    
    

  }
  
  return objects_msg;
}


void drawObjectPrisms(const book_stacking_msgs::ObjectInfos& objects, const ros::Publisher& object_pub, const book_stacking_msgs::PlaneInfo& plane, float r, float g, float b)
{
  visualization_msgs::Marker marker;
  marker.header = plane.header;
  marker.ns = "object_prisms";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5;
  marker.scale.x = 0.02;
  marker.lifetime = ros::Duration(60.0);

  for(size_t i = 0; i < objects.objects.size(); i++){
    pcl::PointCloud<Point> object_cloud;
    pcl::PointCloud<Point> object_cloud_projected;
    pcl::PointCloud<Point> object_hull;
    pcl::fromROSMsg(objects.objects[i].cloud,object_cloud);
    
    pcl::ProjectInliers<Point> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(object_cloud));
    proj.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(plane.model));
    //setIndices
    proj.filter(object_cloud_projected);

    pcl::ConvexHull<Point> hull;
    hull.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(object_cloud_projected));
    hull.reconstruct(object_hull);

    Eigen::Vector4f min_point;
    Eigen::Vector4f max_point;
    pcl::getMinMax3D(object_cloud,min_point,max_point);
    
    for(size_t j = 0; j < object_hull.points.size()-1; j++){
      //for each point in the hull, we want to make several line segments
      //Seg for bottom of prism
      geometry_msgs::Point pt1;
      pt1.x = object_hull.points[j].x;
      pt1.y = object_hull.points[j].y;
      pt1.z = object_hull.points[j].z;
      geometry_msgs::Point pt2;
      pt2.x = object_hull.points[j+1].x;
      pt2.y = object_hull.points[j+1].y;
      pt2.z = object_hull.points[j+1].z;

      marker.points.push_back(pt1);
      marker.points.push_back(pt2);

      //Seg for top of prism
      geometry_msgs::Point pt3;
      pt3.x = object_hull.points[j].x;
      pt3.y = object_hull.points[j].y;
      pt3.z = max_point.z();

      geometry_msgs::Point pt4;
      pt4.x = object_hull.points[j+1].x;
      pt4.y = object_hull.points[j+1].y;
      pt4.z = max_point.z();

      marker.points.push_back(pt3);
      marker.points.push_back(pt4);
      
      //Seg for bottom vertices to top vertices
      marker.points.push_back(pt1);
      marker.points.push_back(pt3);

    }

    
  }
  object_pub.publish(marker);
}

void drawPlaneMarkers(const std::vector<book_stacking_msgs::PlaneInfo>& planes, const ros::Publisher& plane_pub, float r, float g, float b)
{
  book_stacking_msgs::PlaneInfos planeinfos;
  planeinfos.planes = planes;
  drawPlaneMarkers(planeinfos,plane_pub,r,g,b);
}
void drawPlaneMarkers(const book_stacking_msgs::PlaneInfos& planes, const ros::Publisher& plane_pub, float r, float g, float b)
{
//ROS_INFO("# of planes: %d",(int)(planes.planes.size()));

  visualization_msgs::MarkerArray markers;
  for (size_t i = 0; i < planes.planes.size(); i++){
    
      pcl::PointCloud<Point> hull_pts;
      pcl::fromROSMsg(planes.planes[i].hull,hull_pts);
      visualization_msgs::Marker marker;
      marker.header = planes.planes[i].header;
      marker.ns = "plane_hulls";
      marker.id = i;
      marker.type = visualization_msgs::Marker::LINE_STRIP;//SPHERE_LIST;//LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.color.r = r;//0.0f;
      marker.color.g = g;//1.0f;
      marker.color.b = b;//0.0f;
      marker.color.a = 0.5;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.lifetime = ros::Duration(60.0*10.0);

      for(size_t j = 0; j < hull_pts.points.size(); j++){
          geometry_msgs::Point pt;
          pt.x = hull_pts.points[j].x;
          pt.y = hull_pts.points[j].y;
          pt.z = hull_pts.points[j].z;

          marker.points.push_back(pt);
      }
      geometry_msgs::Point pt;
      pt.x = hull_pts.points[0].x;
      pt.y = hull_pts.points[0].y;
      pt.z = hull_pts.points[0].z;
      marker.points.push_back(pt);
      markers.markers.push_back(marker);
  }
  plane_pub.publish(markers);
  
}

book_stacking_msgs::PlaneInfos getPlanesByNormals(const pcl::PointCloud<Point>& cloud,
					   unsigned int max_planes, bool cluster_planes,
					   bool use_concave, bool use_omp, 
					   double dist_thresh, 
					   double max_sac_iterations, 
					   double sac_probability, 
					   unsigned int min_inliers, 
					   double search_radius, 
					   double cluster_tolerance)
{
  //int min_inliers = 1000;
  //ROS_INFO("Extracting planes from cloud with %u points",(unsigned int)cloud.points.size());
  book_stacking_msgs::PlaneInfos all_planes;
  pcl::toROSMsg(cloud,all_planes.full_cloud);

  bool more_planes = true;
  pcl::PointCloud<Point> remaining_cloud = cloud;
 
  //pcl::IntegralImageNormalEstimation ne;
  //pcl::PointCloud<pcl::Normal> normals;
  //ne.compute(cloud, normals, 0.02f, 10.0f, ne.AVERAGE_3D_GRADIENT);

  pcl::PointCloud<pcl::Normal> normals;
  if(use_omp){
    pcl::NormalEstimationOMP<Point,pcl::Normal> ne;
    ne.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(cloud));
    pcl::KdTreeFLANN<Point>::Ptr tree (new pcl::KdTreeFLANN<Point>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(search_radius);//(0.1);//0.5
    ne.compute(normals);
  } else {
    pcl::NormalEstimation<Point,pcl::Normal> ne;
    ne.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(cloud));
    pcl::KdTreeFLANN<Point>::Ptr tree (new pcl::KdTreeFLANN<Point>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(search_radius);//(0.1);//0.5
    ne.compute(normals);
  }

  pcl::PointCloud<pcl::Normal> remaining_normals = normals;
  
  while(more_planes && (all_planes.planes.size() < max_planes) && (remaining_cloud.points.size() > min_inliers)){
    //ROS_INFO("Remaining cloud: %u", (unsigned int) remaining_cloud.points.size());
    
    pcl::PointIndices plane_inliers;
    pcl::ModelCoefficients plane_coefficients;
    
    //Segment
    pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
    seg_.setDistanceThreshold(dist_thresh);//(0.03);//0.04
    seg_.setMaxIterations(max_sac_iterations);//(1000);
    seg_.setNormalDistanceWeight(0.1);
    seg_.setOptimizeCoefficients(true);
    seg_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg_.setMethodType(pcl::SAC_RANSAC);
    seg_.setProbability(sac_probability);//(0.99);
    seg_.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(remaining_cloud));
    seg_.setInputNormals(boost::make_shared<pcl::PointCloud<pcl::Normal> >(remaining_normals));
    seg_.segment(plane_inliers,plane_coefficients);
    
    if(plane_coefficients.values[3] < 0.0){
      plane_coefficients.values[0] = -plane_coefficients.values[0];
      plane_coefficients.values[1] = -plane_coefficients.values[1];
      plane_coefficients.values[2] = -plane_coefficients.values[2];
      plane_coefficients.values[3] = -plane_coefficients.values[3];
    }

    if(plane_inliers.indices.size() < min_inliers){
      more_planes = false;
    } else {

      //Pull out the inliers
      pcl::ExtractIndices<Point> get_plane_cloud;
      pcl::PointCloud<Point> plane_cloud;
      get_plane_cloud.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(remaining_cloud));
      get_plane_cloud.setIndices(boost::make_shared<pcl::PointIndices>(plane_inliers));
      get_plane_cloud.setNegative(false);
      get_plane_cloud.filter(plane_cloud);
    
      pcl::EuclideanClusterExtraction<Point> plane_cluster;
      plane_cluster.setClusterTolerance(cluster_tolerance);
      plane_cluster.setMinClusterSize(min_inliers);
      std::vector<pcl::PointIndices> clusters;
      plane_cluster.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(plane_cloud));
      plane_cluster.extract(clusters);

      if(clusters.size() == 0){
	more_planes = false;
      }

      for(size_t i = 0; i < clusters.size(); i++){
	pcl::PointCloud<Point> plane_clust_cloud;
	pcl::copyPointCloud(plane_cloud,clusters[i],plane_clust_cloud);
	
	book_stacking_msgs::PlaneInfo p;    
	p.header = cloud.header;
	pcl::toROSMsg(plane_clust_cloud,p.cloud);
	p.model = plane_coefficients;
	p.inliers = clusters[i];
	p.normal[0] = plane_coefficients.values[0];
	p.normal[1] = plane_coefficients.values[1];
	p.normal[2] = plane_coefficients.values[2];
	p.normal[3] = plane_coefficients.values[3];
	
	//ROS_INFO("normal: %lf %lf %lf %lf",plane_coefficients.values[0],plane_coefficients.values[1],
	//		 plane_coefficients.values[2],plane_coefficients.values[3]);
	
	//make a hull
	pcl::PointCloud<Point> projected_plane;
	pcl::ProjectInliers<Point> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(plane_cloud));
	proj.setIndices(boost::make_shared<pcl::PointIndices>(clusters[i]));
	proj.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(plane_coefficients));
	proj.filter(projected_plane);
	
	pcl::PointCloud<Point> plane_hull;
	if(use_concave){
	  std::vector<pcl::Vertices> pgons;
	  pcl::ConcaveHull<Point> hull;                                        
          hull.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(projected_plane));
	  hull.setAlpha(0.1);                             
          hull.reconstruct(plane_hull,pgons);          
          pcl::toROSMsg(plane_hull,p.hull);
	} else {
	  pcl::ConvexHull<Point> hull;
	  hull.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(projected_plane));
	  hull.reconstruct(plane_hull);
	  pcl::toROSMsg(plane_hull,p.hull);
	}

	ROS_INFO("Got plane with %u points",(unsigned int) p.inliers.indices.size());
	
	all_planes.planes.push_back(p);

	//remove the points
	pcl::PointCloud<Point> plane_removed_cloud;
	pcl::ExtractIndices<Point> extract_plane_indices;
	extract_plane_indices.setNegative(true);
	extract_plane_indices.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(remaining_cloud));
	extract_plane_indices.setIndices(boost::make_shared<const pcl::PointIndices>(plane_inliers));
	extract_plane_indices.filter(plane_removed_cloud);
	remaining_cloud = plane_removed_cloud;
	
	//remove the normals too
	pcl::PointCloud<pcl::Normal> plane_removed_normals;
	pcl::ExtractIndices<pcl::Normal> extract_normal_indices;
	extract_normal_indices.setNegative(true);
	extract_normal_indices.setInputCloud(boost::make_shared<const pcl::PointCloud<pcl::Normal> >(remaining_normals));
	extract_normal_indices.setIndices(boost::make_shared<const pcl::PointIndices>(plane_inliers));
	extract_normal_indices.filter(plane_removed_normals);
	remaining_normals = plane_removed_normals;
	
	//ROS_INFO("remaining cloud now has: %u", (unsigned int) remaining_cloud.points.size());

      }
      
    }



  }

  return all_planes;
}
