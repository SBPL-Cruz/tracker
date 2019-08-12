#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <set>
#include <fstream>
#include <string>
#include <vector>

// PCL specific includes
#include <pcl/common/centroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

// cylinder fitting
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <visualization_msgs/MarkerArray.h>

using namespace std;
// ros::Publisher pub;
std::vector<ros::Publisher> g_pub_vec;
ros::Publisher g_pub_cyl_cloud;
ros::Publisher  g_pub_cyl_markers;
ros::Publisher  g_pub_pose_markers;
ros::Publisher g_pub_cropped_cloud;
ros::Publisher g_pub_plane_cloud;
ros::Publisher g_pub_pose;
ros::Publisher g_pub_pose_filtered;
std::vector<std::pair<Eigen::Vector4f, int> > g_last_centroids;
tf::StampedTransform g_transform;
bool g_has_transform = false;
int g_next_id = 0;
char target_frame[]="base_footprint";
pcl::PCDWriter writer;
ofstream outfile;
ros::Time start;
ros::Duration t;
int start_record=0;// Flag of starting recording data
double filtered_x;
double filtered_y;
double filtered_z;

tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf) 
{
   tf::Transform tf(sTf.getBasis(),sTf.getOrigin()); //construct a transform using elements of sTf
   //getBasis():Return the basis matrix for the rotation
   //getOrigin():Return the origin vector translation
   return tf;
}

///@{**********Kalman filter structure***********
    typedef  struct{
        double filterValue;  // The filter value at k-1 time k-1时刻的滤波值，即是k-1时刻的值 
        double kalmanGain;   // Kalman增益
        double A;   // x(n)=A*x(n-1)+u(n),u(n)~N(0,Q)
        double B;
        double H;   // z(n)=H*x(n)+w(n),w(n)~N(0,R)
        double P;   // Estimate error covariance 估计误差协方差
        double Q;   // Predicted noise covariance 预测噪声方差 由系统外部测定给定
        double R;   // Measured noise covariance (obtained by experiment data) 测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
        
    }  KalmanInfo;
    /*
    * @brief Init_KalmanInfo   Initial value 初始化滤波器的初始值
    * @param info  Filter pointer 滤波器指针
    */
    void Init_KalmanInfo(KalmanInfo* info, double Q, double R,double filtervalue)
    {
        info->A = 1; 
        info->B = 0; 
        info->H = 1;  
        info->P = 2; 
        info->Q = Q;    
        info->R = R;    
        info->filterValue = filtervalue;// Initial value
    }
    double KalmanFilter(KalmanInfo* kalmanInfo, double lastMeasurement,double A,double B)
    {
        kalmanInfo->A=A;
        kalmanInfo->B=B;

        //Predict the value of the next moment
        double predictValue = kalmanInfo->A* kalmanInfo->filterValue +kalmanInfo->B ;   //x的先验估计由上一个时间点的后验估计值和输入信息给出，此处需要根据基站高度做一个修改
        
        //Finding covariance
        kalmanInfo->P = kalmanInfo->A*kalmanInfo->A*kalmanInfo->P + kalmanInfo->Q;  //计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q
        double preValue = kalmanInfo->filterValue;  //Record the value of the last actual coordinate 记录上次实际坐标的值
    
        // compute kalman gain计算kalman增益
        kalmanInfo->kalmanGain = kalmanInfo->P*kalmanInfo->H / (kalmanInfo->P*kalmanInfo->H*kalmanInfo->H + kalmanInfo->R);  //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)
        // correct results, compute filter value 修正结果，即计算滤波值
        kalmanInfo->filterValue = predictValue + (lastMeasurement - predictValue)*kalmanInfo->kalmanGain;  //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))
        //update estimation 更新后验估计
        kalmanInfo->P = (1 - kalmanInfo->kalmanGain*kalmanInfo->H)*kalmanInfo->P;//计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]
    
        return  kalmanInfo->filterValue;
    }
    KalmanInfo* info_x=new KalmanInfo;
    KalmanInfo* info_y=new KalmanInfo;
    KalmanInfo* info_z=new KalmanInfo;
///@}


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    ros::Time begin = ros::Time::now();
    ros::Duration d;
    if (!ros::ok())
    	return;

  ///@{**********Downsample****************
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_XYZ  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f  (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2::Ptr downsampled (new sensor_msgs::PointCloud2);


    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
  
    // std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
    //         << " data points." << std::endl;
    // Convert to PCL data type
    pcl_conversions::toPCL(*input, *cloud);
    
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (cloud_filtered);
    // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
    pcl::fromPCLPointCloud2(cloud_filtered, *downsampled_XYZ);
    //if (!downsampled_XYZ->points.empty ())
    //{writer.write ("downsample.pcd", *downsampled_XYZ, false);}
    std::cerr << "PointCloud after filtering: " << downsampled_XYZ->width * downsampled_XYZ->height << " data points." << std::endl;
    d = ros::Time::now() - begin;
    ROS_INFO("downsampling at: %f", d.toSec());
  ///@}

  ///@{**********Transform cloud***********
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;

    if (!g_has_transform) {
        tf::TransformListener listener;
        //Block until a transform is possible or it times out
        listener.waitForTransform(target_frame, input->header.frame_id,
                                  ros::Time(0), ros::Duration(10.0));
        //Get the transform between two frames by frame ID                       
        listener.lookupTransform( target_frame, input->header.frame_id, ros::Time(0), g_transform);
    
        g_has_transform = true;
    }
    
    pcl_ros::transformPointCloud(*downsampled_XYZ, transformed_cloud, get_tf_from_stamped_tf(g_transform));
    *downsampled_XYZ  = transformed_cloud;
    d = ros::Time::now() - begin;

    //export transformed cloud
    //if (!downsampled_XYZ->points.empty ())
    //{writer.write ("transform.pcd", *downsampled_XYZ, false);}

    //publish downsampled pointcloud
    #if 0
    
    sensor_msgs::PointCloud2::Ptr output_cropped (new sensor_msgs::PointCloud2);
    pcl::toROSMsg (*downsampled_XYZ, *output_cropped);
    output_cropped->header.frame_id = target_frame;
    g_pub_cropped_cloud.publish(output_cropped);
    #endif
    

    ROS_INFO("transformation at: %f", d.toSec());
  ///@}

  ///@{**********Crop region of interest****
    //box size
    Eigen::Vector4f minPoint; 
    minPoint[0]=0.1;  // define minimum point x 
    minPoint[1]=0.4; // define minimum point y
    minPoint[2]=0.66;  // define minimum point z
    Eigen::Vector4f maxPoint; 
    maxPoint[0]=0.42;  // define max point x 
    maxPoint[1]=1.6;  // define max point y
    maxPoint[2]=0.9;  // define max point z
    
    /* Eigen::Vector4f minPoint; 
    minPoint[0]=0.2;  // define minimum point x 
    minPoint[1]=-0.2; // define minimum point y
    minPoint[2]=0.6;  // define minimum point z
    Eigen::Vector4f maxPoint; 
    maxPoint[0]=1.8;  // define max point x 
    maxPoint[1]=0.2;  // define max point y
    maxPoint[2]=0.9;  // define max point z*/


    // Define translation and rotation ( this is optional) 
    Eigen::Vector3f boxTranslatation; 
    boxTranslatation[0]=0.0;   
    boxTranslatation[1]=0.0;   
    boxTranslatation[2]=0.0;   
    // this moves your cube from (0,0,0)//minPoint to (1,2,3)  // maxPoint is now(6,8,10) 

    Eigen::Vector3f boxRotation; 
    boxRotation[0]= -0.3491;  // rotation around x-axis 
    boxRotation[1]=0;  // rotation around y-axis 
    boxRotation[2]=0.0;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis. 
    // OR:
    // cropFilter.setTransform(boxTransform);
    

    pcl::CropBox<pcl::PointXYZ> cropFilter; 
    cropFilter.setInputCloud (downsampled_XYZ); 
    cropFilter.setMin(minPoint); 
    cropFilter.setMax(maxPoint); 
    // cropFilter.setTranslation(boxTranslatation); 
    //cropFilter.setRotation(boxRotation); 
    cropFilter.filter (*downsampled_XYZ);

    //export cropped cloud
    //if (!downsampled_XYZ->points.empty ())
    //{writer.write ("crop.pcd", *downsampled_XYZ, false);}

    //publish cropped pointcloud
    #if 1
    sensor_msgs::PointCloud2::Ptr output_cropped (new sensor_msgs::PointCloud2);
    pcl::toROSMsg (*downsampled_XYZ, *output_cropped);
    output_cropped->header.frame_id = target_frame;
    g_pub_cropped_cloud.publish(output_cropped);
    #endif

    std::cerr << "PointCloud after crop filtering: " << downsampled_XYZ->width * downsampled_XYZ->height << " data points." << std::endl;
    d = ros::Time::now() - begin;
    ROS_INFO("cropping at: %f", d.toSec());
    
  ///@}

  ///@{**********Clustering****************
    //Create the SACSegmentation object and set the model and method type
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);//For more info: wikipedia.org/wiki/RANSAC
    seg.setMaxIterations (100);

    //planemodified
    seg.setDistanceThreshold (0.05);//How close a point must be to the model to considered an inlier
    //seg.setDistanceThreshold (0.02);

    int i = 0, nr_points = (int) downsampled_XYZ->points.size ();

    //Contains the plane point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    // While 30% of the original cloud is still there
    while (downsampled_XYZ->points.size () > 0.3 * nr_points)
    {
    	if (!ros::ok())
    		break;
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (downsampled_XYZ);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (downsampled_XYZ);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);

        // std::cerr << "PointCloud representing the planar component: " 
        //         << cloud_plane->width * cloud_plane->height << " data points." << std::endl;
        // std::stringstream ss;
        // ss << "table_scene_lms400_plane_" << i << ".pcd";
        // writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        downsampled_XYZ.swap(cloud_f);
        i++;
    }

    #if 1
        //publish plane pointcloud
        sensor_msgs::PointCloud2::Ptr output_plane (new sensor_msgs::PointCloud2);
        pcl::toROSMsg (*cloud_plane, *output_plane);
        output_plane->header.frame_id = target_frame;
        g_pub_plane_cloud.publish(output_plane);
    #endif

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (downsampled_XYZ);
    
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    //ec.setClusterTolerance (0.02); // 2cm
    ec.setClusterTolerance (0.02);//modified
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (500);
    ec.setSearchMethod (tree);
    ec.setInputCloud (downsampled_XYZ);
    ec.extract (cluster_indices);

    //Create a publisher for each cluster
    ros::NodeHandle nh;
    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        std::string topicName = "/pcl_tut/cluster" + boost::lexical_cast<std::string>(i);
        ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> (topicName, 1);
        g_pub_vec.push_back(pub);
    }

    int j = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        cloud_clusters.push_back(cloud_cluster);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (downsampled_XYZ->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        
        //Convert the pointcloud to be used in ROS
        sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
        pcl::toROSMsg (*cloud_cluster, *output);
        output->header.frame_id = target_frame;

        // Publish the data
        g_pub_vec[j].publish (output);
        ++j;
    }
    d = ros::Time::now() - begin;
    ROS_INFO("clustering at: %f", d.toSec());
    ROS_INFO("Num of clusters: %zu", cloud_clusters.size());
  ///@}

  ///@{**********Cylinder******************
    std::vector<Eigen::Vector4f> centroids;
    std::vector<std::pair<Eigen::Vector4f, int> > current_centroid_ids;
    visualization_msgs::MarkerArray ma;
    ar_track_alvar_msgs::AlvarMarkers pose_markers;

    for (int i = 0; i < cloud_clusters.size(); ++i) {
    ///@{   Cylinder fitting
        pcl::PassThrough<pcl::PointXYZ> pass;
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg2; 
        
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
      
        // Datasets
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
        pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
      
        std::cerr << "PointCloud has: " << cloud_clusters[i]->points.size () << " data points." << std::endl;
      
        // Build a passthrough filter to remove spurious NaNs
        pass.setInputCloud (cloud_clusters[i]);
        pass.setFilterFieldName ("z");
        //pass.setFilterLimits (0, 1.5);
        pass.setFilterLimits (0, 1.5);//modified
        pass.filter (*cloud_filtered2);
        std::cerr << "PointCloud after Pass filtering: " << cloud_filtered2->points.size () << " data points." << std::endl;
      
        // Estimate point normals
        ne.setSearchMethod (tree2);
        ne.setInputCloud (cloud_filtered2);
        ne.setKSearch (50);
        ne.compute (*cloud_normals2);
      
        // Create the segmentation object for cylinder segmentation and set all the parameters
        seg2.setOptimizeCoefficients (true);
        seg2.setModelType (pcl::SACMODEL_CYLINDER);
        seg2.setMethodType (pcl::SAC_RANSAC);
        seg2.setNormalDistanceWeight (0.1);
        seg2.setMaxIterations (10000);
        //seg2.setDistanceThreshold (0.05);
        seg2.setDistanceThreshold (0.05);//modified
        seg2.setRadiusLimits (0.02, 0.08);//modified
        seg2.setInputCloud (cloud_filtered2);
        seg2.setInputNormals (cloud_normals2);
      
        // Obtain the cylinder inliers and coefficients
        seg2.segment (*inliers_cylinder, *coefficients_cylinder);
        // std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
    
        // Write the cylinder inliers to disk
        extract.setInputCloud (cloud_filtered2);
        extract.setIndices (inliers_cylinder);
        extract.setNegative (false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());
        extract.filter (*cloud_cylinder);
        sensor_msgs::PointCloud2::Ptr output_cyl (new sensor_msgs::PointCloud2);
      
        //color
        // pcl::PointCloud<pcl::PointXYZRGB> cloud_cylinder_color;
        // pcl::copyPointCloud(*cloud_cylinder, cloud_cylinder_color);
        // uint8_t r = 0, g = 255, b = 0;    // Example: Red color
        // uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      
        // for (size_t i = i; i < cloud_cylinder->points.size(); ++i) {
        //     // output_cyl->points[i].r = 200;
        //     cloud_cylinder_color.points[i].rgb = *reinterpret_cast<float*>(&rgb);
        // }
      
        #if 1
        // publish cylinder inlier cloud
        pcl::toROSMsg (*cloud_cylinder, *output_cyl);
        output_cyl->header.frame_id = target_frame;
        g_pub_cyl_cloud.publish(output_cyl);
        #endif


        if (cloud_cylinder->points.empty ()) {
          std::cerr << "Can't find the cylindrical component." << std::endl;
          *cloud_cylinder = *cloud_filtered2;
          // continue;
        }
        else
        {
          std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
          writer.write ("obtable_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
        }
        // ROS_INFO("Cloud in: %zu, Cloud out: %zu", cloud_filtered2->points.size(), cloud_cylinder->points.size ());
        // if (cloud_cylinder->points.size() * 1.0/ cloud_filtered2->points.size() < 0.5) {
        //     ROS_INFO("Very few inliers so ignoring");
        //     continue;
        // }
    ///@}

    ///@{   Cylinder Centroids
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cylinder, centroid);
        centroids.push_back(centroid);
        
        ROS_INFO("Centroid x: %f  y: %f  z: %f\n", centroid[0], centroid[1], centroid[2]);

        if (start_record==0){start = ros::Time::now();}// Record the time before starting recording
        if ((centroid[1]<1.25) &&(centroid[1]>0.4)){start_record=1;}// Start recording
        
        if (start_record==1)// Save data to .txt file
        {
            t=ros::Time::now()-start;
            //Compute filter value
            filtered_x=KalmanFilter(info_x,centroid[0], 1, 0.00365*d.toSec());
            filtered_y=KalmanFilter(info_y,centroid[1], 1, -0.24*d.toSec());
            filtered_z=KalmanFilter(info_z,centroid[2], 1, 0);
            //outfile.open("/home/chenxiaoyu/Data2/data.txt", ios::binary | ios::app | ios::in | ios::out);
            //outfile<<centroid[0]<<" "<<centroid[1]<<" "<< centroid[2]<<" "<< t.toSec() <<" "<<filtered_x<<" "<<filtered_y<<" "<<filtered_z<<"\n";
            //outfile.close();
            ROS_INFO("Filtered Centroid x: %f  y: %f  z: %f\n", filtered_x, filtered_y, filtered_z);
            geometry_msgs::PoseStamped pos;
            pos.header.frame_id=target_frame;
            pos.header.stamp= ros::Time::now();
            pos.pose.position.x=centroid[0];
            pos.pose.position.y=centroid[1];
            pos.pose.position.z=centroid[2];
            pos.pose.orientation.w=1.0;
            g_pub_pose.publish(pos);
            geometry_msgs::PoseStamped pos_filtered;
            pos_filtered.header.frame_id=target_frame;
            pos_filtered.header.stamp= ros::Time::now();
            pos_filtered.pose.position.x=filtered_x;
            pos_filtered.pose.position.y=filtered_y;
            pos_filtered.pose.position.z=filtered_z;
            pos_filtered.pose.orientation.w=1.0;
            g_pub_pose_filtered.publish(pos_filtered);
        }

    ///@}
    
    ///@{   Cylinder IDs
        double min_dist = 1000000;
        double thresh = 0.2;
        int centroid_id;

        for (size_t cdx = 0; cdx < g_last_centroids.size(); ++cdx) {
            Eigen::Vector4f c = g_last_centroids[cdx].first;
            double dist = sqrt(pow(c[0] - centroid[0], 2) + pow(c[1] - centroid[1], 2) + pow(c[2] - centroid[2], 2));
            if (dist < min_dist) {
                min_dist = dist;
                centroid_id = g_last_centroids[cdx].second;
            }
        }
        ROS_INFO("dist: %f", min_dist);
        if (min_dist > thresh) {
            //find max id
            // int max_id = 0;
            // for (size_t i = 0; i < g_last_centroids.size(); ++i) {
            //     if (g_last_centroids[i].second > max_id) {
            //         max_id = g_last_centroids[i].second;
            //     }
            // }
            // centroid_id = max_id + 1;
            // centroid_id = g_last_centroids.size();
            centroid_id = g_next_id % 10;;
            g_last_centroids.push_back(std::pair<Eigen::Vector4f, int> (centroid, centroid_id));
            ROS_INFO("Created new id: %d", centroid_id);
            g_next_id++;
        }
        else {
            ROS_INFO("Matched old id: %d", centroid_id);
        }

        current_centroid_ids.push_back(std::pair<Eigen::Vector4f, int> (centroid, centroid_id));
    ///@}

    ///@{   Cylinder Markers
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = target_frame;
        marker.ns = "cylinder";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];
        marker.pose.position.z = centroid[2];
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.15;
        if (centroid_id == 0) {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
        }
        else if (centroid_id == 1) {
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;          
        }
        else if (centroid_id == 2) {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;          
        }
        else if (centroid_id == 3) {
            marker.color.r = 0.5f;
            marker.color.g = 0.5f;
            marker.color.b = 0.0f;          
        }
        else if (centroid_id == 4) {
            marker.color.r = 0.5f;
            marker.color.g = 0.0f;
            marker.color.b = 0.5f;          
        }
        else if (centroid_id == 5) {
            marker.color.r = 0.0f;
            marker.color.g = 0.5f;
            marker.color.b = 0.5f;          
        }
        else{
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;          
        } 
        marker.color.a = 1.0f;
        marker.lifetime = ros::Duration(0.2);
        ma.markers.push_back(marker);
    ///@}
    }
    


    // update centroids
    g_last_centroids = current_centroid_ids;

    ROS_INFO("\n %zu Cylinder: ", g_last_centroids.size());
    for (int i = 0; i < g_last_centroids.size(); ++i) {
      ROS_INFO("    id: %d", g_last_centroids[i].second);
    }

    g_pub_cyl_markers.publish(ma);
  ///@}

  ///@{**********Alvar pose markers********
    for (int i = 0; i < current_centroid_ids.size(); ++i) {
        ar_track_alvar_msgs::AlvarMarker pose_marker;
        pose_marker.id = current_centroid_ids[i].second;
        pose_marker.pose.header.stamp = input->header.stamp;
        pose_marker.pose.header.frame_id = target_frame;
        pose_marker.pose.pose.position.x = current_centroid_ids[i].first[0];
        pose_marker.pose.pose.position.y = current_centroid_ids[i].first[1];
        pose_marker.pose.pose.position.z = current_centroid_ids[i].first[2];
        pose_marker.pose.pose.orientation.w = 1.0;
        pose_marker.header.stamp = input->header.stamp;
        pose_marker.header.frame_id = target_frame;
        pose_markers.markers.push_back(pose_marker);
    }
    pose_markers.header.stamp = input->header.stamp;
    pose_markers.header.frame_id = target_frame;
    g_pub_pose_markers.publish(pose_markers);
  ///@}
  
    d = ros::Time::now() - begin;
  
    ROS_INFO("Cycle time %f", d.toSec());
    printf("--------------------------------------------------------------\n\n");
    return;
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "cluster_extraction");
    ros::NodeHandle nh;
   
    // Clear data
    //outfile.open("/home/chenxiaoyu/Data2/data.txt", ios::trunc);
    //outfile.close();
    
    Init_KalmanInfo(info_x, 1e-04, 0.004, 0.269);
    Init_KalmanInfo(info_y, 1e-04, 0.006, 1.25);
    Init_KalmanInfo(info_z, 1e-05, 0.002, 0.8);
    
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/head_camera/depth_registered/points", 1, cloud_cb);
    
    // Create a ROS publisher for the output point cloud
    g_pub_cyl_cloud = nh.advertise<sensor_msgs::PointCloud2> ("cyl_cloud", 100);
    g_pub_cropped_cloud = nh.advertise<sensor_msgs::PointCloud2> ("cropped_cloud", 100);
    g_pub_plane_cloud = nh.advertise<sensor_msgs::PointCloud2> ("plane_cloud", 100);
    g_pub_cyl_markers = nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 100);
    g_pub_pose_markers = nh.advertise<ar_track_alvar_msgs::AlvarMarkers> ("ar_pose_marker", 100);
    g_pub_pose=nh.advertise<geometry_msgs::PoseStamped>("pose",100);
    g_pub_pose_filtered=nh.advertise<geometry_msgs::PoseStamped>("pose_filtered",100);

    // Spin
    ros::spin ();
}