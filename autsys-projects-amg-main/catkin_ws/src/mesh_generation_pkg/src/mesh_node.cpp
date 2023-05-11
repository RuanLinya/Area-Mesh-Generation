/*This node is to generate Mesh. 
Process: 1. subscribe left and right images
         2. calculate disparity using SGBM algorithm
         3. generate point cloud from disparity using camera parameters
         4. merge point cloud in world frame
         5. preprocessing point cloud(downsampling, outlier filter)
         6. generate triangle and poisson mesh from point cloud
         7. Visualization of Mesh on height
          */

#include <opencv2/opencv.hpp>
#include<iostream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/make_shared.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h> 
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/surface/gp3.h> 
#include <pcl/visualization/pcl_visualizer.h>

#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <cstdlib>
#include <string>
#include <stereo_msgs/DisparityImage.h>
#include <geometry_msgs/PoseStamped.h>

using namespace message_filters;
using namespace std;

// define opencv window to show images;
static const std::string OPENCV_WINDOW1 = "left_image";
static const std::string OPENCV_WINDOW2 = "right_image";
static const std::string OPENCV_WINDOW3 = "Disparity";

class mesh_generation
{
    ros::NodeHandle nh_;
    //subscribe left and right images and process them in one callback
    message_filters::Subscriber<sensor_msgs::Image> image_left_sub;
    message_filters::Subscriber<sensor_msgs::Image> image_right_sub;
    TimeSynchronizer<sensor_msgs::Image,sensor_msgs::Image> *sync;

    ros::Subscriber point_cloud_sub;
    ros::Publisher pc_publisher;
    ros::Subscriber current_state;

    tf::TransformListener listener;
    pcl::PointCloud<pcl::PointXYZ> cloud_sum; 
    pcl::PointCloud<pcl::PointXYZ> cloud_total;
    std::string taget_frame;
    std::string folder_path;
    std::string file_path;
    std::vector<double> cloudx, cloudy;
    ros::Time time_st;

    double camera_cx;
    double camera_cy;
    double camera_fx;
    double camera_fy;
    double b = -0.2; //baseline
    double height, vx, vy, vz, wx, wy, wz, X, Y;
    int i, m;
    bool flag1, flag2;
    double goalx, goaly, goalz;


  public:
    mesh_generation()
    {
      cv::namedWindow(OPENCV_WINDOW1);
      cv::namedWindow(OPENCV_WINDOW2);
      cv::namedWindow(OPENCV_WINDOW3);
      image_left_sub.subscribe(nh_, "/realsense/rgb/left_image_raw", 1);
      image_right_sub.subscribe(nh_, "/realsense/rgb/right_image_raw", 1);
      sync = new TimeSynchronizer<sensor_msgs::Image,sensor_msgs::Image>(image_left_sub, image_right_sub, 1);
      sync->registerCallback(boost::bind(&mesh_generation::callback, this, _1, _2));
      pc_publisher = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud_sum", 1);
      current_state = nh_.subscribe("current_state_est", 1, &mesh_generation::onCurrentState, this);
      taget_frame = "world";
      folder_path = ros::package::getPath("mesh_generation_pkg");
      camera_cx = 160;    // width/2
      camera_cy = 120;    // height/2
      camera_fx = 160;    // FOV = 2*actan(0.5*width / focal) --> fx = 0.5*width/tan(FOV/2)           
      camera_fy = 160;    // FOV = 2*actan(0.5*heigth / focal) --> fy = 0.5*height/tan(FOV/2),   FOV=90
      i = 0;
      m = 0;
      flag1 = false;
      flag2 = false;
      goalx = 16;
      goaly = 4*2;
      goalz = 3.5;
    }

  ~mesh_generation(){
    cv::destroyWindow(OPENCV_WINDOW1);
    cv::destroyWindow(OPENCV_WINDOW2);
    cv::destroyWindow(OPENCV_WINDOW3);
  }

  //----current state of drone--------
  void onCurrentState(const nav_msgs::Odometry& cur_state)
  {
    X = cur_state.pose.pose.position.x;
    Y = cur_state.pose.pose.position.y;
    height = cur_state.pose.pose.position.z;
    vx = cur_state.twist.twist.linear.x;
    vy = cur_state.twist.twist.linear.y;
    vz = cur_state.twist.twist.linear.z;
    wx = cur_state.twist.twist.angular.x;
    wy = cur_state.twist.twist.angular.y;
    wz = cur_state.twist.twist.angular.z;

    //determine if drone moved or not
    if((abs(vx)>0.8)||(abs(vy)>0.8)||(abs(vz)>0.8))
    {
      flag1 = false;
      flag2 = false;
    }
    if((abs(vx)<0.2)&&(abs(vy)<0.2)&&(abs(vz)<0.2)&&(flag1==false)&&(abs(wz)<0.1)&&(abs(wy)<0.1)&&(abs(wx)<0.1))
    {
    	flag1 = true;
	    ros::Duration(3).sleep();
    }
  }


  void callback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
  {  
    time_st = ros::Time::now();  
    tf::StampedTransform transform;
    listener.lookupTransform("camera", "world", ros::Time(0), transform);
     
    //---------------cv_bridge convert ros message to image----------------------
    cv_bridge::CvImagePtr image_left;
    try
    {
      image_left = cv_bridge::toCvCopy(msg_left, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_bridge::CvImagePtr image_right;
    try
    {
      image_right = cv_bridge::toCvCopy(msg_right, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //---------------show left and right image------------------------------------    
    cv::imshow(OPENCV_WINDOW1, image_left->image);
    cv::moveWindow(OPENCV_WINDOW1, 100, 100);
    cv::waitKey(3);
    cv::imshow(OPENCV_WINDOW2, image_right->image);
    cv::moveWindow(OPENCV_WINDOW2, 430, 100);
    cv::waitKey(3);

    //------generate point cloud if drone stop at one of the waypoints-------------
    if((abs(vx)<0.2)&&(abs(vy)<0.2)&&(abs(vz)<0.2)&&(abs(wz)<0.1)&&(abs(wy)<0.1)&&(abs(wx)<0.1))
    {
      if(flag2==false)
      {
        ROS_INFO("I arrived one of the waypoints");

        //compute and show 'disparity' using SGBM algorithm
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,96,9,8*9*9,32*9*9,1,63,10,100,32);
        cv::Mat disparity_sgbm, disparity;
        cv::Mat left;
        cv::Mat right;
        cv::cvtColor(image_left->image, left, cv::COLOR_BGR2GRAY);
        cv::cvtColor(image_right->image, right, cv::COLOR_BGR2GRAY);
        sgbm->compute(left, right, disparity_sgbm);
        disparity_sgbm.convertTo(disparity, CV_32F, 1.0/16.0f);
        cv::imshow(OPENCV_WINDOW3, disparity/96);
        cv::moveWindow(OPENCV_WINDOW3, 830, 100);
        cv::waitKey(3);

        //generate point cloud from disparity
        pcl::PointCloud<pcl::PointXYZ> cloud; 
        for (int v = 0; v < left.rows; v++){
            for (int u = 0; u < left.cols; u++)
            {
                if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0) continue;   
                pcl::PointXYZ p;
                double x = (u - camera_cx) / camera_fx;
                double y = (v - camera_cy) / camera_fy;
                double depth = camera_fy * b / (disparity.at<float>(v, u)); //+ 0.0001;
                p.x = x * depth;
                p.y = y * depth;
                p.z = depth;
                cloud.push_back(p); 
            } 
        }

        //transform point cloud to world frame
        cloud.header.stamp = time_st.toNSec()/1e3;
        cloud.header.frame_id = "camera";

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());        
        pcl_ros::transformPointCloud("world", cloud, *pcl_cloud, listener);     

        cloud_sum.header.stamp = cloud.header.stamp;
        cloud_sum.header.frame_id = "world";
        cloud_sum = cloud_sum + *pcl_cloud;
        //remove nan
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(cloud_sum, cloud_sum, indices);
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud_sum, output);
        pc_publisher.publish(output);
        flag2 = true; 

        //save point cloud to .pcd files
        if(i== 9) 
        {        
          file_path = folder_path + "/p_" + std::to_string(m) +".pcd";
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum_preprossed(new pcl::PointCloud<pcl::PointXYZ>());
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum_pointer(new pcl::PointCloud<pcl::PointXYZ>());
          *cloud_sum_pointer = cloud_sum;
          PreProcessingOfPointCloud(cloud_sum_pointer, cloud_sum_preprossed);
          pcl::io::savePCDFileASCII(file_path, *cloud_sum_preprossed);
          ROS_INFO("pcd file saved");
          cloud_sum.clear();
          i = 0;
          m = m + 1;
        }
        i = i + 1;  
      }  
    }
    

    //------load pcd file and concatenate point clouds if drone arrive at the goal position-------------
     if (m==1) 
     {
       ROS_INFO("merge saved point clouds");
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pointer(new pcl::PointCloud<pcl::PointXYZ>());
       for (int k = 0; k < m; k++) // K:number of waypoints;
       {
        file_path = folder_path + "/p_" + std::to_string(k) + ".pcd";
      //  file_path = folder_path + "/p_mix.pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud_pointer)==-1)
        {
          PCL_ERROR("Couldn't read .pcd file \n");
        }
        else
        {
          ROS_INFO("successfully load pcd file");
        }
        cloud_total = cloud_total + *cloud_pointer;
      }

    //-------visualize the total point clouds------------------- 
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_total_pointer(new pcl::PointCloud<pcl::PointXYZ>());
      *cloud_total_pointer = cloud_total;
      //rgb cloud for height visualization
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_height(new pcl::PointCloud<pcl::PointXYZRGB>()); 
      pcl_height = pointcloudVisualizer(cloud_total_pointer);
      // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_1 (new pcl::visualization::PCLVisualizer ("point cloud Viewer"));
      // viewer_1->setBackgroundColor (0, 0, 0);
      // viewer_1->addPointCloud(cloud_total_pointer);
      // viewer_1->addCoordinateSystem (1.0);
      // viewer_1->initCameraParameters ();
      // viewer_1->setCameraPosition(50, 50, 50, -1, -1, -1, 0, -1, 0);
      // while (!viewer_1->wasStopped())
      // {
      //   viewer_1->spinOnce(100);
      // }

    //--------start mesh generation-----------------------------
      ROS_INFO("start mesh_generation");
      

      //without color
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_total_preprossed(new pcl::PointCloud<pcl::PointXYZ>());
      PreProcessingOfPointCloud(cloud_total_pointer, cloud_total_preprossed);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_total_preprossed_poisson(new pcl::PointCloud<pcl::PointXYZ>());
      PreProcessingOfPointCloud_poission(cloud_total_pointer, cloud_total_preprossed_poisson);

      //with color
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_total_preprossed_c(new pcl::PointCloud<pcl::PointXYZRGB>());
      PreProcessingOfPointCloud_c(pcl_height, cloud_total_preprossed_c);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_total_preprossed_poisson_c(new pcl::PointCloud<pcl::PointXYZRGB>());
      PreProcessingOfPointCloud_poission_c(pcl_height, cloud_total_preprossed_poisson_c);

      //------------------Normal Calculation---------------------------------
      //without color
      pcl::PointCloud<pcl::Normal>::Ptr normals;
      normals = CalculateNormal(cloud_total_preprossed);
      pcl::PointCloud<pcl::Normal>::Ptr normals_poisson;
      normals_poisson = CalculateNormal(cloud_total_preprossed_poisson);

      //with color
      pcl::PointCloud<pcl::Normal>::Ptr normals_c;
      normals_c = CalculateNormal_c(cloud_total_preprossed_c);
      pcl::PointCloud<pcl::Normal>::Ptr normals_poisson_c;
      normals_poisson_c = CalculateNormal_c(cloud_total_preprossed_poisson_c);

      //------------------triangle mesh generation-------------------
      //without color
      pcl::PolygonMesh triangles;
      triangles = greedy_traingle_GenerateMesh(cloud_total_preprossed, normals);

      //with color
      pcl::PolygonMesh triangles_c;
      triangles_c = greedy_traingle_GenerateMesh_c(cloud_total_preprossed_c, normals_c);
      ROS_INFO("finish triangle mesh_generation");
      //------------------Poisson reconstruction----------------------
      //without color
      pcl::PolygonMesh mesh;
      mesh = poisson_reconstruction(cloud_total_preprossed_poisson, normals_poisson);

      //with color
      pcl::PolygonMesh mesh_c;
      mesh_c = poisson_reconstruction_c(cloud_total_preprossed_poisson_c, normals_poisson_c);
      ROS_INFO("finish mesh_generation");

      // visualize generated mesh
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Mesh Viewer"));
        
      viewer->initCameraParameters ();
      //without color
      int v1(0);
      viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
      viewer->setBackgroundColor (0, 0, 0, v1);
      viewer->addText ("Greedy reconstruction with color", 50, 50, "v1 text", v1);
      viewer->addPolygonMesh(triangles_c,"triangle mesh with color",v1);
      int v2(0);
      viewer->createViewPort (0.5, 0.0, 1.0, 0.5, v2);
      viewer->addText ("Poisson reconstruction with color", 50, 50, "v2 text", v2);
      viewer->addPolygonMesh(mesh_c,"Poisson reconstruction with color",v2);
      viewer->setBackgroundColor (0, 0, 0,v2);

      //with color
      int v3(0);
      viewer->createViewPort (0.0, 0.5, 0.5, 1.0, v3);
      viewer->setBackgroundColor (0, 0, 0, v3);
      viewer->addText ("Greedy reconstruction", 50, 50, "v3 text", v3);
      viewer->addPolygonMesh(triangles,"triangle mesh",v3);
      int v4(0);
      viewer->createViewPort (0.5, 0.5, 1.0, 1.0, v4);
      viewer->addText ("Poisson reconstruction", 50, 50, "v4 text", v4);
      viewer->addPolygonMesh(mesh,"Poisson reconstruction",v4);
      viewer->setBackgroundColor (0, 0, 0,v4);

      viewer->addCoordinateSystem (1.0);

      viewer->setCameraPosition(50, 50, 50, -1, -1, -1, 0, -1, 0, v1);
      viewer->setCameraPosition(50, 50, 50, -1, -1, -1, 0, -1, 0, v2);
      viewer->setCameraPosition(50, 50, 50, -1, -1, -1, 0, -1, 0, v3);
      viewer->setCameraPosition(50, 50, 50, -1, -1, -1, 0, -1, 0, v4);


      while (!viewer->wasStopped ())
      {
          viewer->spinOnce ();
          boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }    
     }
  }

//------------below are some predefined functions for mesh generation----------------------

  void transformPointCloud(const sensor_msgs::PointCloud2::ConstPtr &in, const std::string out_id, sensor_msgs::PointCloud2::Ptr &out)
  {
    if (out_id != in->header.frame_id)
    {
      tf::StampedTransform transform;
      try
      {
        listener.lookupTransform(out_id, (*in).header.frame_id, ros::Time(0), transform);
      }
      catch (tf::LookupException &e)
      {
        std::cout<<"cuole"<<std::endl;
        ROS_ERROR ("%s", e.what ());
      }
      Eigen::Matrix4f eigen_transform;
      pcl_ros::transformAsMatrix(transform, eigen_transform);
      pcl_ros::transformPointCloud(eigen_transform, *in, *out);
      (*out).header.frame_id = out_id;
    }
    else
    {
      out = boost::make_shared<sensor_msgs::PointCloud2> (*in);
    }
  }

//preprocessing without color
  //preprocessing
  void PreProcessingOfPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) 
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>());
    DownSample(cloud_in, cloud_temp);
    OutlierFilter(cloud_temp, cloud_out);
  }
  void PreProcessingOfPointCloud_poission(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) 
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>());
    DownSample(cloud_in, cloud_temp);
    OutlierFilter_poisson(cloud_temp, cloud_out);
  }

  //down sample
  void DownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
  {
    pcl::VoxelGrid<pcl::PointXYZ> downSampled;  
    downSampled.setInputCloud(cloud_in);           
    downSampled.setLeafSize(0.2f, 0.2f, 0.2f);  //1 = 1m, 0.1 = 10cm
    downSampled.filter(*cloud_out);   
  }

  //Filtering
  void OutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_out)
  { 
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;  
    pcFilter.setInputCloud(cloud_in);             
    pcFilter.setRadiusSearch(1);           
    pcFilter.setMinNeighborsInRadius(10);      
    pcFilter.filter(*cloud_out);        
  } 
  void OutlierFilter_poisson(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_out)
  { 
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;  
    pcFilter.setInputCloud(cloud_in);             
    pcFilter.setRadiusSearch(1);           
    pcFilter.setMinNeighborsInRadius(50);      
    pcFilter.filter(*cloud_out);        
  } 

  //smoothing
  void SmoothPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_out)
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZ>); 
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setSearchMethod(treeSampling);
    mls.setComputeNormals(false); 
    mls.setInputCloud(cloud_in); 
    mls.setPolynomialOrder(3);  
    mls.setPolynomialFit(false);
    mls.setSearchRadius(0.1); 
    mls.process(*cloud_out);
  }

  //calculate normals
  pcl::PointCloud<pcl::Normal>::Ptr CalculateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in)
  {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation; 
    normalEstimation.setInputCloud(cloud_in);   
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>); 
    normalEstimation.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); 
    normalEstimation.setKSearch(10);           
    normalEstimation.compute(*normals);     
    return normals;
  }


  // greedy traingle mesh generation
  pcl::PolygonMesh greedy_traingle_GenerateMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)
  {	
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZ>());

    SmoothPointcloud(cloud_in, cloud_out1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    OutlierFilter(cloud_out1, cloud_out);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_sum, output);
    pc_publisher.publish(output);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::concatenateFields(*cloud_in, *normals, *cloud_with_normals);


    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZINormal>);
    tree2->setInputCloud(cloud_with_normals);


    pcl::GreedyProjectionTriangulation<pcl::PointXYZINormal> gp3; 
    pcl::PolygonMesh triangles;

    gp3.setSearchRadius(0.5);  ///0.1
    gp3.setMu(2.5);  
    gp3.setMaximumNearestNeighbors(100);  
    gp3.setMinimumAngle(M_PI / 28); //18
    gp3.setMaximumAngle(2 * M_PI / 3); 

    gp3.setMaximumSurfaceAngle(M_PI / 3); //4
    gp3.setNormalConsistency(false); 

    gp3.setInputCloud(cloud_with_normals);  
    gp3.setSearchMethod(tree2);  
    gp3.reconstruct(triangles);
    cloud_with_normals->width = cloud_with_normals->height = 0;

    return triangles;
  }

  // poisson mesh generation
   pcl::PolygonMesh poisson_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZ>());

    SmoothPointcloud(cloud, cloud_out1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    OutlierFilter(cloud_out1, cloud_out);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_sum, output);
    pc_publisher.publish(output);

    //reverse normal direction
    for(size_t i = 0; i < normals->size(); ++i)
    {
      normals->points[i].normal_x *= -1;
      normals->points[i].normal_y *= -1;
      normals->points[i].normal_z *= -1;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    ROS_INFO("start Poisson reconstruction");
    
    pcl::Poisson<pcl::PointNormal> pn;
    pn.setConfidence(false); 
    pn.setDegree(2); 
    pn.setDepth(8); 
    

    pn.setIsoDivide(8); 
    pn.setManifold(false); 

    pn.setOutputPolygons(false); 
    pn.setSamplesPerNode(3.0); 
    pn.setScale(1); 
    pn.setSolverDivide(8); 
    //pn.setIndices();

    pn.setSearchMethod(tree2);
    pn.setInputCloud(cloud_with_normals);

    pcl::PolygonMesh mesh;

    pn.performReconstruction(mesh);

    ROS_INFO("finish Poisson reconstruction");
    // io::savePLYFile("result.ply", mesh);

    return mesh;
  }


  //height visualizer pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloudVisualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); 
    pcl::copyPointCloud(*cloud_xyz, *cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Elevation_rendering(new pcl::PointCloud<pcl::PointXYZRGB>);

   //step 1: Median calculation
    pcl::PointXYZI minPt, maxPt;                       // store max and min point
    pcl::getMinMax3D(*cloud, minPt, maxPt);     // getMinMax3D ( )need cloud in XYZI type
    double Z_Max = maxPt.z;
    double Z_Min = minPt.z;
    double Z_Median1 = Z_Min + (Z_Max - Z_Min) / 3;
    double Z_Median2 = Z_Median1 + (Z_Max - Z_Min) / 3;
    //color value for each point
    for (int index = 0; index < cloud->points.size(); index++)
    {
         if (cloud->points[index].z >= Z_Min && cloud->points[index].z < Z_Median1)
        {
            pcl::PointXYZRGB point;
            point.x = cloud->points[index].x;
            point.y = cloud->points[index].y;
            point.z = cloud->points[index].z;
            point.r = 128 - int(((Z_Median1 - cloud->points[index].z) / (Z_Median1 - Z_Min)) * 128);
            point.g = 255 - int(((Z_Median1 - cloud->points[index].z) / (Z_Median1 - Z_Min)) * 255);
            point.b = 0 + int(((Z_Median1 - cloud->points[index].z) / (Z_Median1 - Z_Min)) * 255);
            cloud_Elevation_rendering->push_back(point);
        }
        if (cloud->points[index].z >= Z_Median1&& cloud->points[index].z < Z_Median2)
        {
            pcl::PointXYZRGB point;
            point.x = cloud->points[index].x;
            point.y = cloud->points[index].y;
            point.z = cloud->points[index].z;
            point.r = 255 - int(((Z_Median2 - cloud->points[index].z) / (Z_Median2 - Z_Median1)) * 128);
            point.g = 255;
            point.b = 0;
            cloud_Elevation_rendering->push_back(point);
        }
        if (cloud->points[index].z >= Z_Median2 && cloud->points[index].z < Z_Max)
        {
            pcl::PointXYZRGB point;
            point.x = cloud->points[index].x;
            point.y = cloud->points[index].y;
            point.z = cloud->points[index].z;
            point.r = 255;
            point.g = 255 - int(((cloud->points[index].z - Z_Median2) / (Z_Max - Z_Median2)) * 255);
            point.b = 0;
            cloud_Elevation_rendering->push_back(point);
        }
    }
    return cloud_Elevation_rendering;

  }
//preprocessing with color
  //point cloud preprocessing
  void PreProcessingOfPointCloud_c(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out) 
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
    DownSample_c(cloud_in, cloud_temp);
    OutlierFilter_c(cloud_temp, cloud_out);
  }

  void PreProcessingOfPointCloud_poission_c(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out) 
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
    DownSample_c(cloud_in, cloud_temp);
    OutlierFilter_poisson_c(cloud_temp, cloud_out);
  }

  //down sample
  void DownSample_c(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> downSampled;  
    downSampled.setInputCloud(cloud_in);           
    downSampled.setLeafSize(0.2f, 0.2f, 0.2f);  //1 = 1m, 0.1 = 10cm
    downSampled.filter(*cloud_out);   
  }

  //Filtering
  void OutlierFilter_c(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out)
  { 
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> pcFilter;  
    pcFilter.setInputCloud(cloud_in);             
    pcFilter.setRadiusSearch(1);           
    pcFilter.setMinNeighborsInRadius(5);      
    pcFilter.filter(*cloud_out);        
  } 

   void OutlierFilter_poisson_c(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out)
  { 
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> pcFilter;  
    pcFilter.setInputCloud(cloud_in);             
    pcFilter.setRadiusSearch(1);           
    pcFilter.setMinNeighborsInRadius(50);      
    pcFilter.filter(*cloud_out);        
  } 

  //smoothing
  void SmoothPointcloud_c(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out)
  {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZRGB>); 
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
    mls.setSearchMethod(treeSampling);
    mls.setComputeNormals(false); 
    mls.setInputCloud(cloud_in); 
    mls.setPolynomialOrder(6);  
    mls.setPolynomialFit(false);
    mls.setSearchRadius(1); 
    mls.process(*cloud_out);
  }

  //calculate normals
  pcl::PointCloud<pcl::Normal>::Ptr CalculateNormal_c(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in)
  {
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation; 
    normalEstimation.setInputCloud(cloud_in);   
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>); 
    normalEstimation.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); 
    normalEstimation.setKSearch(10);           
    normalEstimation.compute(*normals);     
    return normals;
  }


  // greedy traingle mesh generation
  pcl::PolygonMesh greedy_traingle_GenerateMesh_c(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)
  {	
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZRGB>());

    SmoothPointcloud_c(cloud_in, cloud_out1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());
    OutlierFilter_c(cloud_out1, cloud_out);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_sum, output);
    pc_publisher.publish(output);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloud_in, *normals, *cloud_with_normals);

    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud(cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3; 
    pcl::PolygonMesh triangles;

    gp3.setSearchRadius(0.5);  ///0.1
    gp3.setMu(2.5);  
    gp3.setMaximumNearestNeighbors(100);  
    gp3.setMinimumAngle(M_PI / 28); //18
    gp3.setMaximumAngle(2 * M_PI / 3); 

    gp3.setMaximumSurfaceAngle(M_PI / 3); //4
    gp3.setNormalConsistency(false); 

    gp3.setInputCloud(cloud_with_normals);  
    gp3.setSearchMethod(tree2);  
    gp3.reconstruct(triangles);
    cloud_with_normals->width = cloud_with_normals->height = 0;

    //for mesh color
    pcl::PointCloud<pcl::PointXYZRGB> cloud_color_mesh; 
    pcl::fromPCLPointCloud2(triangles.cloud, cloud_color_mesh); 

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud_in);
    // K nearest neighbor search
    int K = 5;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    for(int i=0;i<cloud_color_mesh.points.size();++i)
    {
      uint8_t r = 0;
      uint8_t g = 0;
      uint8_t b = 0;
      float dist = 0.0; 
      int red = 0;
      int green = 0;
      int blue = 0;
      uint32_t rgb;

      if ( kdtree.nearestKSearch (cloud_color_mesh.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {
          for (int j = 0; j < pointIdxNKNSearch.size (); ++j) 
          { 

            r = cloud_in->points[ pointIdxNKNSearch[j] ].r;
            g = cloud_in->points[ pointIdxNKNSearch[j] ].g;
            b = cloud_in->points[ pointIdxNKNSearch[j] ].b;

            red += int(r);
            green += int(g);
            blue += int(b);
            dist += 1.0/pointNKNSquaredDistance[j]; 

            std::cout<<"red: "<<int(r)<<std::endl; 
            std::cout<<"green: "<<int(g)<<std::endl; 
            std::cout<<"blue: "<<int(b)<<std::endl; 
            cout<<"dis:"<<dist<<endl;
          }  
      }

      cloud_color_mesh.points[i].r = int(red/pointIdxNKNSearch.size ()+0.5); 
      cloud_color_mesh.points[i].g = int(green/pointIdxNKNSearch.size ()+0.5); 
      cloud_color_mesh.points[i].b = int(blue/pointIdxNKNSearch.size ()+0.5);
    }
    toPCLPointCloud2(cloud_color_mesh, triangles.cloud);

    return triangles;
  }

  // poisson mesh generation
   pcl::PolygonMesh poisson_reconstruction_c(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
  {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZRGB>());

    SmoothPointcloud_c(cloud, cloud_out1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());
    OutlierFilter_c(cloud_out1, cloud_out);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_sum, output);
    pc_publisher.publish(output);

    //reverse normal direction
    for(size_t i = 0; i < normals->size(); ++i)
    {
      normals->points[i].normal_x *= -1;
      normals->points[i].normal_y *= -1;
      normals->points[i].normal_z *= -1;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud(cloud_with_normals);

    ROS_INFO("start Poisson reconstruction");
    
    pcl::Poisson<pcl::PointXYZRGBNormal> pn;
    pn.setConfidence(false); 
    pn.setDegree(2); 
    pn.setDepth(8); 
    

    pn.setIsoDivide(8); 
    pn.setManifold(false); 

    pn.setOutputPolygons(false); 
    pn.setSamplesPerNode(3.0); 
    pn.setScale(1); 
    pn.setSolverDivide(8); 
    //pn.setIndices();

    pn.setSearchMethod(tree2);
    pn.setInputCloud(cloud_with_normals);

    pcl::PolygonMesh mesh;

    pn.performReconstruction(mesh);

    ROS_INFO("finish Poisson reconstruction");
    // mesh coloring for visualization

    pcl::PointCloud<pcl::PointXYZRGB> cloud_color_mesh; 
    pcl::fromPCLPointCloud2(mesh.cloud, cloud_color_mesh); 

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud);
    // K nearest neighbor search
    int K = 5;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    for(int i=0;i<cloud_color_mesh.points.size();++i)
    {
      uint8_t r = 0;
      uint8_t g = 0;
      uint8_t b = 0;
      float dist = 0.0; 
      int red = 0;
      int green = 0;
      int blue = 0;
      uint32_t rgb;

      if ( kdtree.nearestKSearch (cloud_color_mesh.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {
          for (int j = 0; j < pointIdxNKNSearch.size (); ++j) 
          { 

            r = cloud->points[ pointIdxNKNSearch[j] ].r;
            g = cloud->points[ pointIdxNKNSearch[j] ].g;
            b = cloud->points[ pointIdxNKNSearch[j] ].b;

            red += int(r);
            green += int(g);
            blue += int(b);
            dist += 1.0/pointNKNSquaredDistance[j]; 

            std::cout<<"red: "<<int(r)<<std::endl; 
            std::cout<<"green: "<<int(g)<<std::endl; 
            std::cout<<"blue: "<<int(b)<<std::endl; 
            cout<<"dis:"<<dist<<endl;
          }  
      }

      cloud_color_mesh.points[i].r = int(red/pointIdxNKNSearch.size ()+0.5); 
      cloud_color_mesh.points[i].g = int(green/pointIdxNKNSearch.size ()+0.5); 
      cloud_color_mesh.points[i].b = int(blue/pointIdxNKNSearch.size ()+0.5);

    }
    toPCLPointCloud2(cloud_color_mesh, mesh.cloud);
    return mesh;
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_node");
  mesh_generation ld;
  ros::spin();
  return 0;
}
/*reference:
  1. http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
  2. https://johnwlambert.github.io/stereo/
  3. http://wiki.ros.org/stereoimageproc
  4. http://wiki.ros.org/rviz (Accessed on Mar. 21, 2023)
  5. https://diglib.eg.org/handle/10.2312/SGP.SGP06.061-070
  6. https://pointclouds.org/documentation/tutorials/index.html
  7. http://t.csdn.cn/YpuzY
  8. http://t.csdn.cn/Dyq1a
  9. https://blog.csdn.net/Architet_Yang/article/details/90049715
*/
