
/*
This node is to generate colored mesh. 
Process: 1. subscribe point cloud generated from stereo_image_proc
         2. merge point cloud in world frame
         3. preprocessing point cloud(downsampling, outlier filter)
         4. generate triangle and poisson mesh from point cloud
         5. add RGB informationto mesh
         6. visulization on pcl viewer
          */
#include <opencv2/opencv.hpp>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/thread.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h> 
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/surface/gp3.h> 
#include <pcl/visualization/pcl_visualizer.h>

#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <cstdlib>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stereo_msgs/DisparityImage.h>
#include <ros/package.h>

using namespace std;
using namespace pcl;
const double camera_factor = 1000;

static const std::string OPENCV_WINDOW1 = "Display1";
static const std::string OPENCV_WINDOW2 = "Display2";
static const std::string tfrgbL = "Quadrotor/RGBCameraLeft";

class colored_mesh_generation
{
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub;
    ros::Publisher pc_publisher;
    ros::Subscriber current_state;

    tf::TransformListener listener;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_sum;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_total;
    std::string taget_frame;
    double goalx, goaly, goalz;
    double height, X, Y, vx, vy, vz, wx, wy, wz;
    int i, m;
    bool flag1, flag2;
    std::string folder_path;
    std::string file_path;

  public:
    colored_mesh_generation()
    {
      pc_publisher = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud_sum", 1);
      point_cloud_sub = nh_.subscribe("/stereo/points2", 1, &colored_mesh_generation::callback, this);
      current_state = nh_.subscribe("current_state_est", 1, &colored_mesh_generation::onCurrentState, this);
      folder_path = ros::package::getPath("mesh_generation_pkg");
      taget_frame = "world";
      i = 0;
      m = 0;
      flag1 = false;
      flag2 = false;
      goalx = 16;
      goaly = 4*2;
      goalz = 3.5;
    }

    ~colored_mesh_generation(){
      cv::destroyWindow(OPENCV_WINDOW1);
      cv::destroyWindow(OPENCV_WINDOW2);
    } 

    //---------current state of drone-------------
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
      if((abs(vx)>0.5)||(abs(vy)>0.5)||(abs(vz)>0.5))
      {
        flag1 = false;
        flag2 = false;
      }
      if((abs(vx)<0.1)&&(abs(vy)<0.1)&&(abs(vz)<0.1)&&(flag1==false))
      {
        flag1 = true;
        ros::Duration(3).sleep();
      }
    }
    

    void callback(const sensor_msgs::PointCloud2& cloud_msg)
    {
      if((abs(vx)<0.1)&&(abs(vy)<0.1)&&(abs(vz)<0.1))
      {
        if(flag2==false)
        {
          ROS_INFO("I arrived one of waypoints");
          //transform point cloud to world frame
          ros::Time time_st = ros::Time::now(); 
          tf::StampedTransform transform;
          sensor_msgs::PointCloud2 out;
          listener.lookupTransform("camera", "world", ros::Time(0), transform);
          pcl_ros::transformPointCloud(taget_frame, cloud_msg, out, listener);
          pcl::PointCloud<pcl::PointXYZRGB>pcl_cloud;
          pcl::fromROSMsg(out, pcl_cloud);

          //merge point cloud together and publish to rostopic
          cloud_sum.header.stamp = time_st.toNSec()/1e3;
          cloud_sum.header.frame_id = "world";
          cloud_sum = cloud_sum + pcl_cloud;
          std::vector<int> indices;
          pcl::removeNaNFromPointCloud(cloud_sum, cloud_sum, indices);
          sensor_msgs::PointCloud2 output;
          pcl::toROSMsg(cloud_sum, output);
          pc_publisher.publish(output);
          flag2 = true; 

          //save point cloud to .pcd files
          if(i == 9) 
          {
            file_path = folder_path + "/p_" + std::to_string(m) +".pcd";
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sum_preprossed(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sum_pointer(new pcl::PointCloud<pcl::PointXYZRGB>());
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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pointer(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (int k = 0; k < m; k++) // K:number of waypoints;
        {
          file_path = folder_path + "/p_" + std::to_string(k) + ".pcd";
          if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path, *cloud_pointer)==-1)
          {
            PCL_ERROR("Couldn't read .pcd file \n");
          }
          else{
            ROS_INFO("successfully load pcd file");
          }
          cloud_total = cloud_total + *cloud_pointer;
        }

        //-------visualize the total point clouds-------------------
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_total_pointer(new pcl::PointCloud<pcl::PointXYZRGB>());
        *cloud_total_pointer = cloud_total;

        //------------------preprocessing for mesh generation------------------------------
        ROS_INFO("start mesh_generation");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_total_preprossed(new pcl::PointCloud<pcl::PointXYZRGB>());
        PreProcessingOfPointCloud(cloud_total_pointer, cloud_total_preprossed);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_total_preprossed_poisson(new pcl::PointCloud<pcl::PointXYZRGB>());
        PreProcessingOfPointCloud_poission(cloud_total_pointer, cloud_total_preprossed_poisson);
        //------------------Pointcloud visualization---------------------------
        // cloud_viewer(cloud_sum_preprossed);

        //------------------Normal Calculation---------------------------------
        pcl::PointCloud<pcl::Normal>::Ptr normals;
        normals = CalculateNormal(cloud_total_preprossed);
        pcl::PointCloud<pcl::Normal>::Ptr normals_poisson;
        normals_poisson = CalculateNormal(cloud_total_preprossed_poisson);

        //------------------triangle mesh generation-------------------
        pcl::PolygonMesh triangles;
        triangles = greedy_traingle_GenerateMesh(cloud_total_preprossed, normals);
        ROS_INFO("finish triangle mesh_generation");

        //------------------Poisson reconstruction----------------------
        pcl::PolygonMesh mesh;
        mesh = poisson_reconstruction(cloud_total_preprossed_poisson, normals_poisson);
        ROS_INFO("finish mesh_generation");


        //------------------Mesh Visualization-----------------------------------
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Mesh Viewer"));
        
        viewer->initCameraParameters ();
        int v1(0);
        viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor (0, 0, 0, v1);
        viewer->addText ("Greedy reconstruction ", 50, 50, "v1 text", v1);
        viewer->addPolygonMesh(triangles,"triangle mesh",v1);
        int v2(0);
        viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
        viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
        viewer->addText ("Poisson reconstruction", 50, 50, "v2 text", v2);

        viewer->addPolygonMesh(mesh,"Poisson reconstruction",v2);
        viewer->setBackgroundColor (0, 0, 0,v2);
        viewer->addCoordinateSystem (1.0);
        viewer->setCameraPosition(50, 50, 50, -1, -1, -1, 0, -1, 0, v1);
        viewer->setCameraPosition(50, 50, 50, -1, -1, -1, 0, -1, 0, v2);


        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
      }
    }

//---------------------------------Cloud Visualization-----------------------------------------
    pcl::visualization::PCLVisualizer::Ptr cloud_viewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
      pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("colored Cloud"));
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
      viewer->addPointCloud(cloud, rgb, "sample cloud");
      viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); //backgroundcolor grey
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

      viewer->addCoordinateSystem(0.5); //0.5 *kosy
      while (!viewer->wasStopped())
      {
        viewer->spinOnce();
      }
      return (viewer);
    }

//-----------------------------------Poission reconstruction-------------------------------------------
    PolygonMesh poisson_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
    {

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZRGB>());

      SmoothPointcloud(cloud, cloud_out1);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());
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
      // io::savePLYFile("result.ply", mesh);

  //-------------------Mesh color----------------------------------------
      PointCloud<PointXYZRGB> cloud_color_mesh; 
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

//------------------------Preprocessing of PointCloud------------------------------------------

    //preprocessing
    void PreProcessingOfPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out) 
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
      DownSample(cloud_in, cloud_temp);
      OutlierFilter(cloud_temp, cloud_out);
    }

    void PreProcessingOfPointCloud_poission(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out) 
    {

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
      DownSample(cloud_in, cloud_temp);
      OutlierFilter_poisson(cloud_temp, cloud_out);
    }

    //down sample
    void DownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
    {
      pcl::VoxelGrid<pcl::PointXYZRGB> downSampled;  
      downSampled.setInputCloud(cloud_in);           
      downSampled.setLeafSize(0.2f, 0.2f, 0.2f);  //1 = 1m, 0.1 = 10cm
      downSampled.filter(*cloud_out);   
    }

    //Filtering
    void OutlierFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out)
    { 
      pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> pcFilter;  
      pcFilter.setInputCloud(cloud_in);             
      pcFilter.setRadiusSearch(1);           
      pcFilter.setMinNeighborsInRadius(3);      
      pcFilter.filter(*cloud_out);        
    } 
    void OutlierFilter_poisson(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out)
    { 
      pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> pcFilter;  
      pcFilter.setInputCloud(cloud_in);             
      pcFilter.setRadiusSearch(1);           
      pcFilter.setMinNeighborsInRadius(50);      
      pcFilter.filter(*cloud_out);        
    } 

    //smoothing
    void SmoothPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out)
    {
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZRGB>); 
      pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
      mls.setSearchMethod(treeSampling);
      mls.setComputeNormals(false); 
      mls.setInputCloud(cloud_in); 
      mls.setPolynomialOrder(3);  
      mls.setPolynomialFit(false);
      mls.setSearchRadius(0.1); 
      mls.process(*cloud_out);
    }

//--------------------------------------Normal calculation----------------------------------------------------------
    pcl::PointCloud<pcl::Normal>::Ptr CalculateNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in)
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

//----------------------------------------triangle mesh generation---------------------------------------------
    pcl::PolygonMesh greedy_traingle_GenerateMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)
    {	
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZRGB>());

      SmoothPointcloud(cloud_in, cloud_out1);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());
      OutlierFilter(cloud_out1, cloud_out);

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

      PointCloud<PointXYZRGB> cloud_color_mesh; 
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

      
        toPCLPointCloud2(cloud_color_mesh, triangles.cloud);

        return triangles;
      }
    }
    
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "colored_node");
  colored_mesh_generation ld;
  ros::spin();
  return 0;
}

/*
reference:
http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html
https://pointclouds.org/documentation/tutorials/greedy_projection.html
https://pointclouds.org/documentation/structpcl_1_1_polygon_mesh.html
https://pointclouds.org/documentation/classpcl_1_1visualization_1_1_p_c_l_visualizer.html
https://pointclouds.org/documentation/tutorials/kdtree_search.html
http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
https://johnwlambert.github.io/stereo/
http://wiki.ros.org/stereoimageproc
http://wiki.ros.org/rviz (Accessed on Mar. 21, 2023)
https://diglib.eg.org/handle/10.2312/SGP.SGP06.061-070
https://pointclouds.org/documentation/tutorials/index.html
http://t.csdn.cn/YpuzY
http://t.csdn.cn/Dyq1a
https://blog.csdn.net/Architet_Yang/article/details/90049715

*/