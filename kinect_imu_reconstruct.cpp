#include <iostream>
#include "converter.h"
#include <string.h>
#include<utility>
#include<vector>


#include<boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/pcl_plotter.h>
#include<pcl/common/common_headers.h>
#include<pcl/features/normal_3d.h>
#include <pcl/console/time.h>   // TicToc





// COMPONENTS io common visualization registration

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5 (new pcl::PointCloud<pcl::PointXYZ>);


//boost::shared_ptr<pcl::visualization::PCLPlotter> * plotter (new pcl::visualization::PCLPlotter("3DViewer"));
pcl::visualization::PCLPlotter * plotter = new pcl::visualization::PCLPlotter ();
//pcl::visualization::PCLPlotter plotter ("PCL Plotter");
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3DViewer"));
//pcl::visualization::CloudViewer viewer ("PCL visualizer");
pcl::console::TicToc time;
int iterations = 1;
if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gaoxinyang/kinect_project/Experiment6/output/mesh_1_rotated.pcd", *cloud1) == -1) //* load the file

//if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/gaoxinyang/kinect_project/Experiment7/input/mesh_1.pcd", *cloud1) == -1) //* load the file

  {
    PCL_ERROR ("Couldn't read pcd file \n");
    return (-1);
  }
cout<<"Mesh1 read"<<endl;




if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gaoxinyang/kinect_project/Experiment6/output/mesh_2_rotated.pcd", *cloud2) == -1) //* load the file

//if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gaoxinyang/kinect_project/Experiment7/input/mesh_2.pcd", *cloud2) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read pcd file \n");
    return (-1);
  }
cout<<"Mesh2 read"<<endl;


//if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gaoxinyang/kinect_project/Experiment6/input/mesh_3.pcd", *cloud3) == -1) //* load the file

if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gaoxinyang/kinect_project/Experiment6/output/mesh_3_origin.pcd", *cloud3) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read pcd file \n");
    return (-1);
  }
cout<<"Mesh3 read"<<endl;


/*
if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gaoxinyang/kinect_project/input_4.pcd", *cloud4) == -1) //* load the file
 //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gaoxinyang/kinect_project/mesh_4_rotated.pcd", *cloud4) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read pcd file \n");
    return (-1);
  }
cout<<"Mesh4 read"<<endl;



if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gaoxinyang/kinect_project/Experiment5/input/mesh_1.pcd", *cloud5) == -1) //* load the file
 //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gaoxinyang/kinect_project/mesh_5_origin.pcd", *cloud5) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read pcd file \n");
    return (-1);
  }
cout<<"Mesh5 read"<<endl;

*/

string msg="###########Experiment 8-Input ";
cout<<msg<<"begins##########"<<endl;

int numIter=20;
//vector<double> iter_value(numIter,0), iter(numIter,0);
double iter_value[numIter], iter[numIter];

//pcl::console::TicToc time;
  //time.tic ();

//ICP Algorithm
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

//....1...2
cout<<"ICP 1...2..."<<endl;
//for(int i=0;i<numIter;i++)
int i=0;

cout<<"hi"<<endl;


  pcl::PointCloud<pcl::PointXYZ> Final12;

  icp.setMaxCorrespondenceDistance (0.5); 
  icp.setMaximumIterations(50); 
  //icp.setTransformationEpsilon (1e-12); 
  //icp.setEuclideanFitnessEpsilon (1e-12); 
 
    icp.setInputCloud(cloud1);
  icp.setInputTarget(cloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud_12=cloud1;
 //icp.align(*Final12);
 //icp.align(Final12);
//pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud_12(&Final12);


//cout<<"fitness score .....1...2"<<endl;


for (int i=0;i<5;i++)
{

time.tic ();
icp.setMaximumIterations (iterations);
    cloud1=icp_cloud_12;
    icp.setInputCloud(cloud1);
    icp.align(*icp_cloud_12);
   // cout<<" "<<icp.getFitnessScore();
std::cout << "\n Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;
if (icp.hasConverged ())
  {
    std::cout << "ICP has converged, score is " << icp.getFitnessScore () << std::endl;
    //std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }
}

//....12...3
cout<<"ICP 12...3......................................................."<<endl;
  icp.setInputCloud(icp_cloud_12);
  icp.setInputTarget(cloud3);
  //icp.setMaximumIterations (60);
  pcl::PointCloud<pcl::PointXYZ> Final12_3;
  //icp.align(Final12_3);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud_123(&Final12_3);
 pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud_123=icp_cloud_12;
for (int i=0;i<5;i++)
{
time.tic ();
icp.setMaximumIterations (iterations);
    icp_cloud_12=icp_cloud_123;
    icp.setInputCloud(icp_cloud_12);
    icp.align(*icp_cloud_123);
    //cout<<" "<<icp.getFitnessScore();
std::cout << "\n Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;
if (icp.hasConverged ())
  {
    std::cout << "ICP has converged, score is " << icp.getFitnessScore () << std::endl;
    //std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }
}


cout<<msg<<"ends##########"<<endl;

/*
//....123...4
cout<<"ICP 123...4..."<<endl;
  icp.setInputCloud(icp_cloud_123);
  icp.setInputTarget(cloud4);
  pcl::PointCloud<pcl::PointXYZ> Final123_4;
  icp.align(Final123_4);
  pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud_1234(&Final123_4);




//....1234...5
cout<<"ICP 1234...5..."<<endl;
  icp.setInputCloud(icp_cloud_1234);
  icp.setInputTarget(cloud5);
  pcl::PointCloud<pcl::PointXYZ> Final1234_5;
  icp.align(Final1234_5);
  pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud_12345(&Final1234_5);
*/
  //pcl::PLYWriter writer;
  //writer.write("/home/gaoxinyang/kinect_project", Final12, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true, true);
  //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   //viewer.showCloud (Final12);


//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3DViewer"));
//pcl::visualization::CloudViewer viewer ("PCL visualizer");
//viewer.showCloud (icp_cloud_123);

//pcl::io::savePCDFileASCII ("icp_12_inp_6.pcd", Final12);
//pcl::io::savePCDFileASCII ("icp_123_inp_6.pcd", Final12_3);

//std::cout <<  " Time Taken " << time.toc () << " ms\n" << std::endl;
/*
pcl::io::savePCDFileASCII ("output1234_A.pcd", Final123_4);

pcl::io::savePCDFileASCII ("Final_output_A.pcd", Final1234_5);
*/
//int var1;
//viewer.setSize (666, 555);


//viewer->setBackgroundColor(0,0,0); 
//viewer->addPointCloud<pcl::PointXYZ>(Final12,"samplecloud");
  //for (size_t i = 0; i < cloud->points.size (); ++i)
    //std::cout << "    " << cloud->points[i].x
    //          << " "    << cloud->points[i].y
    //          << " "    << cloud->points[i].z << std::endl;

  return (0);
}



//Time with IMU Data : .....  2.0369e+06 ms
//Time without IMU :...... 548460  ms


//Experment5
//time with IMU data : ...... 25633 ms
//time without IMU: ... ... 11348 ms


//Experiment6
//time with IMU data : .......
//time without IMU : ........




