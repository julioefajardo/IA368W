#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <boost/thread/mutex.hpp>

#include <math.h>
#include <Eigen/Dense>

#include <iostream>

using namespace std;
using namespace Eigen;

#define IDEAL

boost::mutex pose_mutex;
boost::mutex scan_mutex;

// 8 X 8 meters map, cell -> 80 mm
const double  CELL_SIZE	= 0.08;
const int SIZE = 100;


const double angle_increment = 0.0174533;	
const unsigned int n_angles = 181; 
const double precission = 50;	//mm
float phi[181];
sensor_msgs::LaserScan *current_scan = new sensor_msgs::LaserScan;

// Map
MatrixXf mmapa = MatrixXf::Zero(SIZE,SIZE);   
std::string world_frame_id = ros::this_node::getName() + "/map";
nav_msgs::OccupancyGrid mapa;
uint8_t flag = 0;

// Robot Pose
Vector3f pose(0.0,0.0,0.0);	//(x,y,theta)
uint8_t mute = 1;

// Covariance Matrix
MatrixXf sigma(2,2);

// Measurement
Vector2f mu;
Vector2f cellp;
Vector2f delta;

double P;
const double Pocc   = 0.8;
const double Pfree  = 0.2;
const double Pprior = 0.45; //trying with 0.4, 0.45 instead 0.5
double K = 0.08;
double invSen;
double lo;
int16_t Prob;

// Transformations
double roll, pitch, yaw;

void processPose(const nav_msgs::Odometry::ConstPtr& p){
  tf::Quaternion q(
       p->pose.pose.orientation.x,
       p->pose.pose.orientation.y,
       p->pose.pose.orientation.z,
       p->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  pose(0) = p->pose.pose.position.x;
  pose(1) = p->pose.pose.position.y;
  pose(2) = yaw;
}
          
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){  
  *current_scan = *scan;   
}

int main(int argc, char** argv){

  ros::init(argc, argv, "my_mapping_node");
  ros::NodeHandle nh_;
  ros::Subscriber scanSub = nh_.subscribe<sensor_msgs::LaserScan>("/RosAria/sim_lms2xx_1_laserscan",10,&processLaserScan);
  ros::Subscriber poseSub = nh_.subscribe<nav_msgs::Odometry>("/RosAria/pose",10,&processPose);
  ros::Publisher map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/OccupancyGrid", 1, true);

  mapa.info.resolution = CELL_SIZE;         
  mapa.info.width      = SIZE;          
  mapa.info.height     = SIZE;          
  mapa.info.origin.position.x = -(SIZE) / 2.0 * CELL_SIZE;
  mapa.info.origin.position.y = -(SIZE) / 2.0 * CELL_SIZE;
  mapa.info.origin.orientation.w = 1.0;  
  mapa.data.assign(SIZE*SIZE, -1);

  for(int i=0; i<n_angles; i++) phi[i] = -M_PI/2.0 + angle_increment*i;
  sigma << 0.005, 0.0,
         0.0 , 0.0005;
  lo = log(Pprior/(1.0-Pprior));
  
  ros::Rate rate(100);

  cout << "Generating map ..." << endl;

  while(ros::ok()){
    for(int k=0; k<current_scan->ranges.size(); k++){
      boost::unique_lock<boost::mutex> scoped_lock(scan_mutex);
      mu(0) = current_scan->ranges[k];
      mu(1) = phi[k];
      for(int j=0; j<SIZE; j++){
        for(int i=0; i<SIZE; i++){
          double xi = (i*CELL_SIZE + CELL_SIZE/2.0) - ((SIZE/2.0)*CELL_SIZE);
          double yj = (j*CELL_SIZE + CELL_SIZE/2.0) - ((SIZE/2.0)*CELL_SIZE); 
          boost::unique_lock<boost::mutex> scoped_lock(pose_mutex);
          cellp(0) = sqrt((xi-pose(0))*(xi-pose(0)) + (yj-pose(1))*(yj-pose(1)));	// radius (r)
          cellp(1) = atan2((yj-pose(1)), (xi-pose(0))) - pose(2);			// angle  (b)	
          delta = mu -cellp;
          if (abs(delta(1)) < 2*angle_increment){
            #ifdef IDEAL 
              if (cellp(0) < (mu(0)-CELL_SIZE)) P = Pfree;
              else {
                if (cellp(0) > (mu(0)+CELL_SIZE)) P = Pprior; 
                else P = Pocc;
              }	
              mmapa(i,j) = mmapa(i,j) + log(P/(1.0-P)) - lo;
            #else
              if(cellp(0)<(mu(0)-CELL_SIZE)) P = Pfree;
              else {
                if (cellp(0) > (mu(0)+CELL_SIZE)) P = Pprior; 
                else P = Pocc;
              }  
              invSen = P + (K/(2*M_PI*sigma(0,0)*sigma(1,1)) + 0.5 - P)*exp(-0.5*delta.transpose()*sigma.inverse()*delta); 
              mmapa(i,j) = mmapa(i,j) + log(invSen/(1.0-invSen)) - lo;
            #endif
	  }
        }
      }
    }    
    for(int j=0; j<SIZE; j++){
      for(int i=0; i<SIZE; i++){
        //if(mmapa(i,j)>abs(0.01)) Prob = -1;
        //else Prob = (100 * (1 - (1 / (1 + exp( mmapa(i,j) ) ) ) ) );
        Prob = (100 * (1 - (1 / (1 + exp( mmapa(i,j) ) ) ) ) ); 
        if((Prob>=48)&&(Prob<=52)) mapa.data[i+j*SIZE] = -1; 
        else mapa.data[i+j*SIZE] = (int8_t)Prob;  
      } 
    }
    map_pub.publish(mapa);
    rate.sleep(); 
    ros::spinOnce();
  }
}
