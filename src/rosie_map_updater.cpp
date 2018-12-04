#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <string>
#include <numeric>      // std::accumulate

#include <rosie_map_updater/NewGrid.h>
#include <rosie_map_controller/RequestMapStoring.h>
#include <rosie_map_controller/MapStoring.h>
#include <rosie_map_controller/RequestLoading.h>
#include <rosie_map_controller/WallDefinition.h>
#include <cstdlib>

ros::ServiceClient storeMapClient;
ros::ServiceClient loadClient;

//rosie_map_controller::StartRRT startSrv;
rosie_map_controller::RequestMapStoring mapSrv;
rosie_map_controller::RequestLoading loadSrv;

boost::shared_ptr<sensor_msgs::PointCloud> lastPointCloud_ptr;
boost::shared_ptr<tf::TransformListener> laser_tfl_ptr;
boost::shared_ptr<tf::StampedTransform> laser_point_tf_ptr;

float PI = 3.1415926f;
float robotsize = 0.2f;
float OFFSET[] = {0,0};
float XDIM;
float YDIM;
int forgettingTime = 10; // in sec

ros::Time load_time;
std::vector<ros::Time> last_load_time;

ros::ServiceClient occClient;
rosie_map_updater::NewGrid newGridSrv;
ros::Publisher wallStack_pub;
ros::Publisher wall_viz_pub;

std::vector<float> lidarx;
std::vector<float> lidary;
std::vector<float> wallArray;
std::vector<float> ALL_OBS;

nav_msgs::Odometry pose;

int mapInitialized = 0;
int gridInitialized = 0;

std::vector<std::vector<float> > newWalls;


float certaintyValue = 100;
float certaintyLimit = -0.015;

void certaintyCallback(std_msgs::Float32 msg){
	certaintyValue = msg.data;
}

float origin[] = {0.0f,0.0f};
float temp[2];
float m;
float r = 0.1f;
float x_1,y_1,x_2,y_2;
float diffy;
float diffx;
float P[8];
float p1_new[2];
float p2_new[2];
float n;
float co;
float si;


bool addToObs(float data[]){
	float p[4];

	//float abst = sqrt(pow(OBS[i+2]-OBS[i+0],2.0)+pow(OBS[i+3]-OBS[i+1],2.0));
	//float m1 = (OBS[i+2]-OBS[i+0])/dist;
	//float m2 = (OBS[i+3]-OBS[i+1])/dist;
	p[0] =data[0];
	p[1] =data[1];
	p[2] =data[2];
	p[3] =data[3];
	//ROS_INFO("x1 %f y1 %f x2 %f y2 %f", p1[0], p1[1], p2[0], p2[1] );


	if((p[3]< p[1]) or (p[3] == p[1] and p[2]<p[0])){
		temp[0] = p[0];
		temp[1] = p[1];
		ROS_INFO("%f %f", p[0], p[1]);
		p[0] = p[2];
		p[1] = p[3];
		ROS_INFO("%f %f", p[2], p[3]);
		p[2] = temp[0];
		p[3] = temp[1];

	}
	diffx = p[2]-p[0];
	diffy = p[3]-p[1];
	m = (float) atan2(diffy, diffx);
	ROS_INFO("m %f",m);
	co = r*cos(m);
	si = r*sin(m);

	x_1 = p[0] - co;
	y_1 = p[1] - si;
	x_2 = p[2] + co;
	y_2 = p[3] + si;
	//m = m+PI/(2.0f);
	si = r*cos(m);
	co = r*sin(m);
	 P[0] = x_1-co;
	 P[1] = y_1+si;
	 P[2] = x_1+co;
	 P[3] = y_1-si;
	 P[4] = x_2+co;
	 P[5] = y_2-si;
	 P[6] = x_2-co;
	 P[7] = y_2+si;
	 ROS_INFO("%f %f %f %f %f %f %f %f", P[0], P[1], P[2], P[3], P[4], P[5], P[6], P[7] );
	 for(int i=0;i<8;i++){
		 ALL_OBS.push_back(P[i]);
	 }

	 return true;
}

float minX, minY, maxX, maxY;
float czone;
float sX, sY, eX, eY;
std::vector<visualization_msgs::Marker> markers;
float resolution;
rosie_map_controller::MapStoring wallStack;
rosie_map_controller::MapStoring originalMap;
rosie_map_controller::MapStoring completeMap;
bool mapInitializing;
void wallCallback(const visualization_msgs::MarkerArray msg){
	if(mapInitializing){
		return;
	}
	mapInitializing = 1;
		int numbMarkers = msg.markers.size();
		czone = robotsize/(2.0f) + 0.02f; //additional extra security distance

		wallArray.clear();
		wallArray.resize(4*numbMarkers);
		for(int i = 0; i < 4*numbMarkers; ++i){
			wallArray[i] = -1.0f;
		}
		float single_wall[4];
		rosie_map_controller::WallDefinition tempwall;

		minX = minY = maxX = maxY = 0;
		for(int k = 0; k < numbMarkers; ++k){
			//ROS_ERROR("Updater marker loop: %d", k);
			//Extract point data
			std::string pointsText = msg.markers[k].text;
			std::stringstream ss(pointsText);

			ss>>sX;
			ss>>sY;
			ss>>eX;
			ss>>eY;

			//Set point data on every 4th index
			wallArray[k<<2]=sX;
			wallArray[(k<<2)+1]=sY;
			wallArray[(k<<2)+2]=eX;
			wallArray[(k<<2)+3]=eY;

			if(sX < minX){
				minX = sX;
			}
			if(sX > maxX){
				maxX = sX;
			}
			if(eX < minX){
				minX = eX;
			}
			if(eX > maxX){
				maxX = eX;
			}
			if(sY < minY){
				minY = sY;
			}
			if(sY > maxY){
				maxY = sY;
			}
			if(eY < minY){
				minY = eY;
			}
			if(eY > maxY){
				maxY = eY;
			}
		}

		OFFSET[0] = minX;
		OFFSET[1] = minY;

		XDIM = (maxX - minX);
		YDIM = (maxY - minY);

		for(int i = 0; i<(wallArray.size()); i=i+4){
			//ROS_ERROR("Updater wall array loop: %d", i);
			single_wall[0] = wallArray[i];
			single_wall[1] = wallArray[i+1];
			single_wall[2] = wallArray[i+2];
			single_wall[3] = wallArray[i+3];
			tempwall.x1 = single_wall[0];
			tempwall.y1 = single_wall[1];
			tempwall.x2 = single_wall[2];
			tempwall.y2 = single_wall[3];
			tempwall.certainty = 32000;
			bool test1 = addToObs(single_wall);
			originalMap.NewWalls.push_back(tempwall);
		}
		
		//ROS_INFO("map initialized");

		
		last_load_time.clear();
		loadSrv.request.request = 1;
		if(loadClient.call(loadSrv)){
			wallStack = loadSrv.response.mappings;
			std::vector<float> walls;
			float objTemp1[4];
			//ROS_ERROR("wallStackSize: %d", wallStack.NewWalls.size());
			int size = (int)wallStack.NewWalls.size();
			for(int i = 0; i< size; i++){
				if(wallStack.NewWalls[i].certainty >= 0){
					objTemp1[0] = wallStack.NewWalls[i].x1;
					objTemp1[1] = wallStack.NewWalls[i].y1;
					objTemp1[2] = wallStack.NewWalls[i].x2;
					objTemp1[3] = wallStack.NewWalls[i].y2;

					last_load_time.push_back(ros::Time::now());
					bool test1 = addToObs(objTemp1);
				}
			}
		}
		completeMap.NewWalls.insert(completeMap.NewWalls.begin(), originalMap.NewWalls.begin(), originalMap.NewWalls.end());
		mapInitialized = 1;
}

bool lidarInitialized = 0;
bool lidarInitializing = 0;
void lidarCallback(sensor_msgs::PointCloud msg){
	if(mapInitialized){	
		if(lidarInitializing){
			return;
		}
		lidarInitializing = 1;
		lidarInitialized = 0;
		//int size = sizeof(msg.points)*sizeof(msg.points[0]);
		lidarx.clear();
		lidary.clear();

		*lastPointCloud_ptr = msg;
		try{
			(*laser_tfl_ptr).waitForTransform("world", msg.header.frame_id, ros::Time(0), ros::Duration(10.0));
			(*laser_tfl_ptr).lookupTransform("world", msg.header.frame_id, ros::Time(0), *laser_point_tf_ptr);
		}catch(tf::TransformException ex){
			//ROS_ERROR("%s",ex.what());
		}		


		float d;
		for(int i = 0; i<lastPointCloud_ptr->points.size(); ++i){
	tf::Vector3 point(msg.points[i].x,msg.points[i].y,0);
	tf::Vector3 point_tf = (*laser_point_tf_ptr) * point;
			if(std::isnan(point_tf.x()) || std::isnan(point_tf.y()) || std::isinf(point_tf.x()) || std::isinf(point_tf.y())){
				continue;
			}
			lidarx.push_back(point_tf.x());
			lidary.push_back(point_tf.y());
		}
		lidarInitialized = 1;
	}
}

float width;
float height;
std_msgs::Header gridheader;
nav_msgs::MapMetaData gridinfo;
//std::vector<signed char> occGrid;
bool gridInitializing = 0;
nav_msgs::OccupancyGrid occGrid;
void gridCallback(nav_msgs::OccupancyGrid msg){
	if(gridInitializing){
		return;
	}
	gridInitializing = 1;
	gridInitialized = 0;
	gridheader = msg.header;
	gridinfo = msg.info;
	resolution = msg.info.resolution;
	width = msg.info.width;
	height = msg.info.height;
	//occGrid = (std_msgs::Int8*)malloc(sizeof(std_msgs::Int8)*gridinfo.width*gridinfo.height);
	//for(int i =0; i < msg.data.size(); i++ ){
	//	occGrid.push_back( msg.data[i] );
	//}

	occGrid = msg;
	gridInitialized = 1;
}

void poseCallback(nav_msgs::Odometry msg){ // for re-calculation of the path when needed
    pose = msg;
}


bool ccw(float A[], float B[], float C[]){
     bool val = abs((C[1]-A[1]) * (B[0]-A[0])) > abs((B[1]-A[1]) * (C[0]-A[0]));
     return val;
}

float dist(float q1[], float q2[]){
    float d = sqrt(pow(q1[0]-q2[0],2) + pow(q1[1]-q2[1],2));
		//ROS_INFO("dist in fct f%", d);
    return d;
}

void regression(std::vector<float> x, std::vector<float> y){
    //if(x.size() != y.size()){
    //    throw exception("x,y unequal size");
    //}
	double n = x.size();

    double avgX = std::accumulate(x.begin(), x.end(), 0.0) / n;
    double avgY = std::accumulate(y.begin(), y.end(), 0.0) / n;

    double numerator = 0.0;
    double denominator = 0.0;

    for(int i=0; i<n; ++i){
        numerator += (x[i] - avgX) * (y[i] - avgY);
        denominator += (x[i] - avgX) * (x[i] - avgX);
    }

    //if(denominator == 0){
    //    throw exception("...");
    //}
	float b = numerator/denominator;
	float a = avgY-b*avgX;
	rosie_map_controller::WallDefinition wall;
	float temp[] = {x[0],a+b*x[0],x[x.size()-1],a+b*x[x.size()-1]};
	wall.x1 = temp[0];
	wall.y1 = temp[1];
	wall.x2 = temp[2];
	wall.y2 = temp[3];
	wall.certainty = 0;
	wallStack.NewWalls.push_back(wall);
	last_load_time.push_back(load_time);
	addToObs(temp);

    //return wall;// = b
		// a = avgY-b*avgX;
}

bool isIntersecting(float p1[], float p2[], float q1[], float q2[]) {
    return (((q1[0]-p1[0])*(p2[1]-p1[1]) - (q1[1]-p1[1])*(p2[0]-p1[0]))
            * ((q2[0]-p1[0])*(p2[1]-p1[1]) - (q2[1]-p1[1])*(p2[0]-p1[0])) < 0)
            and
           (((p1[0]-q1[0])*(q2[1]-q1[1]) - (p1[1]-q1[1])*(q2[0]-q1[0]))
            * ((p2[0]-q1[0])*(q2[1]-q1[1]) - (p2[1]-q1[1])*(q2[0]-q1[0])) < 0);
}

float p1[2];
float p2[2];
float p3[2];
float p4[2];
int nc = 0;
bool ints1;
bool ints2;
bool ints3;
bool ints4;
float center[2];

int checkIntersect(float A[], float B[]){         //array definition might be wrong

    nc = 1;
    for(int i =0 ; i<ALL_OBS.size(); i = i+8){
			//ROS_INFO("ALL OBS size %d", ALL_WALL.size());
				p1[0] =ALL_OBS[i];
				p1[1] =ALL_OBS[i+1];
				p2[0] =ALL_OBS[i+2];
				p2[1] =ALL_OBS[i+3];
				p3[0] =ALL_OBS[i+4];
				p3[1] =ALL_OBS[i+5];
				p4[0] =ALL_OBS[i+6];
				p4[1] =ALL_OBS[i+7];
				//ROS_INFO("x1 %f y1 %f x2 %f y2 %f", p1[0], p1[1], p2[0], p2[1] );

				 ints1 = isIntersecting(p1,p2,A,B);
				 ints2 = isIntersecting(p1,p4,A,B);
				 ints3 = isIntersecting(p3,p2,A,B);
				 ints4 = isIntersecting(p3,p4,A,B);

				/* center[0] = (p2[0]-p1[0])/2;
				 center[1] = (p2[1]-p1[1])/2;
				 ints5 = isIntersectingCap(center, A, B);
				 center[0] = (p3[0]-p4[0])/2;
				 center[1] = (p3[1]-p4[1])/2;
				 ints6 = isIntersectingCap(center, A, B);*/
      	//if(ints5 == 0 and ints6 == 0 and ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==1){
      	if(ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==1){
            nc = 1; //no intersection
	    return -1;
        }else{
            nc = 0;
	    return i/8;
	    break;
        }
    }
}

float randnum;
float randZO(float min, float max){
		srand(ros::Time::now().nsec);
    randnum = (rand() % ((int) max*100))/100.0f; // + (min))/100.0f;
	  //ROS_INFO("r: %f",r);
		return randnum;
}

int newGridInit = 0;
//nav_msgs::OccupancyGrid mapgrid;
void buildGrid(){
	int px;
	int py;
	float wallDist;
	
	gridheader.seq = gridheader.seq +1;
	gridheader.stamp = ros::Time::now();
	gridinfo.map_load_time = load_time;

	occGrid.header = gridheader;
	occGrid.info = gridinfo;
	
	newGridInit = 1;
}

void callService(){
	newGridSrv.request.newGrid = occGrid;//mapgrid;
	occClient.call(newGridSrv);

}

bool markersInitialized = 0;
void buildWallMarkers(){
	/*if(!markersInitialized){
		rosie_map_controller::WallDefinition wall;
		wall.x1 = 1.80;
		wall.y1 = 0.50;
		wall.x2 = 2.0;
		wall.y2 = 0.5;
		wall.certainty = 1000;
		wallStack.NewWalls.push_back(wall);	
	}*/
	for(int i = 0; i<wallStack.NewWalls.size(); i++){
		mapSrv.request.send.NewWalls.push_back(wallStack.NewWalls[i]);
	}	
	storeMapClient.call(mapSrv);
	completeMap.NewWalls.insert(completeMap.NewWalls.begin(), wallStack.NewWalls.begin(), wallStack.NewWalls.end()); 

	visualization_msgs::MarkerArray all_marker;

	visualization_msgs::Marker wall_marker;
	wall_marker.header.frame_id = "/map";
	wall_marker.header.stamp = ros::Time();
	wall_marker.ns = "world";
	wall_marker.type = visualization_msgs::Marker::CUBE;
	wall_marker.action = visualization_msgs::Marker::ADD;
	wall_marker.scale.y = 0.01;
	wall_marker.scale.z = 0.2;
	wall_marker.color.a = 1.0;
	wall_marker.color.r = (255.0/255.0);
	wall_marker.color.g = (0.0/255.0);
	wall_marker.color.b = (0.0/255.0);
	wall_marker.pose.position.z = 0.2;
	std::string line;
	int wall_id = 0;
	for(int i = 0; i < completeMap.NewWalls.size(); ++i){

		rosie_map_controller::WallDefinition wall = completeMap.NewWalls[i];
		float x1 = wall.x1;
		float y1 = wall.y1;
		float x2 = wall.x2;
		float y2 = wall.y2;

        std::istringstream line_stream(line);

        //line_stream << x1 << y1 << x2 << y2;

		// angle and distance
		double angle = atan2(y2-y1,x2-x1);
		double dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

		// set pose
		wall_marker.scale.x = std::max(0.01,dist);

		wall_marker.pose.position.x = (x1+x2)/2;
		wall_marker.pose.position.y = (y1+y2)/2;
		wall_marker.text= "."; //line_stream.str();
		tf::Quaternion quat; quat.setRPY(0.0,0.0,angle);
		tf::quaternionTFToMsg(quat, wall_marker.pose.orientation);	

		wall_marker.id = wall_id;
	  	all_marker.markers.push_back(wall_marker);
		wall_id++;
	}	
	wall_viz_pub.publish(all_marker);

	markersInitialized = 1;
}

float test_points1[2];
float test_points2[2];
std::vector<float> reduced_lidar_x;
std::vector<float> reduced_lidar_y;
std::vector<float> pointDist;
std::vector<float> gradient;
std::vector<float> temporalpoints;
std::vector<float> finalX;
std::vector<float> finalY;
void detectnewWalls(){
	reduced_lidar_x.clear();
	reduced_lidar_y.clear();
	int test;
	for(int i =0; i<lidarx.size(); i++){
		test_points1[0] = lidarx[i];
		test_points1[1] = lidary[i];
		test_points2[0] = pose.pose.pose.position.x;
		test_points2[1] = pose.pose.pose.position.y;		
		test = checkIntersect(test_points1,test_points2);
		if(test<0){ // lidar not in any wall
			reduced_lidar_x.push_back(lidarx[i]);
			reduced_lidar_y.push_back(lidary[i]);
		}else{
			ROS_ERROR("TEST: %d vs size: %d", test, wallStack.NewWalls.size());
			wallStack.NewWalls[test].certainty = -wallStack.NewWalls[test].certainty+1;
		}
	}
	/*
	pointDist.clear();
	for(int i = 0; i<reduced_lidar_x.size(); i++){
		pointDist.push_back(sqrt(pow(reduced_lidar_x[i],2)+pow(reduced_lidar_y[i],2)));
	}

	srand(ros::Time::now().toSec());
	int idx = std::rand()%(pointDist.size() -1);
	gradient.clear();
	temporalpoints.clear();
	//temporalpoints.insert(temporalpoints.end(), pointDist.begin()+idx, pointDist.end());
	//temporalpoints.insert(temporalpoints.end(), pointDist.begin(), pointDist.begin()+idx-1);
	finalX.clear();
	finalY.clear();
	if(reduced_lidar_x.size()>5){
		for(int i = 0; i<pointDist.size()-1; i++){
			gradient.push_back((pointDist[i]-pointDist[i+1])/2);
		}
		float sum = 0 ;
		for(int i = 0; i< gradient.size(); i++){
			sum += gradient[i];
		}
		sum = sum/gradient.size();
		int del = 0;
		for(int i = 0; i< gradient.size(); i++){
			if(std::abs(gradient[i]-sum)<=0.1){
				/*if(i+1+idx < reduced_lidar_x.size()-1){
					finalX.push_back(reduced_lidar_x[i+1+idx]);
					finalY.push_back(reduced_lidar_y[i+1+idx]);
				}else{
					finalX.push_back(reduced_lidar_x[i+1+idx-reduced_lidar_x.size()]);
					finalY.push_back(reduced_lidar_y[i+1+idx-reduced_lidar_x.size()]);
				}
				finalX.push_back(reduced_lidar_x[i+1]);
				finalY.push_back(reduced_lidar_y[i+1]);
			}
		}
	}*/
	if(finalX.size()>5){	
		regression(reduced_lidar_x, reduced_lidar_y);
		buildGrid();
		callService();
	}
}

void forgetWalls(){
	for(int i = 0; i<wallStack.NewWalls.size(); i++){
		if(wallStack.NewWalls[i].certainty >= 0){
			if((load_time.toSec()-last_load_time[i].toSec())>forgettingTime*wallStack.NewWalls[i].certainty){
				wallStack.NewWalls[i].certainty = -(wallStack.NewWalls[i].certainty);
			}
		}
	}
}

bool grip_command;
int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_rrt");

    ros::NodeHandle n;
		ros::Subscriber lidar_sub = n.subscribe<sensor_msgs::PointCloud>("/my_cloud",100, lidarCallback);
		ros::Subscriber wall_sub = n.subscribe<visualization_msgs::MarkerArray>("/maze_map", 1000, wallCallback);
		ros::Subscriber occ_sub = n.subscribe<nav_msgs::OccupancyGrid>("/rosie_occupancy_grid", 1, gridCallback);
		ros::Subscriber pose_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, poseCallback);
		ros::Subscriber loc_cert_sub = n.subscribe<std_msgs::Float32>("/localization_certainty", 1, certaintyCallback);
		wallStack_pub = n.advertise<rosie_map_controller::MapStoring>("/wall_stack", 10);
		wall_viz_pub = n.advertise<visualization_msgs::MarkerArray>("/new_wall_visualization", 1);
		storeMapClient = n.serviceClient<rosie_map_controller::RequestMapStoring>("request_store_mapping");
		loadClient = n.serviceClient<rosie_map_controller::RequestLoading>("request_load_mapping");
		occClient = n.serviceClient<rosie_map_updater::NewGrid>("update_grid");
		static tf::TransformBroadcaster br;

        ros::Rate loop_rate(10);
	laser_tfl_ptr.reset(new tf::TransformListener);
	laser_point_tf_ptr.reset(new tf::StampedTransform);
	lastPointCloud_ptr.reset(new sensor_msgs::PointCloud);


		//objInit();
		//batInit();

    while(ros::ok()){
		load_time = ros::Time::now();
		//loop_rate.sleep();
		//certaintyValue= 0.5;
		ROS_ERROR("certaintyValue %f", certaintyValue);
		if(mapInitialized && gridInitialized && lidarInitialized && (certaintyValue < certaintyLimit)){
				ROS_ERROR("forgetWalls");
				forgetWalls();
				ROS_ERROR("detectWalls");
				detectnewWalls();
				//buildWallMarkers();
				//lidarInitializing = 0;
				//gridInitializing =0;
				ROS_ERROR("transform");
				tf::Transform transform;
				transform.setOrigin( tf::Vector3(0,0, 0) );
				tf::Quaternion qtf;
				qtf.setRPY(0, 0, 0);
				transform.setRotation( qtf );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "path"));


		}
		if(mapInitialized){
			ROS_ERROR("mapInitialized - map updater");
			wallStack_pub.publish(completeMap);
			buildWallMarkers();
			}
		lidarInitializing = 0;
		gridInitializing =0;
		ROS_ERROR("Spin");
		ros::spinOnce();
		ROS_ERROR("Sleep");
		loop_rate.sleep();
	}
}
