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

float PI = 3.1415926f;
float robotsize = 0.2f;
float OFFSET[] = {0,0};
float XDIM;
float YDIM;
int forgettingTime = 60; // in sec

ros::Time load_time;
std::vector<ros::Time> last_load_time;

ros::ServiceClient occClient;
rosie_map_updater::NewGrid newGridSrv;
ros::Publisher wallStack_pub;

std::vector<float> lidarx;
std::vector<float> lidary;
std::vector<float> wallArray;
std::vector<float> ALL_OBS;

nav_msgs::Odometry pose;

int mapInitialized = 0;
int gridInitialized = 0;

visualization_msgs::MarkerArray all_marker;
std::vector<std::vector<float> > newWalls;
int wallCounter = -1;

float certaintyValue;
float certaintyLimit = 0.4;
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
void wallCallback(const visualization_msgs::MarkerArray msg){
	markers.clear();
	for(int i =0; i<msg.markers.size();i++){
		markers.push_back(msg.markers[i]);
	}

	if(!mapInitialized){
		ROS_INFO("Initializing!");
		int numbMarkers = msg.markers.size();
		czone = robotsize/(2.0f) + 0.02f; //additional extra security distance

		wallArray.clear();
		wallArray.resize(4*numbMarkers);
		for(int i = 0; i < 4*numbMarkers; ++i){
			wallArray[i] = -1.0f;
		}


		minX = minY = maxX = maxY = 0;
		for(int k = 0; k < numbMarkers; ++k){
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
		float single_wall[4];
		for(int i = 0; i<(wallArray.size()); i=i+4){
			single_wall[0] = wallArray[i];
			single_wall[1] = wallArray[i+1];
			single_wall[2] = wallArray[i+2];
			single_wall[3] = wallArray[i+3];

			bool test1 = addToObs(single_wall);
		}
		mapInitialized = 1;
		//ROS_INFO("map initialized");


		std::vector<ros::Time> now (4,ros::Time::now());
		last_load_time.clear();
		if(loadClient.call(loadSrv)){
			wallStack = loadSrv.response.mappings;
			std::vector<float> walls;
			float objTemp1[4];
			for(int i = 0; i< wallStack.NewWalls.size(); i++){
				if(wallStack.NewWalls[i].certainty >= 0){
					objTemp1[0] = wallStack.NewWalls[i].x1;
					objTemp1[1] = wallStack.NewWalls[i].y1;
					objTemp1[2] = wallStack.NewWalls[i].x2;
					objTemp1[3] = wallStack.NewWalls[i].y2;
					//walls.push_back(objTemp1[0]);
					//walls.push_back(objTemp1[1]);
					//walls.push_back(objTemp1[2]);
					//walls.push_back(objTemp1[3]);
					//newWalls.push_back(walls);
					last_load_time.insert(last_load_time.begin(), now.begin(), now.end());
					bool test1 = addToObs(objTemp1);
				}
			}
		}
	}
}
void lidarCallback(sensor_msgs::PointCloud msg){

	//int size = sizeof(msg.points)*sizeof(msg.points[0]);
	lidarx.clear();
	lidary.clear();
	for(int i = 0; i<msg.points.size(); ++i){
		lidarx.push_back(msg.points[i].x);
		lidary.push_back(msg.points[i].y);
	/*
	float meanR;
	float range[5];
	for(int i = 2; i<lidarx.size()-2; i++){
		range[0] = sqrt(pow(lidarx[i-2],2)+pow(lidary[i-2],2));
		range[1] = sqrt(pow(lidarx[i-1],2)+pow(lidary[i-1],2));
		range[2] = sqrt(pow(lidarx[i],2)+pow(lidary[i],2));
		range[3] = sqrt(pow(lidarx[i+1],2)+pow(lidary[i+1],2));
		range[5] = sqrt(pow(lidarx[i+1],2)+pow(lidary[i+1],2));
		meanR = (range[0] + range[1] + range[2])/3-range[0];
	*/
	}
		//int px = msg.points[i].x;
		//int py = msg.points[i].y;
		//if(abs(px) < win_cells && abs(py) < win_cells){
		//	window[(int) (py*2*win_cells + px)] = 125;
		//}
	}

float width;
float height;
std_msgs::Header gridheader;
nav_msgs::MapMetaData gridinfo;
std::vector<signed char> occGrid;
void gridCallback(nav_msgs::OccupancyGrid msg){
	gridheader = msg.header;
	gridinfo = msg.info;
	resolution = msg.info.resolution;
	width = msg.info.width;
	height = msg.info.height;
	//occGrid = (std_msgs::Int8*)malloc(sizeof(std_msgs::Int8)*gridinfo.width*gridinfo.height);
	for(int i =0; i < msg.data.size(); i++ ){
		occGrid.push_back( msg.data[i] );
	}
	gridInitialized = 1;
}

void poseCallback(nav_msgs::Odometry msg){ // for re-calculation of the path when needed
    pose = msg;
		pose.pose.pose.position.x = 0.25f;
		pose.pose.pose.position.y = 0.40f;
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
		wallCounter++;
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
int checkIntersect(float A[]){         //array definition might be wrong
		nc = true;
		int cnt = 0;
    for(int i =0 ; i<ALL_OBS.size(); i = i+8){

        //float abst = sqrt(pow(OBS[i+2]-OBS[i+0],2.0)+pow(OBS[i+3]-OBS[i+1],2.0));
        //float m1 = (OBS[i+2]-OBS[i+0])/dist;
        //float m2 = (OBS[i+3]-OBS[i+1])/dist;
				p1[0] =ALL_OBS[i];
				p1[1] =ALL_OBS[i+1];
				p2[0] =ALL_OBS[i+2];
				p2[1] =ALL_OBS[i+3];
				p3[0] =ALL_OBS[i+4];
				p3[1] =ALL_OBS[i+5];
				p4[0] =ALL_OBS[i+6];
				p4[1] =ALL_OBS[i+7];
				center[0] = (p3[0]-p1[0])/2;
				center[1]	= (p3[1]-p1[1])/2;

				//ROS_INFO("x1 %f y1 %f x2 %f y2 %f", p1[0], p1[1], p2[0], p2[1] );

				 ints1 = isIntersecting(p1,p2,A,center);
				 ints2 = isIntersecting(p1,p4,A,center);
				 ints3 = isIntersecting(p3,p2,A,center);
				 ints3 = isIntersecting(p3,p4,A,center);

				// ints1 = ccw(A,P1,P4) != ccw(B,P1,P4) and ccw(A,B,P1) != ccw(A,B,P4);
        // ints2 = ccw(A,P1,P2) != ccw(B,P1,P2) and ccw(A,B,P1) != ccw(A,B,P2);
        // ints3 = ccw(A,P3,P2) != ccw(B,P3,P2) and ccw(A,B,P3) != ccw(A,B,P2);
        // ints4 = ccw(A,P3,P4) != ccw(B,P3,P4) and ccw(A,B,P3) != ccw(A,B,P4);
				if( i < wallArray.size()*8 || (i >= wallArray.size()*8 && wallStack.NewWalls[cnt].certainty>=0)){
					if(ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==0){
							nc = 0;
					}else{
							nc = 1;
							break;
					}
				}else if((i >= wallArray.size()*8 && wallStack.NewWalls[cnt].certainty<0)){
					if(ints1==0 and ints2==0 and ints3==0 and ints4==0 and nc ==0){
							nc = -cnt;
							break;
					}
				}

				if(i >= wallArray.size()*8){
					cnt++;
				}
    }
		return nc;
}

float randnum;
float randZO(float min, float max){
		srand(ros::Time::now().nsec);
    randnum = (rand() % ((int) max*100))/100.0f; // + (min))/100.0f;
	  //ROS_INFO("r: %f",r);
		return randnum;
}

int newGridInit = 0;
nav_msgs::OccupancyGrid mapgrid;
void buildGrid(){
	float wallDist;
	for(int i = 0; i<wallStack.NewWalls.size(); i++){
		wallDist = sqrt(pow(wallStack.NewWalls[i].x1-wallStack.NewWalls[i].x2,2) + pow(wallStack.NewWalls[i].y1-wallStack.NewWalls[i].y2,2))/resolution;
		for(float d = 0.0; d <= wallDist; d+=resolution){
			int px = (int)((d*((wallStack.NewWalls[i].x2-wallStack.NewWalls[i].x1)/wallDist)+wallStack.NewWalls[i].x1)/resolution);
			int py = (int)((d*((wallStack.NewWalls[i].y2-wallStack.NewWalls[i].y1)/wallDist)+wallStack.NewWalls[i].y1)/resolution);
			occGrid[py*width+px] = 125;

			for(int y = -czone; y <= czone; y++){
				for(int x = -czone; x <= czone; x++){
					if((px+x) >= 0 && (px+x) < width && (py+y) >= 0 && (py+y) < height){
						if(occGrid[(py+y)*width+px+x] != 125){
							occGrid[(py+y)*width+px+x] = 124;
						}
					}
				}
			}
		}
	}

	gridheader.seq = gridheader.seq +1;
	gridheader.stamp = ros::Time::now();
	gridinfo.map_load_time = load_time;

	mapgrid.header = gridheader;
	mapgrid.info = gridinfo;
	mapgrid.data.clear();
	for(int i = 0; i < width*height; ++i){
		mapgrid.data.push_back(occGrid[i]);
	}
	newGridInit = 1;
}

void callService(){
	newGridSrv.request.newGrid = mapgrid;
	occClient.call(newGridSrv);

}

bool markersInitialized = 0;
void buildWallMarkers(){
	//mapSrv.request.send = wallStack;
	//storeMapClient.call(mapSrv);
	wallStack_pub.publish(wallStack);
	/*
	all_marker.markers.clear();
	//all_marker.resize(markers.size());
	for(int i = 0; i<markers.size();i++){
		markers[i].id = i;
		markers[i].header.stamp = ros::Time();
		all_marker.markers.push_back(markers[i]);
	}
	wall_id = markers.size()-1;

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
	double x1, x2, y1, y2;
	for(int i = 0; i< newWalls.size(), i++){
		x1= newWalls[i][0];
	  x2= newWalls[i][2];
	  y1= newWalls[i][1];
	  y2= newWalls[i][3];

		std::istringstream line_stream(line);

		line_stream >> x1 >> y1 >> x2 >> y2;

	  // angle and distance
	  double angle = atan2(y2-y1,x2-x1);
	  double dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

	  // set pose
	  wall_marker.scale.x = std::max(0.01,dist);
	  wall_marker.pose.position.x = (x1+x2)/2;
	  wall_marker.pose.position.y = (y1+y2)/2;
	  wall_marker.text=line_stream.str();
	  tf::Quaternion quat; quat.setRPY(0.0,0.0,angle);
	  tf::quaternionTFToMsg(quat, wall_marker.pose.orientation);

	  // add to array
	  wall_marker.id = markers.size();
	  all_marker.markers.push_back(wall_marker);
		wall_id++;
		*/

		markersInitialized = 1;
	}

float test_points[2];
std::vector<float> reduced_lidar_x;
std::vector<float> reduced_lidar_y;

void detectnewWalls(){
	reduced_lidar_x.clear();
	reduced_lidar_y.clear();
	int test;
	for(int i =0; i<lidarx.size(); i++){
		test_points[0] = lidarx[i];
		test_points[1] = lidary[i];
		test = checkIntersect(test_points);
		if(test==1){
			reduced_lidar_x.push_back(lidarx[i]);
			reduced_lidar_y.push_back(lidary[i]);
		}else if(test < 0){
			wallStack.NewWalls[-test].certainty = -wallStack.NewWalls[-test].certainty+1;
		}
	}
	std::vector<float> pointDist;
	for(int i = 0; i<reduced_lidar_x.size(); i++){
		pointDist.push_back(sqrt(pow(reduced_lidar_x[i],2)+pow(reduced_lidar_y[i],2)));
	}
	std::vector<float> gradient;
	std::vector<float> temporalpoints;
	srand(ros::Time::now().toSec());
	int idx = std::rand()%(pointDist.size() -1);

	temporalpoints.insert(temporalpoints.begin(), pointDist.begin()+idx+1, pointDist.end());
	temporalpoints.insert(temporalpoints.begin(), pointDist.begin(), pointDist.begin()+idx);
	if(reduced_lidar_x.size()>5){
		for(int i = 0; i<pointDist.size()-1; i++){
			gradient.push_back((pointDist[i]-pointDist[i+1])/2);
		}
		float sum = 0 ;
		for(int i = 0; i< gradient.size(); i++){
			sum += gradient[i];
		}
		sum = sum/gradient.size();
		for(int i = 0; i< gradient.size(); i++){
			if(std::abs(gradient[i]-sum)>0.1){
				gradient.erase(gradient.begin()+i);
				reduced_lidar_x.erase(reduced_lidar_x.begin()+i+2+idx);
				reduced_lidar_y.erase(reduced_lidar_y.begin()+i+2+idx);
				i--;
			}
		}
		regression(reduced_lidar_x, reduced_lidar_y);
		buildGrid();
		callService();
		buildWallMarkers();
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
		ros::Subscriber occ_sub = n.subscribe<nav_msgs::OccupancyGrid>("/rosie_occupancy_grid", 1000, gridCallback);
		ros::Subscriber pose_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, poseCallback);
		ros::Subscriber loc_cert_sub = n.subscribe<std_msgs::Float32>("/localization_certainty", 10, certaintyCallback);
		wallStack_pub = n.advertise<rosie_map_controller::MapStoring>("/wall_stack", 10);
		storeMapClient = n.serviceClient<rosie_map_controller::RequestMapStoring>("request_store_objects");
		loadClient = n.serviceClient<rosie_map_controller::RequestLoading>("request_load_mapping");
		occClient = n.serviceClient<rosie_map_updater::NewGrid>("update_grid");
		static tf::TransformBroadcaster br;

    ros::Rate loop_rate(10);


		//objInit();
		//batInit();

    while(ros::ok()){
			load_time = ros::Time::now();
			//loop_rate.sleep();
			if(mapInitialized and gridInitialized and (certaintyValue < certaintyLimit)){
					forgetWalls();
					detectnewWalls();

					//occ_pub.publish(mapgrid);
					//wall_pub.publish(all_marker);
					tf::Transform transform;
					transform.setOrigin( tf::Vector3(0,0, 0) );
					tf::Quaternion qtf;
					qtf.setRPY(0, 0, 0);
					transform.setRotation( qtf );
					br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "path"));

		 		ros::spinOnce();
				loop_rate.sleep();
			}
		}
}
