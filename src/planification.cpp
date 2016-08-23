#include <ros/ros.h>
#include "rrt.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>


#define SMOOTHING_TOLERANCE 5
#define SMOOTHING_STRENGTH 0.5
#define SMOOTHING_DATA_WEIGHT 0.5
#define L1 0.01

#define SCALING 0.01

nav_msgs::Odometry current_pos;

static bool cpt=true;
void my_mouse_callback( int event, int x, int y, int flags,  void* ptr);

using namespace cv;
void curPosCb(const nav_msgs::Odometry::ConstPtr& msg);

void showTree(Node *q_i, cv::Mat* map);
void drawPath(std::vector<Node*> path, Mat *map);
std::vector<Node*> rrtPath(Node *end, Node *tree);
std::vector<Node*> pathSmoothing(std::vector<Node*> path, Mat *map);
Mat traitementImage(Mat &map);

double thetar = 0;
static int cpteur = 0;
static double du = 1000000;

geometry_msgs::Point sonar_data[8];

void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& pointCloud) {
  int n_sensors = 8; // 8 sonars
  for(int i=0;i<n_sensors;i++){
    sonar_data[i].x = pointCloud->points[i].x;
    sonar_data[i].y = pointCloud->points[i].y;
  }
}

float sonarNorm(float x, float y){ return sqrt(x*x+y*y); }

int main(int argc, char **argv) {
  ros::init(argc, argv, "planification");
  ros::NodeHandle n;

  Node tree(127,322);
  cv::Point p;

  Mat map_init, map;
  map = imread("/home/amucaj/map/map.pgm", CV_LOAD_IMAGE_GRAYSCALE);
  map = traitementImage(map);

  namedWindow("Selection point de départ", WINDOW_NORMAL);
  while(cpt) {
    cvSetMouseCallback("Selection point de départ", myMouseCallback, (void*) &p);
    imshow("Selection point de départ", map);
    cvWaitKey(5);
  }
  cpt = true;

  tree.x = p.x; tree.y = p.y;

  std::cout << "Initializing tree" << std::endl;

  std::srand(std::time(0));
  while(notInFreeSpace(&tree,map) || !pixelTest(tree,map,0)) {
    tree.x = std::rand() % map.rows; tree.y = std::rand() % map.cols;
  }

  std::cout << tree.x << " " << tree.y << std::endl;
  std::cout << "Building graph" << std::endl;

  _rrt(&tree, 6000, map);

  std::cout << "Drawing graph" << std::endl;    

  Mat m_bis; map.copyTo(m_bis);

  showTree(&tree,&m_bis);
  namedWindow("RRT graph", WINDOW_NORMAL);
  imshow("RRT graph", m_bis);    
  
  std::cout << "Drawing path solution" << std::endl;
  Node end;

  cpt = true;
  while(cpt) {
    cvSetMouseCallback("Selection point d'arrivée", my_mouse_callback,(void*) &p);
    cvWaitKey(5);
  }

  end.x = p.x; end.y = p.y;

  while(notInFreeSpace(&end,map)) {
    end.x = std::rand() % map.rows, end.y = std::rand() % map.cols;
  }

  vector<Node*> path = pathSmoothing(rrt_path(&end, &tree), &map);
  drawPath(path,&map);

  namedWindow("Path", WINDOW_NORMAL);
  imshow("Path", map);
  waitKey(0);

  imwrite("/home/amucaj/MapReworked.pgm", map);

  // Inversion axe Y
  for (int i = 0; i < path.size(); ++i) {
    path[i]->y = -path[i]->y;
  }

  ros::Subscriber sonar_sub = n.subscribe<sensor_msgs::PointCloud>("/rosaria_driver/sonar", 1, sonarCallback);
  float obstacle_distance_check = 0.25 + 0.5;
  int obstacle_detected_sonar_id;
  float obstacle_detected_min_distance;
  float temp_norm, sonar_speed;

  obstacle_detected_sonar_id = -1;
  obstacle_detected_min_distance = 99999;

  ros::Subscriber sub_cur_pos = n.subscribe("rosaria_driver/pose", 1, curPosCb);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("rosaria_driver/cmd_vel",1);
  geometry_msgs::Twist t;

  float k=0;
  double thetas=0, thetae=0, d=0;
  double ys[2] = {0,0}, ps[2] = {0,0}, xs[2] = {0,0};
  double pbis[2] = {0,0};
  int i=path.size() - 2;
  double u1 = 0.50;

  double d_tmp=100000000;

  while(ros::ok()) {
    if (i > 0 && i < path.size()) {
      obstacle_detected_min_distance = 99999;
      obstacle_detected_sonar_id = - 1;
      for(int j=0; j <= 7; j++) {
        temp_norm = sonarNorm(sonar_data[j].x,sonar_data[j].y);
        if(temp_norm < obstacle_distance_check) {
          if(obstacle_detected_min_distance > temp_norm) {
            obstacle_detected_min_distance = temp_norm;
            obstacle_detected_sonar_id = j;
          }
        }
      }

      if(obstacle_detected_sonar_id == -1) {
        thetas = atan2(path[i-1]->y - path[i+1]->y,path[i-1]->x - path[i+1]->x);
        thetae = thetar - thetas;
        if(fabs(thetae) > 3.1459/2) {
          t.linear.x = 0;
          if(thetae > 0)
            t.angular.z = 0.5;
          else
            t.angular.z = -0.5;
        } else {
          ys[0] = cos(thetae);
          ys[1] = sin(thetae);
          xs[0] = cos(thetas);
          ys[0] = sin(thetas);
          pbis[0] = cos(thetar)*L1 + current_pos.pose.pose.position.x+path[path.size()-1]->x*SCALING;
          pbis[1] = -sin(thetar)*L1 + current_pos.pose.pose.position.y+path[path.size()-1]->y*SCALING;
          ps[0] = (pbis[0] - path[i]->x*SCALING);
          ps[1] = (pbis[1] - path[i]->y*SCALING);
          d = sqrt(pow(ps[0]*ys[0] + ys[1]*ps[1],2));
          du = ps[0]*xs[0] + ps[1]*xs[1];
          if(du > 0) {
            --i;
          }

          k=1;
          t.linear.x = u1;
          t.angular.z = 0.1*(-u1 /(L1*cos(thetae)) * sin(thetae) - u1/cos(thetae) * k * d);
        }
      } else {
        sonar_speed = 0;
        u1 = 0.1;
        float coeff[] = { -1.2 ,-1.0, -0.8, -0.6 , 0.6, 0.8 , 1.0, 1.2};
        for (int j = 0; j <= 7; ++j) {
          sonar_speed -=  sonarNorm(sonar_data[j].x, sonar_data[j].y)*coeff[j];
        }
        t.linear.x = u1;
        t.angular.z = sonar_speed*0.2;
      }
    } else {
      t.linear.x = 0;
      t.angular.z = 0;
    }
    pub.publish(t);
    ros::spinOnce();
  }
  return 0;
}


Mat traitementImage(Mat &map){
  for(int i = 0; i < map.rows; i++) {
    for(int j = 0; j < map.cols; j++) {
      cv::Scalar c = map.at<uchar>(i,j);
      if(c[0] < 240)
        map.at<uchar>(i,j) = 0;
    }
  }
  cv::Mat dst, dst_erode;
  cv::dilate(map,dst,Mat(), Point(-1,-1) ,5);
  cv::erode(dst, dst_erode,Mat(), Point(-1,-1), 5);
  return dst_erode;
}

void myMouseCallback(int event, int x, int y, int flags, void * ptr) {
  cv::Point * p = (cv::Point *) ptr;
  p->x=x;
  p->y=y;
  switch(event) {
    case CV_EVENT_LBUTTONDOWN:
    cpt=false;
    break;
    default:
    break;
  }
}

void showTreeRec(Node* q_i, cv::Mat* map) {
  int n = q_i->forest.size(), i;
  for(i=0; i < n; i++) {
    cv::Point point_q_i(q_i->x, q_i->y);
    cv::Point point_q_f(q_i->forest[i]->x, q_i->forest[i]->y);
    cv::Scalar color_c(0, 0, 255);
    line(*map, point_q_i, point_q_f, color_c);
    showTreeRec(q_i->forest[i], map);
  }
  return; 
}

void showTree(Node *tree, cv::Mat* map) {	
  return showTreeRec(tree, map);
}

void drawPath(std::vector<Node*> path, Mat *map) {
	int n = path.size();
	for(int i=0; i < (n-1); i++) {
		cv::Point start(path[i]->x, path[i]->y);
		cv::Point end(path[i+1]->x, path[i+1]->y);
		cv::Scalar c(0, 0, 0);
		cv::line(*map, start, end, c);
	}
}

std::vector<Node*> rrtPath(Node *end, Node *tree) {
	Node *graph_end = closestTo(end, tree), *temp;
	std::vector<Node*> path;
	temp = graph_end;
	while(temp) {
		path.push_back(temp);
		temp = temp->parent;
	}
	return path;
}

std::vector<Node*> pathSmoothing(std::vector<Node*> path, Mat *map) {
  int n = path.size();
  std::vector<Node*> newpath;
  Node *temp; 
  bool point_ok;
  double change;
  for(int i=0;i<n;i++) {
    newpath.push_back(path[i]);
  }
  change = SMOOTHING_TOLERANCE;
  while(change >= SMOOTHING_TOLERANCE) {
    change = 0;
    for(int i=1; i < (n-1); i++) {
      temp = new Node(newpath[i]->x,newpath[i]->y);
      temp->x += SMOOTHING_STRENGTH*(path[i]->x - temp->x);
      temp->x += SMOOTHING_DATA_WEIGHT*(newpath[i-1]->x + newpath[i+1]->x - 2*temp->x);
      temp->y += SMOOTHING_STRENGTH*(path[i]->y - temp->y);
      temp->y += SMOOTHING_DATA_WEIGHT*(newpath[i-1]->y + newpath[i+1]->y - 2*temp->y);
      if(_collisionWithObject(newpath[i-1], temp, *map) && _collisionWithObject(temp, newpath[i+1], *map)) {
        change += sqrt((temp->x - newpath[i]->x)*(temp->x - newpath[i]->x)+(temp->y - newpath[i]->y)*(temp->y - newpath[i]->  y));
        delete newpath[i];
        newpath[i] = temp;
      } else {
        delete temp;
      }
    }
  }
  return newpath;
}

void curPosCb(const nav_msgs::Odometry::ConstPtr& msg) {
  thetar = tf::getYaw(msg->pose.pose.orientation);
  current_pos.pose.pose.position.x = msg -> pose.pose.position.x;
  current_pos.pose.pose.position.y = msg -> pose.pose.position.y;
  current_pos.pose.pose.position.z = msg -> pose.pose.position.z;
  current_pos.pose.pose.orientation.x = msg -> pose.pose.orientation.x;
  current_pos.pose.pose.orientation.y = msg -> pose.pose.orientation.y;
  current_pos.pose.pose.orientation.z = msg -> pose.pose.orientation.z;
  current_pos.pose.pose.orientation.w = msg -> pose.pose.orientation.w;
}
