#ifndef RRT_HPP
#define RTT_HPP

#include <cstdlib>
#include <iostream>
#include <ctime>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "node.hpp"

#define LARGEUR_ROBOT 100
#define LONGUEUR_ROBOT 200

using namespace cv;

static int deltaQ = 40;
static int nb_in_tree = 0;

Node* closestToRec(Node* q_rand, Node* q_i) {
  int n = q_i->forest.size(), i;
  double d, d_temp;
  Node *q_temp, *q_res;
  
  q_res = q_i;
  d = sqrt((q_i->x - q_rand->x)*(q_i->x - q_rand->x) + (q_i->y - q_rand->y)*(q_i->y - q_rand->y));
  
  for(i=0;i<n;i++) {
    q_temp = closestToRec(q_rand, q_i->forest[i]);
    d_temp = sqrt((q_temp->x - q_rand->x)*(q_temp->x - q_rand->x) + (q_temp->y - q_rand->y)*(q_temp->y - q_rand->y));
    q_res = (d_temp < d) ? q_temp : q_res; 
    d = (d_temp < d) ? d_temp : d;	
  } 
  return q_res; 
}

Node* closestTo(Node* q_rand, Node *tree) {
  return closestToRec(q_rand,tree);
}

void extend(Node* q_rand, Node* tree, Mat map);

double norme(Node* n_rand, Node* n_near) {
 return sqrt(pow(n_rand->x - n_near->x,2) + pow(n_rand->y - n_near->y,2));
}

bool pixelTest(Node& u, Mat &map, int mode) {
 int R=40;
 int xTemp = u.x - R;
 int yTemp = u.y - R;
 if(mode == 0){
  for(int i = 0; i < 2*R; ++i) {	
  	for(int j = 0; j < 2*R; ++j) {
  		// if(pow(xTemp - u.x + i,2) + pow(yTemp - u.y + j,2) <= pow(R,2))
  		{
        if(((xTemp + i >= 0) && (xTemp + i < map.cols)) && ((yTemp  + j>= 0) && (yTemp  + j< map.rows))) {
  		    cv::Scalar c = map.at<uchar>(yTemp + j, xTemp+i);
  		    if(c[0] < 254)
  		      return false;
        }
  		}
  	}
  }
 } else {  
  if(mode == 1 || mode == 4) {
    for (int j = 0; j < 2*R ; ++j) {
      if(((xTemp >= 0) && (xTemp < map.cols)) && ((yTemp  + j >= 0) && (yTemp  + j< map.rows))) {
        cv::Scalar c = map.at<uchar>(yTemp + j, xTemp);
        if(c[0] < 254)
        return false;
      }
      if(((xTemp + 1 >= 0) && (xTemp + 1 < map.cols)) && ((yTemp  + j>= 0) && (yTemp  + j< map.rows))) {
        cv::Scalar c = map.at<uchar>(yTemp + j, xTemp+1);
        if(c[0] < 254)
        return false;
      }
    }  
  }
  if(mode == 1 || mode == 2) {
    for (int i = 0; i < 2*R ; ++i) {
      if(((xTemp + i >= 0) && (xTemp + i < map.cols)) && ((yTemp + 2*R - 1 >= 0) && (yTemp + 2*R - 1< map.rows))) {
        cv::Scalar c = map.at<uchar>(yTemp + 2*R - 1, xTemp + i);
        if(c[0] < 254)
        return false;
      }
      if(((xTemp + i >= 0) && (xTemp + i < map.cols)) && ((yTemp + 2*R - 2>= 0) && (yTemp + 2*R - 2< map.rows))) {
        cv::Scalar c = map.at<uchar>(yTemp + 2*R - 2, xTemp+i);
        if(c[0] < 254)
        return false;
      }
    }
  }
  if(mode == 2 || mode == 3) {
    for (int j = 0; j < 2*R ; ++j) {
      if(((xTemp + 2*R - 1 >= 0) && (xTemp + 2*R - 1< map.cols)) && ((yTemp  + j >= 0) && (yTemp  + j< map.rows))) {
        cv::Scalar c = map.at<uchar>(yTemp + j, xTemp + 2*R - 1);
        if(c[0] < 254)
        return false;
      }
      if(((xTemp + 2*R - 2 >= 0) && (xTemp + 2*R - 2 < map.cols)) && ((yTemp  + j>= 0) && (yTemp  + j< map.rows))) {
        cv::Scalar c = map.at<uchar>(yTemp + j, xTemp+2*R - 2);
        if(c[0] < 254)
        return false;
      }
    }
  }
  if(mode == 3 || mode == 4) {
    for (int i = 0; i < 2*R ; ++i) {
      if(((xTemp + i >= 0) && (xTemp + i < map.cols)) && ((yTemp  >= 0) && (yTemp < map.rows))) {
        cv::Scalar c = map.at<uchar>(yTemp , xTemp + i);
        if(c[0] < 254)
        return false;
      }
      if(((xTemp + i >= 0) && (xTemp + i < map.cols)) && ((yTemp + 1>= 0) && (yTemp + 1< map.rows))) {
        cv::Scalar c = map.at<uchar>(yTemp + 1, xTemp+i);
        return false;
      }
    }
  }
 }
 return true;
}


bool _collisionWithObject(Node* qNew, Node* qNear, Mat &map) {
  int delta = 0, mode = 0;
  Node u;
  u.x = -qNear->x + qNew->x;
  u.y = -qNear->y + qNew->y;
  float normU = norme(qNew,qNear);
  Node n1;
  n1.x = qNear->x + u.x*(delta/normU);
  n1.y = qNear->y + u.y*(delta/normU);
  if (!pixelTest(n1, map, mode))
   return false;
  if(qNear->x + u.x*((delta+1)/normU) - n1.x > 0) {
    if(qNear->y + u.y*((delta+1)/normU) - n1.y < 0)
      mode = 1;
    else 
      mode = 2;
  } else {
    if(qNear->y + u.y*((delta+1)/normU) - n1.y < 0)
      mode = 4;
    else 
      mode = 3;
  }

  while(norme(&n1,qNear)<= normU) {
     ++delta;
     n1.x = qNear->x + u.x*(delta/normU);
     n1.y = qNear->y + u.y*(delta/normU);
     if(pixelTest(n1,map,mode) == false)
      return false;
  }
  return true;
}

bool isInMap(Node* n, Mat m) {
    return (((n->x >= 0) && (n->x < m.cols)) && ((n->y >= 0) && (n->y < m.rows)));
}

bool notInFreeSpace(Node* n_rand, Mat map) {
  if(!isInMap(n_rand,map))
    return false;
  cv::Scalar c = map.at<uchar>(n_rand->y,n_rand->x);
  if (c[0] >= 254) return false;
    else return true;
}


void _rrt(Node *tree, int k, Mat map) {
  std::srand(std::time(0));
  while(nb_in_tree < k) {
    if(nb_in_tree%100 == 0 && deltaQ >= 5) {
      deltaQ = deltaQ -1;
    }
    int x_rand = std::rand()%map.cols, y_rand = std::rand()%map.rows;
    Node* q_rand = new Node(x_rand,y_rand);
    extend(q_rand, tree, map);
  }
}

void extend(Node* q_rand, Node* tree, Mat map){
 Node *q_near = closestTo(q_rand, tree);
 double norm = norme(q_rand,q_near);
 Node *q_new; 
 if(!norm)
    return;
 q_new = new Node(q_near->x + (deltaQ/norm)*(q_rand->x - q_near->x), q_near->y + (deltaQ/norm)*(q_rand->y - q_near->y));
 if(notInFreeSpace(q_new,map))
    return;
 if(!_collisionWithObject(q_new, q_near, map))
    return; 
 q_new->distFromRoot = norme(q_new,q_near);
 q_near->forest.push_back(q_new);
 q_new->parent = q_near;
 ++nb_in_tree;
}

#endif
