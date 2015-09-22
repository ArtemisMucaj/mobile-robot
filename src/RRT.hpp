#ifndef RRT_HPP
#define RTT_HPP

#include <cstdlib>
#include <iostream>
#include <ctime>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Node.hpp"

#define LARGEUR_ROBOT 100
#define LONGUEUR_ROBOT 200

using namespace cv;

static int deltaQ = 40;
static int nb_in_tree = 0;


Node* closest_to_rec(Node* q_rand, Node* q_i){
  int n = q_i->forest.size(), i;
  double d, d_temp;
  Node *q_temp, *q_res;
  
  // we say that q_i is the nearest neighbour as we have not checked its forest yet
  q_res = q_i;
  
  // distance between q_rand and q_i
  d = sqrt((q_i->x - q_rand->x)*(q_i->x - q_rand->x) + (q_i->y - q_rand->y)*(q_i->y - q_rand->y));
  
  for(i=0;i<n;i++){
    //std::cout << i << std::endl;
    // recursive call on q_i's forest
    q_temp = closest_to_rec(q_rand, q_i->forest[i]);
    //std::cout << "(" << q_temp->x << "," << q_temp->y << ")" << std::endl;
    //std::cout << "(" << q_res->x << "," << q_res->y << ")" << std::endl << std::endl;
 
    // distance between q_rand and q_i->forest[i] node
    d_temp = sqrt((q_temp->x - q_rand->x)*(q_temp->x - q_rand->x) + (q_temp->y - q_rand->y)*(q_temp->y - q_rand->y));
    
    // checking if node q_i->forest[i] node is nearest neighbour and updating if so
    q_res = (d_temp < d) ? q_temp : q_res; 
    
    // updating nearest neighbour's distance found
    d = (d_temp < d) ? d_temp : d; 
		
  }
  
  return q_res; 
}

Node* closest_to(Node* q_rand, Node *tree){
  // Root is first assumed as the nearest neighbour	
  return closest_to_rec(q_rand,tree);
}

void extend(Node* q_rand, Node* tree, Mat map);

double norme(Node* n_rand, Node* n_near){
 return sqrt(pow(n_rand->x - n_near->x,2) + pow(n_rand->y - n_near->y,2));
}

bool pixel_test(Node& u, Mat &map, int mode){
 int R=40;
 int xTemp = u.x - R;
 int yTemp = u.y - R;

 if(mode == 0){
 // Parcours tout le carré 
 for(int i=0;i<2*R;++i)
 {	
  	for(int j=0;j<2*R;++j)
  	{
  		//if( pow(xTemp - u.x + i,2) + pow(yTemp - u.y + j,2) <= pow(R,2) )
  		{
        if(((xTemp + i >= 0) && (xTemp + i < map.cols)) && ((yTemp  + j>= 0) && (yTemp  + j< map.rows))){
  		    cv::Scalar c = map.at<uchar>(yTemp + j, xTemp+i);
  		    //std::cout << c[0] << std::endl; 
  		    if(c[0] < 254)
  		      return false;
        }
  		  
  		}
  	}
   }
 }else{   
    // Parours optimisé
    if(mode == 1 || mode == 4){
    // Parcours des deux premières lignes...
    for (int j = 0; j < 2*R ; ++j)
    {
      if(((xTemp >= 0) && (xTemp < map.cols)) && ((yTemp  + j >= 0) && (yTemp  + j< map.rows))){
        cv::Scalar c = map.at<uchar>(yTemp + j, xTemp);
        //std::cout << c[0] << std::endl; 
        if(c[0] < 254)
        return false;
      }
      if(((xTemp + 1 >= 0) && (xTemp + 1 < map.cols)) && ((yTemp  + j>= 0) && (yTemp  + j< map.rows))){
        cv::Scalar c = map.at<uchar>(yTemp + j, xTemp+1);
        //std::cout << c[0] << std::endl; 
        if(c[0] < 254)
        return false;
      }
    }  
  }
  if(mode == 1 || mode == 2){
    // Parcours des deux dernières colonnes ...
    for (int i = 0; i < 2*R ; ++i)
    {
      if(((xTemp + i >= 0) && (xTemp + i < map.cols)) && ((yTemp + 2*R - 1 >= 0) && (yTemp + 2*R - 1< map.rows))){
        cv::Scalar c = map.at<uchar>(yTemp + 2*R - 1, xTemp + i);
        //std::cout << c[0] << std::endl; 
        if(c[0] < 254)
        return false;
      }
      if(((xTemp + i >= 0) && (xTemp + i < map.cols)) && ((yTemp + 2*R - 2>= 0) && (yTemp + 2*R - 2< map.rows))){
        cv::Scalar c = map.at<uchar>(yTemp + 2*R - 2, xTemp+i);
        //std::cout << c[0] << std::endl; 
        if(c[0] < 254)
        return false;
      }
    }
  }

    if(mode == 2 || mode == 3){
      //Parcours des deux dernières lignes...
    for (int j = 0; j < 2*R ; ++j)
    {
      if(((xTemp + 2*R - 1 >= 0) && (xTemp + 2*R - 1< map.cols)) && ((yTemp  + j >= 0) && (yTemp  + j< map.rows))){
        cv::Scalar c = map.at<uchar>(yTemp + j, xTemp + 2*R - 1);
        //std::cout << c[0] << std::endl; 
        if(c[0] < 254)
        return false;
      }
      if(((xTemp + 2*R - 2 >= 0) && (xTemp + 2*R - 2 < map.cols)) && ((yTemp  + j>= 0) && (yTemp  + j< map.rows))){
        cv::Scalar c = map.at<uchar>(yTemp + j, xTemp+2*R - 2);
        //std::cout << c[0] << std::endl; 
        if(c[0] < 254)
        return false;
      }
    }
  }
  if(mode == 3 || mode == 4){
    // Parcours des deux premières colonnes ...
    for (int i = 0; i < 2*R ; ++i)
    {
      if(((xTemp + i >= 0) && (xTemp + i < map.cols)) && ((yTemp  >= 0) && (yTemp < map.rows))){
        cv::Scalar c = map.at<uchar>(yTemp , xTemp + i);
        //std::cout << c[0] << std::endl; 
        if(c[0] < 254)
        return false;
      }
      if(((xTemp + i >= 0) && (xTemp + i < map.cols)) && ((yTemp + 1>= 0) && (yTemp + 1< map.rows))){
        cv::Scalar c = map.at<uchar>(yTemp + 1, xTemp+i);
        //std::cout << c[0] << std::endl; 
        if(c[0] < 254)
        return false;
      }
    }
  }

 }
 
 return true;
}


bool _collision_with_object(Node* qNew, Node* qNear, Mat &map){
  int delta = 0, mode = 0;
  Node u;
  u.x = -qNear->x + qNew->x;
  u.y = -qNear->y + qNew->y;
  float normU = norme(qNew,qNear);
  Node n1;
  n1.x = qNear->x + u.x*(delta/normU);
  n1.y = qNear->y + u.y*(delta/normU);
  //std::cout << normU << std::endl;
  if (!pixel_test(n1, map, mode))
   return false;
  // Choix du mode de collision selon la direction voulue
  if(qNear->x + u.x*((delta+1)/normU) - n1.x > 0){
    if(qNear->y + u.y*((delta+1)/normU) - n1.y < 0)
      mode = 1;
    else 
      mode = 2;
  }else{
    if(qNear->y + u.y*((delta+1)/normU) - n1.y < 0)
      mode = 4;
    else 
      mode = 3;
  }

  while(norme(&n1,qNear)<= normU){
     ++delta;
     n1.x = qNear->x + u.x*(delta/normU);
     n1.y = qNear->y + u.y*(delta/normU);
     if(pixel_test(n1,map,mode) == false)
      return false;
    }
  return true;
}

bool is_in_map(Node* n, Mat m){
    return (((n->x >= 0) && (n->x < m.cols)) && ((n->y >= 0) && (n->y < m.rows)));
}

bool not_in_free_space(Node* n_rand, Mat map){
  if(!is_in_map(n_rand,map))
    return false;
  cv::Scalar c = map.at<uchar>(n_rand->y,n_rand->x);
  //std::cout << c[0] << std::endl;
  if (c[0] >= 254) return false;
    else return true;
}


void _rrt(Node *tree, int k, Mat map){
  
  std::srand(std::time(0));
  while(nb_in_tree < k){
  //for(int i = 0; i < k; i++){
    if(nb_in_tree%100==0 && deltaQ >= 5){
      deltaQ = deltaQ -1;
    }
    //std::cout << "Node " << i << std::endl;
    int x_rand = std::rand()%map.cols, y_rand = std::rand()%map.rows; // configure with map size
    //std::cout << " x_rand : " << x_rand << ", y_rand : " << y_rand << std::endl;
    Node* q_rand = new Node(x_rand,y_rand);
    // add distance from root 
    extend(q_rand, tree, map);
    //std::cout << "Elements in tree : " << nb_in_tree << std::endl;
  }
}

void extend(Node* q_rand, Node* tree, Mat map){
 // Find closest to q_rand in tree 
 Node *q_near = closest_to(q_rand, tree);
 //std::cout << "allô (" << q_rand->x << "," << q_rand->y << ")" << std::endl;
 //std::cout << "qnear (" << q_near->x << "," << q_near->y << ")" << std::endl;
 //sleep(1);
 double norm = norme(q_rand,q_near);
 //std::cout << norm << std::endl;
 Node *q_new; 
 if(!norm)
    return;
 q_new = new Node(q_near->x + (deltaQ/norm)*(q_rand->x - q_near->x), q_near->y + (deltaQ/norm)*(q_rand->y - q_near->y));
 if(not_in_free_space(q_new,map))
    return;
  //std::cout << " Point is in free space ... " << std::endl;
 // Checker collision sur la ligne q_near -> q_new
 if(!_collision_with_object(q_new, q_near, map))
    return; 
  //std::cout << "( " <<q_new->x << ", " << q_new->y << " )" << std::endl;
  //std::cout<< "No collision detected ..." << std::endl;
 // Pas de collision donc ajout dans l'arbre
 q_new->distFromRoot = norme(q_new,q_near);
 // Ajouter q_new a la foret de q_near, q_near est le parent de q_new
 q_near->forest.push_back(q_new);
 q_new->parent = q_near;
 //std::cout<< "Added to tree ..." << std::endl;
 ++nb_in_tree;
}

#endif