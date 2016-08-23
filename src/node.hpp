#ifndef NODE_HPP
#define NODE_HPP

#include <iostream>
#include <vector>
#include <ros/ros.h>

struct Node{
 int x;
 int y;
 double distFromRoot;
 double distFromGoal;
 Node* parent;
 std::vector<Node*> forest; 

 Node(): x(0),y(0), distFromRoot(0), distFromGoal(0){
   //forest = new std::vector<Node*>();
  parent = NULL;
 }
 Node(int x0, int y0): x(x0), y(y0), distFromRoot(0), distFromGoal(0){
   //forest = new std::vector<Node*>();
  parent = NULL;
 }
 
 
 /*Node* operator+ (Node const& a){
  return new Node(a->x+x, a->y+y);  
 }  
 Node* operator* (float a){
  return new Node (a*x, a*y);
 }*/
  
};


#endif
