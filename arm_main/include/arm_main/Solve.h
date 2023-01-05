#ifndef armSolve
#define armSolve
// the name above must be different with .h file's name "Solve"
#include <geometry_msgs/Point.h>
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <cmath>

using namespace std;
class Solve // Capital S, same name as .h file
{
public:
  // local const should be declared in .h file

  // ratio, unit is pixel/mm
  float x_ratio = 1.9;// calculate value is 1.322264
  float y_ratio = 1.9;// calculate value is 1.319

  // centre of the circle, data from camera,unit is pixel
  // 1280*720
  // float x_cam_centre = 629.0;
  // float y_cam_centre = 326.0;
  // 640*480
  float x_cam_centre = 320.0;
  float y_cam_centre = 240.0;

  float distance;
  float radius=35;

  // centre of the circle, data from uARM Swiftpro
  float x_uarm_centre = 226.90;
  float y_uarm_centre = 41.89;

  // local varibles should be declared in .h file
  float x_cam, y_cam, theta_cam;
  float x_middle, y_middle;
  float x_uarm, y_uarm, theta_uarm, theta_servo;

  int x_uarm_int, y_uarm_int, theta_servo_int;

  geometry_msgs::Point Point;
  // declare the local varible "Point", which will be return in .cpp
  geometry_msgs::Point solve(geometry_msgs::Point Point_);
  // the "solve" func is a "geometry_msgs::Point" type func and belong to class "Solve"
  // parameter "Point_" is "geometry_msgs::Point" type
};

#endif