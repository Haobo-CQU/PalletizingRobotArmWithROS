#include "Solve.h"
#include <iostream>

using namespace std;

#define pi 3.14159265

geometry_msgs::Point Solve::solve(geometry_msgs::Point Point_) {
//the "solve" func is a "geometry_msgs::Point"" type func and belong to class "Solve"

  //_cam: data from camera, unit is pixel or degree
  x_cam = Point_.x;
  y_cam = Point_.y;
  theta_cam = Point_.z;

  // cout << endl;
  // cout << "cam point: " << x_cam << " " << y_cam << " " << -theta_cam <<endl;

  //_middle:coordinate for centre of the circle, unit is mm or degree
  // ratio is for camera's height is 43cm
  // x_middle = (x_cam - x_cam_centre) / x_ratio;
  // y_middle = (y_cam_centre - y_cam) / y_ratio;
  
  // cout << endl;
  // cout << "middle point: " << x_middle << " " << y_middle  << " " << theta_cam <<endl;


  // //x or y_uarm: data will be sent out to uARM Swiftpro, unit is mm
  // x_uarm = x_middle + x_uarm_centre;
  // y_uarm = y_middle + y_uarm_centre;


  // x_uarm=3*pow(10,-11)*pow(x_cam,6)-5*pow(10,-8)*pow(x_cam,5)+4*pow(10,-5)*pow(x_cam,4)-0.0156*pow(x_cam,3)+3.4458*pow(x_cam,2)-393.8*x_cam+18290;
  // y_uarm=-3*pow(10,-12)*pow(y_cam,6)+4*pow(10,-9)*pow(y_cam,5)-2*pow(10,-6)*pow(y_cam,4)+0.0007*pow(y_cam,3)-0.105*pow(y_cam,2)+7.0554*y_cam-60.323;
  // x_uarm=0.6682*x_cam+11.262;
  // y_uarm=-0.439*y_cam+129.01;
  // x_uarm = 0.0003*pow(x_cam,2)+0.4513*x_cam+43.501;
  // y_uarm = 0.0005*pow(y_cam,2)-0.6432*y_cam+147.05;
  // x_uarm = 2*pow(10,-6)*pow(x_cam,3)-0.0019*pow(x_cam,2)+1.1365*x_cam-21.703;
  // y_uarm = -1*pow(10,-6)*pow(y_cam,3)+0.0013*pow(y_cam,2)-0.8046*y_cam+155.47;

  // new data with chess---------------------
  // x_uarm=0.6703*x_cam+12.568;
  // y_uarm=-0.4493*y_cam+135.93;
  // bad
  // x_uarm = 7*pow(10,-5)*pow(x_cam,2)+0.6269*x_cam+19.157;
  // y_uarm = 0.0002*pow(y_cam,2)-0.5505*y_cam+144.85;
  // very bad
  // x_uarm = 1*pow(10,-6)*pow(x_cam,3)-0.0011*pow(x_cam,2)+0.9805*x_cam-15.39;
  // y_uarm = -1*pow(10,-6)*pow(y_cam,3)+0.001*pow(y_cam,2)-0.6905*y_cam+152.52;

  // new data, start from 2020.11.27 16:00
  // x_uarm=-6*pow(10,-5)*pow(x_cam,2)+0.7241*x_cam+7.2409;
  // y_uarm=0.0003*pow(y_cam,2)-0.5774*y_cam+150.46;  
  // x_uarm = 2*pow(10,-6)*pow(x_cam,3)-0.0015*pow(x_cam,2)+1.1419*x_cam-30.794;
  // distance=sqrt( pow((x_cam-x_cam_centre),2) + pow((y_cam-y_cam_centre),2) );
  // if (distance<radius){
  //   x_uarm = 0.9997*pow(x_cam,0.9451);
  //   y_uarm = -2*pow(10,-7)*pow(y_cam,3)+0.0004*pow(y_cam,2)-0.599*y_cam+151.68;
  // }else{
  //   x_uarm = 5*pow(10,-5)*pow(x_cam,2)+0.6753*x_cam+11.965;
  //   y_uarm = 0.0001*pow(y_cam,2)-0.4838*y_cam+139.86;
  // }


  // distance=sqrt( pow((x_cam-x_cam_centre),2) + pow((y_cam-y_cam_centre),2) );
  // if (distance<radius){
  //   x_uarm = 0.9997*pow(x_cam,0.9451);
  //   y_uarm = -2*pow(10,-7)*pow(y_cam,3)+0.0004*pow(y_cam,2)-0.599*y_cam+151.68;
  // }else{
  //   x_uarm = 5*pow(10,-5)*pow(x_cam,2)+0.6727*x_cam+12.321;
  //   y_uarm = -2*pow(10,-6)*pow(y_cam,3)+0.0018*pow(y_cam,2)-0.8597*y_cam+164.07;
  // }

  // x_uarm = -4*pow(10,-9)*pow(x_cam,4)+7*pow(10,-6)-0.0038*pow(x_cam,2)+1.5582*x_cam-57.913;
  // y_uarm = 0.0003*pow(y_cam,2)-0.5774*y_cam+150.46; 

  // all data without chess board,very bad
  // x_uarm=0.0002*pow(x_cam,2)+0.5516*x_cam+32.053;
  // y_uarm=0.0004*pow(y_cam,2)+0.6267*y_cam+150.75;

//radius---30

  float f_cam_data_list[14][2] = {
    {178,208},{212,109},{195,284},{251,176},{293,66},
    {294,370},{338,144},{320,213},{324,295},{387,75},
    {429,173},{390,255},{387,356},{455,262}
    };

  float f_uarm_data_list[14][2] = {
    {123.20,34.50}, {145.01,80.21},{132.71,5.35},  {172.16,53.19},{214.50,110.00},
    {205.86,-26.43},{247.77,74.33},{229.04,38.82}, {226.88,3.83}, {288.69,113.67},
    {308.75,63.28}, {271.05,18.71},{277.04,-25.70},{324.49,14.91}
    };


  int index=100;

  for (int i = 0; i<14; i++){
    float f_data_x, f_data_y;
    f_data_x=f_cam_data_list[i][0];
    f_data_y=f_cam_data_list[i][1]; 
    distance=sqrt( pow((x_cam-f_data_x),2) + pow((y_cam-f_data_y),2) );
    cout<<"distance:"<<distance;
    if(distance < radius){
      index=i;
      cout<<"index:"<<index-1;
      break;
    }
  }
  x_uarm = f_uarm_data_list[index][0];
  y_uarm = f_uarm_data_list[index][1];




  // uarm 's theta, unit is degree
  theta_uarm = atan(y_uarm / x_uarm) / pi * 180;//alpha

  // cout << endl;
  // cout << "uarm point: " << x_uarm << " " << y_uarm << " " << theta_uarm <<endl;


  //_servo:data will be sent out to servo
  if(theta_cam < 90){
    theta_servo = theta_uarm - theta_cam + 90;
  }else{
    theta_servo = theta_uarm - theta_cam + 270;
  }

  // cout << endl;
  // cout<<"theta_servo: " << theta_servo <<endl;
  //_int: swiftpro and servo require int data
  x_uarm_int = int(x_uarm); 
  y_uarm_int = int(y_uarm); 
  theta_servo_int = int(theta_servo);


  Point.x = x_uarm_int;
  Point.y = y_uarm_int;
  Point.z = theta_servo_int;

  // cout << endl;
  // cout << "final point: " << Point.x << " " << Point.y << " " << Point.z << endl;

  return Point;
  //"Point" has been declared in .h file
}

  