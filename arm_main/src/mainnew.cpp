// 2020 by YuanHaobo  <yhb0521@126.com>

#include <ros/ros.h>
#include <string>            //maybe it can be deleted
#include "std_msgs/String.h" //maybe it can be deleted
#include <swiftpro/position.h>
#include <swiftpro/status.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Point.h>
#include <arm_main/Solve.h>
#include "arm_main/coordinate.h"  // data from camera
#include <cmath>

#define RATE 50

using namespace std;


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!     2020.11.22 决赛素材使用，适用于固定场景位置     !!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



// variable declaration

bool objmark = 0, posmark = 0, nummark = 0;
std_msgs::UInt16 servo_state;
swiftpro::status gripper_state;   // 不能用0直接赋值？因为他是类，不是类成员
swiftpro::position aim_position;
swiftpro::position store_position;
swiftpro::position init_position;
swiftpro::position store_position_array[3];
// geometry_msgs::Point solve_result;
int solve_result[3] = {180, 0, 90};
arm_main::coordinate objdata;

int store_height_position[9];
bool serial_return_mark=0;        // 用于判断何时运动完成的标识，1表示运动完成，可以退出waitawhile



// funtion declaration

void waitawhile();
void ObjDataCallback(const arm_main::coordinate &objdata_);
void SerialReturnCallback(const swiftpro::status& msg);

class Solve;  // need to be declare, although it has been defined in Solve.h
              // 在下面先实例化为mysolve，然后调用mysolve.solve()


int main(int argc, char** argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh("~");
    ros::Subscriber obj_data_sub      = nh.subscribe("/coordinate_topic",  50, ObjDataCallback);
    // ros::Subscriber pos_data_sub   = nh.subscribe("/swiftpro/pos_data", 50, PosDataCallback);
    ros::Subscriber serial_return_sub = nh.subscribe("/serial_return_topic",1, SerialReturnCallback);

    ros::Publisher motor_pub   = nh.advertise<swiftpro::position>("/position_write_topic",50);
    ros::Publisher servo_pub   = nh.advertise<std_msgs::UInt16>("/servo",50);
    ros::Publisher gripper_pub = nh.advertise<swiftpro::status>("/gripper_topic",50);

    ros::Rate loop_rate(RATE);
    ROS_INFO("Start main");

    // 初始位置
    init_position.x = 95;   init_position.y = -58;   init_position.z = 50; init_position.speed = " F200";
    // 放置区位置，后期需要修改
    store_position.x = 43.50;  store_position.y = 164.82; store_position.z = 75;store_position.speed = " F100";
    store_position_array[0] =  store_position;
    store_position.x = 84.83;  store_position.y = 167.65; store_position.z = 145;store_position.speed = " F100";
    store_position_array[1] = store_position;
    store_position.x = 154.60; store_position.y = 184.69; store_position.z = 90;store_position.speed = " F100";
    store_position_array[2] = store_position;
    // 放置区每个物块需要的高度
    int store_height_position[9] = {52,80,115,55,80,105,60,95,135};
    int height_bias = -43;      // 用于整体调整高度偏差，方便调整机械臂高度后，整体改参数
    for (int i=0;i<9;i++){
        store_height_position[i] += height_bias;
    }

    Solve mysolve;

    gripper_state.status = 0;//0 open; 1 close;
    gripper_pub.publish(gripper_state);
    servo_state.data = 90;
    servo_pub.publish(servo_state);
    motor_pub.publish(init_position);//初始位置
    waitawhile();


    while(ros::ok())
    {
        ros::spinOnce();
        // nummark = sizeof(objdata.segments)/sizeof(objdata.segments[0]);
        if (objmark == 1)
        {   //initial 5 DOF
            // gripper_state.status = 0;//0 open; 1 close;
            // gripper_pub.publish(gripper_state);
            // servo_state.data = 90;
            // servo_pub.publish(servo_state);
            // motor_pub.publish(init_position);//初始位置
            // waitawhile();

            int solve_result_array[9][3]={
                {224, 44,  130}, {202, 109, 105}, {225, 81, 105},
                {279, 114, 145}, {138, 18,  179}, {199, -15, 80},
                {310, 55,  90},  {273, -1,   90}, {155, 45,  90}
            };

            for (int ii = 0; ii<9; ii++)
            {               
                
                // calculate the coordinate
                cout << endl;cout << endl;
                // solve_result = mysolve.solve(objdata.ObjDataArray[ii]);
                solve_result[0] = solve_result_array[ii][0];
                solve_result[1] = solve_result_array[ii][1];
                solve_result[2] = solve_result_array[ii][2];


                cout << "\n____________________________\n____________________________\n\n No." 
                  << ii << ":\n camera sent coordinate: \n" 
                //   << objdata.ObjDataArray[ii]
                //   << "\n solve result: " << solve_result.x << " " << solve_result.y << " " << solve_result.z 
                  << "\n____________________________\n____________________________\n\n" << endl;

                aim_position.x = solve_result[0];
                aim_position.y = solve_result[1];
                aim_position.z = 100;
                aim_position.z += height_bias;
                aim_position.speed = " F150";
                servo_state.data = solve_result[2];

                if (ii ==6){
                    aim_position.speed = " F100";
                }

                if(aim_position.x<20){
                    cout << "the aim_position x_value is too low, coordinate fault, pass this turn" << endl;
                    continue;
                }


                // Start to move
                cout << ii << ".1 Start to move.\n" << endl;
                gripper_state.status = 0;//0 open; 1 close;
                servo_pub.publish(servo_state);
                gripper_pub.publish(gripper_state);
                waitawhile();sleep(1.5);
                motor_pub.publish(aim_position);//分拣区上
                waitawhile();sleep(0.3);

                if (ii ==2){
                    sleep(1);
                }

                aim_position.z = 45;
                aim_position.z += height_bias;
                aim_position.speed = " F80";
                motor_pub.publish(aim_position);//分拣区 move down
                waitawhile();sleep(0.5);

                gripper_state.status = 1;
                gripper_pub.publish(gripper_state);sleep(1);//wait gripper close
                waitawhile();sleep(0.5);
                cout << ii << ".2 Catch successfully.\n" <<endl;

                // return 0;//just for test & debug

                aim_position.z = 100;
                aim_position.z += height_bias;
                aim_position.speed = " F100";              
                motor_pub.publish(aim_position);//分拣区上 move up
                waitawhile();

                cout << ii << ".3 Pick up successfully. Moving to warehouse.\n" <<endl;
                store_position = store_position_array[ii/3];
                store_position.z = 135;
                aim_position.z += height_bias;
                store_position.speed = " F150";
                if (ii ==6){
                    store_position.speed = " F100";
                }  
                servo_state.data = store_position_array[ii/3].z;
                servo_pub.publish(servo_state);
                motor_pub.publish(store_position);//仓库区上
                waitawhile();

                store_position.z = store_height_position[ii];
                cout << ii << ".4 Moving downawrd in warehouse. Warehouse's height is "
                  << store_height_position[ii] << endl << endl;

                store_position.speed = " F100";
                motor_pub.publish(store_position);//仓库区 move down
                waitawhile();

                gripper_state.status = 0;sleep(0.3);
                gripper_pub.publish(gripper_state);sleep(1.7);;//wait gripper open
                waitawhile();sleep(1);//wait gripper open
                cout << ii << ".5 Lay down successfully. Moving upward in warehouse\n" <<endl;
                sleep(0.3);

                if(ii==8){
                    store_position.z = store_position.z + 23;
                    cout<<"\nii==8 \n"<<endl;
                }
                else{
                    store_position.z = store_position.z + 40;
                }
                store_position.speed = " F200";
                motor_pub.publish(store_position);//仓库区上
                waitawhile();
                sleep(0.5);
                cout << ii << ".6 Moving to sorting area.\n" <<endl;
            }

            cout << "Moving back sorting area.\n" <<endl;
            servo_state.data = 90;
            servo_pub.publish(servo_state);
            motor_pub.publish(init_position);//归位位置
            waitawhile();

            objmark = 0;
            cout<<"\nSorting has been done. The main_node finish. \n\n\n!!! Goodbye !!!\n\n\n"<<endl;
            ros::shutdown();

        }else{
            cout<< "objmark != 1, please wait for coordinate_publisher."<<endl;
        }
        loop_rate.sleep();
    }
    return 0;
}


void ObjDataCallback(const arm_main::coordinate &objdata_)
{
    objdata = objdata_;
    objmark = 1;
}

void SerialReturnCallback(const swiftpro::status& msg)
{
    if (msg.status == 1)
		serial_return_mark = 1;
	else if (msg.status == 0)
		serial_return_mark = 0;
	else
	{
		ROS_INFO("Error:Wrong swiftpro status input");
		serial_return_mark = 0;
	}


}


void waitawhile()
{
    while(1)
    {   
        ros::spinOnce();
        if(serial_return_mark==1)
        {
            serial_return_mark=0;
            break;
        }
        sleep(0.3);//0.2
        
    }
}
