#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <algorithm>
#include <math.h>
#include <cstring>
#include <tld_msgs/BoundingBox.h>
#include <std_msgs/String.h>
#include <kobuki_msgs/BumperEvent.h>
/*视频显示框的中心点*/
#define VIDEO_CENTER_X 320
#define VIDEO_CENTER_Y 240
/*中心点横纵坐标允许的误差偏移*/
#define ERROR_OFFSET_X 2
#define ERROR_OFFSET_Y 2
/*BoundingBox的面积允许的误差偏移*/
#define AREA_ERROR_OFFSET_RATIO 0.2
/*turtleBot运动线速度角速度默认值*/
#define CONTROL_SPEED 0.1 
#define CONTROL_TURN 0.1
/*线速度控制和与初始面积差的比例*/
#define CONTROL_SPEED_RATIO 0.0003
/*角速度控制和偏移距离的比例*/ 
#define CONTROL_TURN_RATIO 0.1
/*线速度和角速度最大值*/
//#define CONTROL_SPEED_MAX 0.4
#define CONTROL_SPEED_MAX 0.3
#define CONTROL_TURN_MAX 0.5
/*消息计数的最大值*/
#define COUNT_MAX 1000000

void transform_callback(const tld_msgs::BoundingBoxConstPtr & tracked_object);
void bumperEventCB(const kobuki_msgs::BumperEventConstPtr & msg);

struct bounding_box_info{
	int centerX;
	int centerY;
        int area;
        }BB_info;//boundingBox的信息
double control_speed = 0;//turtlebot线速度控制
double control_turn = 0;//turtlebot角速度控制
int initialBBArea = 0;//第一帧boundingBox的面积
double error_offset_area = 0;
int count = 0;//消息发布计数  
/*为bumper反馈的状态设置标志位*/
bool bumper_left_pressed_ = false;
bool bumper_center_pressed_ = false;
bool bumper_right_pressed_ = false;
bool change_direction_ = false;
bool enable_obs_avoid_ = false;

ros::Publisher pub;//声明一个全局的pub对象

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tld_turtlebot_follower");  
    ros::NodeHandle m;  
    pub = m.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);  
    ros::Subscriber bumper_event_sub = m.subscribe("/mobile_base/events/bumper", 1000, bumperEventCB);
    ros::Subscriber tld_tracked_object_sub = m.subscribe("tld_tracked_object", 1000, transform_callback);
    ros::spin();
    return 0;
}

/************************************************************************
 *ROS_openTLD反馈的跟踪目标大小和位置信息与turtlebot的速度控制的转换；
 *如果发生碰撞，那么首先处理碰撞事件，碰撞信息与turtlebot控制信息的转换
 ***********************************************************************/
void transform_callback(const tld_msgs::BoundingBoxConstPtr & tracked_object)
{       
    ROS_INFO("transform_callback start! ");     
    ROS_INFO("bool enable_obs_avoid_ in function transform_callback(): %d", enable_obs_avoid_);
    ros::Rate loop_rate(10);//loop_rate(10)可以控制while(ros:ok()的循环频率
    if (!enable_obs_avoid_)
    {
        ROS_INFO("tracking control program start!");
        int x = tracked_object -> x;
        int y = tracked_object -> y;
        int width = tracked_object -> width;
        int height = tracked_object -> height; 
        ROS_INFO("x,y,width,height:%d %d %d %d",x,y,width,height);     
        ROS_INFO("turtlebot tracking control message count: %d",count);
        if(count == 0)  
        {
            initialBBArea = width * height;//获取第一帧中boundingBox面积并作为保距跟踪参考值
            error_offset_area = AREA_ERROR_OFFSET_RATIO * initialBBArea;
            ROS_INFO("initial bounding bos area: %d", initialBBArea); 
            count++;
        }
        else
        {
            BB_info.centerX = x+width/2;
            BB_info.centerY = y-height/2;//由于turtleBot不能在三维平面移动，centerY在这里实际上没有起作用
            BB_info.area = width * height; 
            if(BB_info.area == 1)//如果跟踪不到目标（BB_info.area == 1）,角速度和线速度置0。 
            {
                control_turn = 0;
                control_speed = 0;  
                count++;
            }
            else
            {
                ROS_INFO("BB_info.area,initialBBArea:%d %d",BB_info.area,initialBBArea);  
                /*控制turtleBot角速度模块*/
                if ((BB_info.centerX > (VIDEO_CENTER_X - ERROR_OFFSET_X) ) 
                    && (BB_info.centerX < (VIDEO_CENTER_X + ERROR_OFFSET_X)))
                    control_turn = 0;
           	else if ((BB_info.centerX < (VIDEO_CENTER_X - ERROR_OFFSET_X)) 
           	        || (BB_info.centerX == (VIDEO_CENTER_X - ERROR_OFFSET_X)))
               	    control_turn = std::min((CONTROL_TURN * CONTROL_TURN_RATIO * (VIDEO_CENTER_X - ERROR_OFFSET_X - BB_info.centerX)),CONTROL_TURN_MAX);
           	else if ((BB_info.centerX > (VIDEO_CENTER_X + ERROR_OFFSET_X)) 
           	        || (BB_info.centerX == (VIDEO_CENTER_X + ERROR_OFFSET_X)))
              	    control_turn = (-1)*(std::min((CONTROL_TURN * CONTROL_TURN_RATIO * (BB_info.centerX - (VIDEO_CENTER_X + ERROR_OFFSET_X))),CONTROL_TURN_MAX));
           	else
              	control_turn = 0;
           	
           	/*控制turtleBot线速度模块*/
           	if ((BB_info.area > (initialBBArea - error_offset_area) ) && (BB_info.area < (initialBBArea + error_offset_area)))
              	    control_speed = 0;
           	else if ((BB_info.area > (initialBBArea + error_offset_area)) || (BB_info.area == (initialBBArea + error_offset_area)))
               	    control_speed = (-1)*std::min(CONTROL_SPEED*CONTROL_SPEED_RATIO*(BB_info.area- (error_offset_area + initialBBArea)),CONTROL_SPEED_MAX);
           	else if ((BB_info.area < (initialBBArea - error_offset_area)) || (BB_info.area == (initialBBArea - error_offset_area)))
               	    control_speed = std::min(CONTROL_SPEED*CONTROL_SPEED_RATIO*(initialBBArea - error_offset_area - BB_info.area),CONTROL_SPEED_MAX);
           	else
               	    control_speed = 0;     
           	ROS_INFO("control_speed, control_turn:%lf %lf",control_speed,control_turn) ;    
           	count++;
            }           
            ROS_INFO("transformat successs!") ;
            /*发布turtleBot控制消息*/
            geometry_msgs::Twist twist;  
     	    twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn; 
            pub.publish(twist);
            //ros::spinOnce();
            ROS_INFO("Publish turtlebot control message sucess!");  
        }
    } 
    /*如果enable_obs_avoid被置位，那么启动避障模块obs_avoid*/
    else
    {   
        ROS_INFO("Obstacle avoiding program start!");        
	geometry_msgs::Twist cmd2;
	int cmd_count = 0;
        if(bumper_left_pressed_)
        {
       	    ROS_INFO("Bumper_left_ is pressing!");
       	    while(ros::ok()&&change_direction_)
       	    {
       	        cmd_count ++;
       	        ROS_INFO("I am changing!");
       	 	cmd2.angular.z = -0.4;
       	 	cmd2.linear.x = -0.2;
       	 	pub.publish(cmd2);
       	 	loop_rate.sleep();
       	 	if (cmd_count > 15)
       	 	{
       	 	    change_direction_ = false;
       	 	    cmd_count = 0;
       	 	}
		//break ;
       	    }
        	
        }
        else if(bumper_center_pressed_)
        {
            while(ros::ok()&&change_direction_)
            {
       	        cmd_count ++;
       	        ROS_INFO("I am changing!");
       	        cmd2.angular.z = -0.5;
       	        cmd2.linear.x = -0.2 ;
       	        pub.publish(cmd2);
       	        loop_rate.sleep();
       	        if (cmd_count > 20)
       	        {
       	            change_direction_ = false;
       	 	    cmd_count = 0;
       	        }
		//break;
       	    }
        }
        else if(bumper_right_pressed_)
        {
            while(ros::ok()&&change_direction_)
            {
       	        cmd_count ++;
       	 	ROS_INFO("I am changing!");
       	 	cmd2.angular.z = 0.4;
       	 	cmd2.linear.x = -0.2;
       	 	pub.publish(cmd2);
       	 	loop_rate.sleep();
       	 	if (cmd_count > 15)
       	 	{
       	 	    change_direction_ = false;
       	 	    cmd_count = 0;
       	 	}
		//break;
       	    }
        }
	enable_obs_avoid_ = false;//避障程序结束之后，把enable_obs_avoid之后重新置为false.
    }
    //if(count == COUNT_MAX) count = 1;//count大于一定值，可能会报溢出，每发布100000条消息，把count重置1 
}

/************************************************************************************
碰撞事件的回调函数，当碰撞发生时，检测左侧碰撞，中心碰撞， 还是右侧碰撞，
分别置不同的flag位：bumper_left_pressed_，bumper_center_pressed_，bumper_right_pressed_
*************************************************************************************/
void bumperEventCB(const kobuki_msgs::BumperEventConstPtr & msg)
{
    ROS_INFO("bumperEventCB() start!");
    ROS_INFO("bool enable_obstacle_avoid_ in function bumperEventCB(): %d", enable_obs_avoid_);
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
        enable_obs_avoid_ = true;//如果检测到碰撞，把避障使能位置为true
  	    switch (msg->bumper)
  	    {
  	    case kobuki_msgs::BumperEvent::LEFT:
          	if (!bumper_left_pressed_)
          	{
            	    bumper_left_pressed_ = true;
            	    ROS_INFO("bumper_left_pressed_,bumperEventCB") ;
            	    change_direction_ = true;
          	}
          	break;
            case kobuki_msgs::BumperEvent::CENTER:
                if (!bumper_center_pressed_)
                {
            	    bumper_center_pressed_ = true;
            	    ROS_INFO("bumper_center_pressed_,bumperEventCB") ;
            	    change_direction_ = true;
          	}
          	break;
            case kobuki_msgs::BumperEvent::RIGHT:
                if (!bumper_right_pressed_)
                {
            	    bumper_right_pressed_ = true;
            	    change_direction_ = true;
            	    ROS_INFO("bumper_right_pressed_") ;
          	}
          	break;
      	    }
    }
    else
    {
        switch (msg->bumper)
        {
            case kobuki_msgs::BumperEvent::LEFT:    
                bumper_left_pressed_ = false;
        	break;
            case kobuki_msgs::BumperEvent::CENTER: 
        	bumper_center_pressed_ = false;
        	break;
            case kobuki_msgs::BumperEvent::RIGHT:
        	bumper_right_pressed_ = false;
        	break;
        }
    }
    ROS_INFO("bumperEventCB end!");
}

