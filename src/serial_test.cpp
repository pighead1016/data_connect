#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "sensor_msgs/LaserScan.h"

#include "serial_test.h"
/*#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>

#include <semaphore.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>


#include <errno.h>
#include <malloc.h>
#include <termios.h>
#include "math.h"
#include <stdbool.h>
#include <sys/time.h>*/
using namespace std;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
#define Pi 3.1415926
static int g_run=1;
static int g_fd;
static int g_packlen,callback_flag=0;
static pthread_mutex_t g_tMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_tConVar  = PTHREAD_COND_INITIALIZER;
static unsigned char serial_data[20];
static pthread_t g_pthread;
uint16_t left_rad,right_rad;
int left_speed_rad,right_speed_rad,now_left=0,now_right=0;
float ratio = 900.0f ;   //转速转换比例，执行速度调整比例
float D = 0.2680859f ;    //两轮间距，单位是m
float d_wheel=0.125f ;
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度


/****************************************************/
unsigned char speed_data[16]={
	0x55,0xa1,0xb1,
	0xc0,//故障返回
	0xd0,//
	0xe1,0xf1,//电机使能
	0xa1,0xb1,//电机方向
	0xc0,0xd0,0xe0,0xf0,//电机速度高位在前
	0xaa,
	0xe3,0x9b};   //CRC校验低位在前  要发给串口的数据
/*************************CRC校验***********************/
unsigned int CRC_ver(unsigned char *cbuffer,unsigned int ibuffer)
{
    int i,j;
    unsigned int wcrc=0xffff;
	unsigned int wpoly=0xa001;
	for(i=0;i<ibuffer;i++)
	{
		wcrc^=cbuffer[i];
		for (j=0;j<8;j++)
		{
			if(wcrc&0x0001)	wcrc=(wcrc>>1)^wpoly;
			else	wcrc=wcrc>>1;
		}
	}
	return wcrc;
}

/************************************************************
  * 函数名称:  CreatePthread
  * 功能描述:  创建线程
  * 函数参数:  1:线程数据
  * 返  回   值 :  无 
  * 修改日期:       版本号        修改人      修改内容
  *-----------------------------------------------------------
  *  2017/04/10               v001.01               
  ************************************************************/
static void *CreatePthread(void *data)
{

	unsigned char buf[42],head_data[42],end_data[42],tem_data[42];
	bzero(head_data,42);
	bzero(end_data,42);
	fd_set read_fds;
	struct timeval tm;
	int nRet,serial_head=-1,serial_end=-1,head_num=0,end_num=0,data_flag=0;

	while (g_run)
	{
		//g_fd=0;		
		FD_ZERO(&read_fds);
		FD_SET(g_fd, &read_fds);
		
		tm.tv_sec = 0;
		tm.tv_usec = 300000;
		
		nRet = select(g_fd+1, &read_fds, NULL, NULL, &tm);
		//int nReta = read(g_fd, buf, 20);
		//ROS_INFO("data %d",nRet);
		if (nRet < 0)
		{
			pthread_mutex_lock(&g_tMutex);
			pthread_cond_signal(&g_tConVar);
			//ALOGI("select error!\n");
			ROS_WARN("select error!\n");
			g_packlen = -2;
			pthread_mutex_unlock(&g_tMutex);
		}
		else if (nRet == 0)
		{
			pthread_mutex_lock(&g_tMutex);
			pthread_cond_signal(&g_tConVar);
			//ALOGI("timeout\n");
			ROS_WARN("timeout\n");
			g_packlen = 0;
			pthread_mutex_unlock(&g_tMutex);
		}
		else
		{
			if (FD_ISSET(g_fd, &read_fds))
			{
				bzero(buf, 42);
				nRet = read(g_fd, buf, 42);
				if (nRet > 0)
				{
					if(data_flag==0)//数据不完整
					{	
						memcpy(end_data+end_num,buf,nRet);
						nRet=nRet+end_num;
						bzero(tem_data,42);
						for(int i=0;i<nRet;i++)
						{
							if(end_data[i]==0x55)
							{
								serial_head=i;//得到数据头
								head_num=nRet-serial_head;
								memcpy(tem_data,end_data+serial_head,head_num);
								break;
							}
						}
						serial_head=0;
						bzero(end_data,42);
						if(head_num>=16)
						{
							if(tem_data[13]==0xaa)
							{
								memcpy(serial_data,tem_data,16);
								bzero(end_data,42);
								end_num=head_num-16;
								memcpy(end_data,tem_data+16,end_num);
								data_flag=1;
							}
							else
							{
								head_num=0;
								end_num=0;
								bzero(tem_data,42);
								ROS_ERROR("end_data error");
							}
						}
						else
						{
							memcpy(end_data,tem_data,head_num);
							end_num=head_num;
						}
					}
					
					if(data_flag==1)
					{
						data_flag=0;
						ROS_INFO("data receive");
						for(int i=0;i<=15;i++)
							printf(" %02x ",serial_data[i]);

					}
					//usleep(30000);					
					//LidarData(buf, nRet);
					
				}
				else 
				{
					pthread_mutex_lock(&g_tMutex);
					pthread_cond_signal(&g_tConVar);
			//		ALOGI("read error\n");
					ROS_INFO("read error\n");
					g_packlen = -1;
					pthread_mutex_unlock(&g_tMutex);
				}
			}
		}
	}
	return NULL;
}
int open_serial(string port,speed_t baud)
{
	g_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

	if (-1== g_fd){ /* 打开串口一*/ return -1;}
	//ROS_INFO("%d",g_fd);	
	struct  termios Opt;
	cfmakeraw(&Opt);	
	//tcgetattr(g_fd, &Opt);
	cfsetispeed(&Opt,baud);     /*设置为115200Bps*/
	cfsetospeed(&Opt,baud);
	
	//set databits
	Opt.c_cflag |= (CLOCAL | CREAD);
	Opt.c_cflag &=~CSIZE;
        Opt.c_cflag |= CS8; //8个数据位
        //check bit
        Opt.c_cflag &= ~PARENB;    // set parity disable
        Opt.c_iflag &= ~INPCK;
        //stop bit
        Opt.c_cflag &= ~CSTOPB;  //1 STOP
	Opt.c_cc[VTIME] = 0;	
	Opt.c_cc[VMIN] = 1;	
	tcflush(g_fd, TCIFLUSH);
	//tcsetattr(g_fd,TCSANOW,&Opt);
	return 0;
}


union floatData //union的作用为实现char数组和float之间的转换
{
    float d;
    unsigned char data[4];
}right_speed_data,left_speed_data,position_x,position_y,oriention,vel_linear,vel_angular;
void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
    //ROS_WARN_THROTTLE(2, "left speed =  speed=");    

    angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s

    //将转换好的小车速度分量为左右轮速度
    left_speed_data.d = linear_temp - 0.5f*angular_temp*D ;
    right_speed_data.d = linear_temp + 0.5f*angular_temp*D ;
    
    //存入数据到要发布的左右轮速度消息
    
    left_speed_rad = int(left_speed_data.d*ratio/Pi/d_wheel);
    right_speed_rad= int(right_speed_data.d*ratio/Pi/d_wheel);
    
    callback_flag=1;
    //写入数据到串口
    //ROS_INFO("left speed h= %d, l= %d,right speed h=%d, l= %d", speed_data[9],speed_data[10],speed_data[11],speed_data[12]);
    //my_serial.write(speed_data,16);
}
int main(int argv, char **argc)
{
	ros::init(argv, argc, "test_ser");
	ros::NodeHandle n;
	
	string scan_topic = "scan";
	//ros::param::get("~scan_topic", scan_topic);
	//ros::param::get("~laser_link", laser_link);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);
	ros::Subscriber sub = n.subscribe("cmd_vel", 20, callback);
	
	string port = "/dev/ttyUSB0";
    	speed_t baud = B115200;
	ros::param::get("~serial_port", port);
	int ser=open_serial(port,baud);
	if (ser<0)
	{
		ROS_ERROR("serial failed....");
		return -1;
	}
	else
		ROS_INFO("serial open,handle is %d",g_fd);	
    	
	//create read_pthread
	pthread_create(&g_pthread, NULL, CreatePthread, NULL);
	unsigned char str[]=//"this is a test!";
	{0xaa,0x01,0x01,
	0x00,//故障返回
	0x00,//
	0x01,0x01,//电机使能
	0x01,0x01,//电机方向
	0x00,0x00,0x00,0x00,//电机速度高位在前
	0x55,
	0xe3,0x9b};	//str[15]=0xaa;	
	write(g_fd,str,sizeof(str));
	static tf::TransformBroadcaster odom_broadcaster;//定义tf对象
	geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息
	nav_msgs::Odometry odom;//定义里程计对象
	geometry_msgs::Quaternion odom_quat; //四元数变量
    	//定义covariance矩阵，作用为解决文职和速度的不同测量的不确定性
	float covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
                        0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
                        0,  0,    99999, 0,     0,     0,  // covariance on gps_z
                        0,  0,    0,     99999, 0,     0,  // large covariance on rot x
                        0,  0,    0,     0,     99999, 0,  // large covariance on rot y
                        0,  0,    0,     0,     0,     0.01};  // large covariance on rot z 
	//载入covariance矩阵
	for(int i = 0; i < 36; i++)
	{
    	odom.pose.covariance[i] = covariance[i];;
	}       
	int loop_times=10;
	ros::Rate loop_rate(loop_times);//设置周期休眠时间
	float dt=1.0/loop_times;
	uint16_t left_wheel=0,right_wheel=0;
	oriention.d=0.0,position_y.d=0.0,position_y.d=0.0;	
	//ros::Rate loop_rate(5);//设置周期休眠时间
	write(g_fd,speed_data,16);
	while (ros::ok())
	{
		ros::spinOnce();		//cmd_vel
		
		//pthread_mutex_lock(&g_tMutex);
		//pthread_cond_wait(&g_tConVar, &g_tMutex);
		usleep(1000);//延时1ms	
		uint16_t temp_left,temp_right;
		ROS_INFO("left in = %d right in=%d",left_speed_rad,right_speed_rad);
		if(callback_flag==1)
		{
			//ROS_INFO("callback end");
			 /*****************pid调节******************/
			if(now_left<=left_speed_rad) 
				now_left+=100;
			else	now_left-=89;
			if(now_right<=right_speed_rad) 
				now_right+=100;
			else	now_right-=89;

			

			if(now_left>0)	speed_data[7]=0x01,temp_left=now_left;
			else	speed_data[7]=0x00,temp_left=~now_left+1;
    	if(now_right>0)	speed_data[8]=0x00,temp_right=now_right;
			else	speed_data[8]=0x01,temp_right=~now_right+1;
		 	//ROS_INFO("now_left   %d   now_right   %d",now_left,now_right);

			if(temp_left>3000)	temp_left=2990;
			if(temp_left<80)		temp_left=0;
			
			
			if(temp_right>3000)	temp_right=2990;
			if(temp_right<80)		temp_right=0;
			/*********************************************/
			callback_flag=0;
			
		}
		else
		{
			
			if(now_left>=100) 
				now_left-=97;
			else if(now_left<-100)
				now_left+=87;
			else 
				now_left=0;

			if(now_right>=100) 
				now_right-=97;
			else if(now_right<-100)
				now_right+=87;
			else
				now_right=0;
		
			if(now_left>0)	speed_data[7]=0x01,temp_left=now_left;
			else	speed_data[7]=0x00,temp_left=~now_left+1;
    	if(now_right>0)	speed_data[8]=0x00,temp_right=now_right;
			else	speed_data[8]=0x01,temp_right=~now_right+1;

			if(temp_left>3000)	temp_left=2990;
			//if(temp_left<80)		temp_left=0;
			if(temp_right>3000)	temp_right=2990;
			//if(temp_right<80)		temp_right=0;
		}
		ROS_INFO("temp left  %d   temp right   %d",temp_left,temp_right);
		/*speed_data[10] = temp_left;
    speed_data[9]  = temp_left>>8;
    speed_data[12] = temp_right;
    speed_data[11] = temp_right>>8;
    unsigned int crc=CRC_ver(speed_data,14);
    speed_data[14] = crc;
    speed_data[15] = crc>>8;*/
		write(g_fd,speed_data,16);//发送电机数据


		
		
		//pthread_mutex_unlock(&g_tMutex);
		usleep(1000);//延时1ms

		//计算左右轮速度
		float left_speed=0.0,right_speed=0.0;//清零轮速
		left_speed = left_rad/900.0*Pi*d_wheel;
		right_speed= right_rad/900.0*Pi*d_wheel;
		//left_rad=0;right_rad=0;						
		if(speed_data[7]==0x00) 
			left_speed=(-left_speed);
		if(speed_data[8]==0x01) 
			right_speed=(-right_speed);
		//计算角速度，线速度
  	vel_linear.d = (left_speed+right_speed)/2;
  	vel_angular.d= (right_speed-left_speed)/D;

		//计算偏航角
  	oriention.d+= vel_angular.d*dt;
		//ROS_INFO("orient = %.2f",oriention.d);
  	//计算坐标
  	position_x.d+= vel_linear.d*dt*cos(oriention.d);
	  position_y.d+= vel_linear.d*dt*sin(oriention.d);
  

  	//里程计的偏航角需要转换成四元数才能发布
  	odom_quat = tf::createQuaternionMsgFromYaw(oriention.d);//将偏航角转换成四元数

  	//载入坐标（tf）变换时间戳
  	odom_trans.header.stamp = ros::Time::now();
		//发布坐标变换的父子坐标系
		odom_trans.header.frame_id = "odom";     
		odom_trans.child_frame_id = "base_footprint";       
		//tf位置数据：x,y,z,方向
		odom_trans.transform.translation.x = position_x.d;
		odom_trans.transform.translation.y = position_y.d;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;        
		//发布tf坐标变化
		odom_broadcaster.sendTransform(odom_trans);

		//载入里程计时间戳
		odom.header.stamp = ros::Time::now(); 
		//里程计的父子坐标系
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";       
		//里程计位置数据：x,y,z,方向
		odom.pose.pose.position.x = position_x.d;     
		odom.pose.pose.position.y = position_y.d;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;       
		//载入线速度和角速度
		odom.twist.twist.linear.x = vel_linear.d;
		//odom.twist.twist.linear.y = odom_vy;
		odom.twist.twist.angular.z = vel_angular.d;    
		//发布里程计
		odom_pub.publish(odom);	
		loop_rate.sleep();//周期休眠
			
	}
	g_run  = 0;
   	pthread_join(g_pthread, NULL);
	
	close(g_fd);
	return 0;
}

