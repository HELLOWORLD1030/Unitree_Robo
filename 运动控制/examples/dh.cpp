/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <thread>
#include <sys/socket.h>
#include <sys/time.h>
#include <sstream>
#include <vector>
#include <msgpack.hpp>

using namespace UNITREE_LEGGED_SDK;

const int PC_PORT = 32000;
const char PC_IP_ADDR[] = "192.168.123.161";
const int UDP_BUFFER_SIZE = 128;

// Globals
bool RUNNING = true; // indicating whether the main loop is running

std::string movement;
double command_number = 0.0;

class UdpReceiver
{
public:
	std::deque<std::string> msg_queue;

	/* udp receiver thread */
	void run()
	{
		/********************UDP_Receiving_Initializing********************/
		socklen_t fromlen;
		struct sockaddr_in server_recv;
		struct sockaddr_in hold_recv;

		int sock_recv = socket(AF_INET, SOCK_DGRAM, 0);

		int sock_length_recv = sizeof(server_recv);
		bzero(&server_recv, sock_length_recv);

		server_recv.sin_family = AF_INET;
		server_recv.sin_addr.s_addr = INADDR_ANY;
		server_recv.sin_port = htons(PC_PORT); // Setting port of this program
		bind(sock_recv, (struct sockaddr *)&server_recv, sock_length_recv);
		fromlen = sizeof(struct sockaddr_in);
		char buf_UDP_recv[UDP_BUFFER_SIZE]; // for holding UDP data
		/******************************************************************/
		while (RUNNING)
		{
            memset(buf_UDP_recv, 0, sizeof buf_UDP_recv);
			int datalength = recvfrom(sock_recv, buf_UDP_recv, UDP_BUFFER_SIZE, 0, (struct sockaddr *)&hold_recv, &fromlen);
            std::string str(buf_UDP_recv);

            msg_queue.push_back(buf_UDP_recv);
            
            while(msg_queue.size()>1){
                msg_queue.pop_front();
            }
            movement = msg_queue[0];
		}
	}
};


class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)){
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    int time=0;
    bool isScanQrCode=false;
    int flag=0;
    float dt = 0.002;     // 0.001~0.01
};


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime += 1;
    udp.GetRecv(state);
    //printf("%d   %f\n", motiontime, state.imu.quaternion[2]);

    cmd.mode = 0;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;

    if (motiontime % 2){
        cmd.mode = 2;
        cmd.gaitType = 0;
        cmd.velocity[0] = 0.6f; // -1  ~ +1
        cmd.footRaiseHeight = 0.1;
    
    }
    else {
        if (movement == "not detected block")
            cmd.mode = 0;
        else {
            float x_pos = atof(movement.c_str());
        
            std::cout << x_pos << std::endl;

            if (x_pos > 335){
                cmd.mode = 2;
                cmd.gaitType = 0;
                cmd.velocity[1] = -0.3f; // -1  ~ +1
                cmd.footRaiseHeight = 0.1;
                std::cout << "right" << std::endl;
            }
            else if (x_pos < 305&&x_pos>=0) {
                cmd.mode = 2;
                cmd.gaitType = 0;
                cmd.velocity[1] = 0.3f; // -1  ~ +1
                cmd.footRaiseHeight = 0.1;
                std::cout << "left" << std::endl;
            }
            else if(x_pos<0){
                if(!isScanQrCode){
                isScanQrCode=true;
                flag=(int)x_pos;
                }
                switch(flag){
                    case -1:
		    while(time< 100000)
			 {
			   std::cout<<time<<"  -1"<<std::endl;
               if(time<30000)
               {
                   cmd.mode = 0;
               }
			   if(time< 60000 && time>30000)
			    {
				cmd.mode = 1;
				cmd.bodyHeight = -0.5;
			    }
			   if(time > 60000 && time < 90000)
			    {
				cmd.mode = 1;
				cmd.bodyHeight = 0;
			    }
			    time+=1;
			    udp.SetSend(cmd);
			 }
                    break;
                    case -2:
		    while(time < 100000)
			 {
                            std::cout<<time<<"  -2"<<std::endl;
                            if(time<10000)
                             {
                                  cmd.mode = 0;
                             }
                           if(time<30000 && time>10000)
                            {
                            	cmd.mode = 1;
                               	cmd.euler[1] = 0.6;
                            }
                    	   if(time>30000 && time<60000)
			    {
                                cmd.mode = 1;
                                cmd.euler[1] = -0.6;
                    	    }
		           if(time>60000 && time<90000)
		            {
			        cmd.mode = 1;
			        cmd.euler[1] = 0.6;
			    }
			    time+=1;
                            udp.SetSend(cmd);
		         }
                    break;
                    case -3:
		    while(time <100000)
			 {
                 if(time<10000)
               {
                   cmd.mode = 0;
               }
               else
			   { std::cout<<time<<"  -3"<< std::endl;
                    	    cmd.mode=2;
                            cmd.yawSpeed=0.9;
               }
			    time+=1;
			    udp.SetSend(cmd);
			 }
                    break;
                    case -4:
		    while(time <100000)
			 {
                 if(time<10000)
               {
                   cmd.mode = 0;
               }
               else
               {
			    std::cout<<time<<"  -4"<< std::endl;
                    	    cmd.mode=2;
                            cmd.yawSpeed=-0.9;
               }
			    time+=1;
			    udp.SetSend(cmd);
			 }
                    break;
                }
            }
            else {
                cmd.mode = 2;
                cmd.gaitType = 0;
                cmd.velocity[0] = 0.4f; // -1  ~ +1
                cmd.footRaiseHeight = 0.1;
                std::cout << "stay" << std::endl;

            }
        }
    }
    if(time>=100000)
    {
        cmd.mode = 0;
    }
    // cmd.mod
    if(motiontime %100==0)
    {
        std::cout<<"correct"<<std::endl;
        cmd.yawSpeed=0.3;
    }
    udp.SetSend(cmd);
}

int main(void) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
	UdpReceiver udp_receiver = UdpReceiver();
	std::thread udp_recv_thread(&UdpReceiver::run,&udp_receiver); // run udpRecv in a separate thread


    Custom custom(HIGHLEVEL);
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        //std::cout << movement << std::endl;
        sleep(10);
    };
	if (udp_recv_thread.joinable())
	{
		udp_recv_thread.joinable();
	}
    return 0; 
}
