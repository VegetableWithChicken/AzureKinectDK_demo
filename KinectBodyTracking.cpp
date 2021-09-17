#include <stdio.h>
#include <stdlib.h>
#include "k4a/k4a.hpp"
#include "k4abt.hpp"

#include<winsock.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <list>
#include <algorithm>
#include <ios>
using namespace std;

#pragma comment(lib,"ws2_32.lib")


#include <chrono>
#include <ctime>
#include <time.h>
#include <conio.h>
#include <direct.h>

using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

#define FRAME_NUM 10000

void Initialization()//初始化套接字
{
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2);//版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "初始化套接字库失败！" << endl;
	}
	else {
		cout << "初始化套接字库成功！" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
	}
	else {
		cout << "套接字库版本正确！" << endl;
	}

}

int main()
{
	//初始化设备
	k4a::device NativeKinectDevice = NULL;
	NativeKinectDevice = k4a::device::open(0);

	// Start camera. Make sure depth camera is enabled.
	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
	deviceConfig.synchronized_images_only = true;
	deviceConfig.wired_sync_mode = k4a_wired_sync_mode_t::K4A_WIRED_SYNC_MODE_STANDALONE;
	k4a_calibration_t sensor_calibration = NativeKinectDevice.get_calibration(deviceConfig.depth_mode, deviceConfig.color_resolution);
	uint32_t timeOutInMillisecs = 1000;
	std::chrono::milliseconds TimeOutInMilliSecsConverted = std::chrono::milliseconds(timeOutInMillisecs);
	k4abt::tracker NativeTracker = k4abt::tracker::create(sensor_calibration);
	NativeTracker.set_temporal_smoothing(0.8);

	NativeKinectDevice.start_cameras(&deviceConfig);


	//服务器
	Initialization();

	SOCKET Server_s = socket(AF_INET, SOCK_STREAM, 0);
	sockaddr_in server_addr;
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	server_addr.sin_port = htons(5155);
	server_addr.sin_family = PF_INET;
	bind(Server_s, (SOCKADDR*)&server_addr, sizeof(sockaddr));
	listen(Server_s, 10);
	cout << "正在监听" << endl;


	cout << "服务端正在监听连接，请稍候...." << endl;
	int  recvdata = sizeof(SOCKADDR);
	sockaddr accept_addr;
	SOCKET accept_s = accept(Server_s, (SOCKADDR*)&accept_addr, &recvdata);
	if (accept_s)
	{
		do {
			if (!NativeKinectDevice)
			{
				cout << "Kinect device for capturing body tracking frame is Invalid!" << endl;
				return 0;
			}
			if (!NativeTracker)
			{
				cout << "Body Tracker for capturing body tracking frame is Invalid!" << endl;
				return 0;
			}
			// Capture a depth frame
			k4a::capture sensorCapture = nullptr;
			if (!NativeKinectDevice.get_capture(&sensorCapture, TimeOutInMilliSecsConverted))
			{
				cout << "Kinect device get capture Timed Out!";
				return 0;
			}
			try {

				// Enqueue the capture
				if (!NativeTracker.enqueue_capture(sensorCapture, TimeOutInMilliSecsConverted))
				{
					cout << "Adding capture to the Tracker process queue Timed Out!";
					return 0;
				}

				k4abt::frame bodyFrame = nullptr;
				if (!NativeTracker.pop_result(&bodyFrame, TimeOutInMilliSecsConverted))
				{
					cout << "Tracker pop body frame result Timed Out!";
					return 0;
				}
				uint32_t numBodies = bodyFrame.get_num_bodies();
				if (numBodies) {

					k4abt_body_t body;
					bodyFrame.get_body_skeleton(0, body.skeleton);

					send(accept_s, (char*)&body, sizeof(body), 0);
					cout << body.skeleton.joints[27].position.xyz.x << endl;
				}
				if (_kbhit() && _getch() == 0x1b)
				{
					NativeTracker.shutdown();
					NativeKinectDevice.close();
					return 0;
				}
			}
			catch (...)
			{
				NativeTracker.shutdown();
				NativeKinectDevice.close();
			}
		} while (true);
		return 0;
	}
}