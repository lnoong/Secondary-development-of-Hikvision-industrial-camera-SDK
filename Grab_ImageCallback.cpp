#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream> 
#include "/opt/MVS/include/MvCameraControl.h"
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>



using namespace cv; 
using namespace std;
#define MAX_BUF_SIZE    (1920*1200*3)
 void message(int *c){

    int fd,flag,wr_num;
    struct termios options, newstate;
    speed_t baud_rate_i,baud_rate_o;


    unsigned char buf[2]; //向串口发送的数组
if(*c==0){buf[0]={'0'};
buf[1]={'\n'};
}
if(*c==1){buf[0]={'1'};
buf[1]={'\n'};
}
if(*c==2){buf[0]={'2'};
buf[1]={'\n'};
}
if(*c==3){buf[0]={'3'};
buf[1]={'\n'};
}
if(*c==4){buf[0]={'4'};
buf[1]={'\n'};
}
if(*c==5){buf[0]={'5'};
buf[1]={'\n'};
}
if(*c==6){buf[0]={'6'};
buf[1]={'\n'};
}
if(*c==7){buf[0]={'7'};
buf[1]={'\n'};
}
if(*c==8){buf[0]={'8'};
buf[1]={'\n'};
}
if(*c==9){buf[0]={'9'};
buf[1]={'\n'};
}


    fd=open("/dev/ttyUSB0", O_RDWR|O_NONBLOCK|O_NOCTTY|O_NDELAY);	//打开串口
    tcflush(fd, TCIFLUSH);
    tcflush(fd, TCOFLUSH); //清空输出缓存
    tcflush(fd, TCIOFLUSH);//清空输入输出缓存
    tcsetattr(fd, TCSANOW, &options);
    if(fd==-1)
          printf("can not open the COM1!\n");
    else
          printf("open COM1 ok!\n");
    
   if( fcntl(fd, F_SETFL, 0) <0 ) //改为阻塞模式
      printf("fcntl failed\n");
    else
      printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
    
    tcgetattr(fd, &options);

    //设置波特率
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    
    //获取波特率
    tcgetattr(fd, &newstate);
    baud_rate_i=cfgetispeed(&newstate);
    baud_rate_o=cfgetospeed(&newstate);

    //串口设置
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;//设置无奇偶校验位，N
    options.c_cflag &= ~CSTOPB; //设置停止位1
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; //设置数据位
    options.c_cc[VTIME]=0;//阻塞模式的设置
    options.c_cc[VMIN]=1;

    //激活新配置
    tcsetattr(fd, TCSANOW, &options);
    //输出波特率
    printf("输入波特率为%d，输出波特率为%d\n" , baud_rate_i, baud_rate_o);
    
    //写入串口
  wr_num=write(fd,buf,sizeof(buf));
  close(fd);
 }










bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)//判断相机的接入是通过ID还是通过USB
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

void __stdcall ImageCallBackEx(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)//输出一帧图像的信息
{
    if (pFrameInfo)
    {
        printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n", 
        pFrameInfo->nWidth, pFrameInfo->nHeight, pFrameInfo->nFrameNum);
    }
}

int main()
{
    int nRet = MV_OK;
    void* handle = NULL;
    
    do 
    {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        // 枚举设备
        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            break;
        }
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);            
            }
        }
        else
        {
            printf("Find No Devices!\n");
            break;
        }

        printf("Please Intput camera index: ");
        unsigned int nIndex = 0;
        scanf("%d", &nIndex);

        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Intput error!\n");
            break;
        }

        // 选择设备并创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            break;
        }
        
        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            break;
        }
	int WidthValue=1024;
	int HeightValue=768;
	int ExposureTimeValue=40000;
	int GainValue=0;

		
        // 设置触发模式为off
        // set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);

        // 开始取流
        // start grab image
    nRet = MV_CC_StartGrabbing(handle);

    MVCC_INTVALUE stIntvalue = { 0 };
	nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stIntvalue);
	int nBufSize = stIntvalue.nCurValue; //一帧数据大小
	nBufSize = MAX_BUF_SIZE;
	unsigned int    nTestFrameSize = 0;
	unsigned char*  pFrameBuf = NULL;
	pFrameBuf = (unsigned char*)malloc(nBufSize);
 
	MV_FRAME_OUT_INFO_EX stInfo;
	memset(&stInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX)); 

	//上层应用程序需要根据帧率，控制好调用该接口的频率
	//此次代码仅供参考，实际应用建议另建线程进行图像帧采集和处理
	while (true)
	{
            nRet = MV_CC_GetImageForBGR(handle, pFrameBuf, nBufSize, &stInfo, 1000);
			int width = stInfo.nWidth;
			int height = stInfo.nHeight;
			int c;
			if (stInfo.enPixelType == PixelType_Gvsp_BGR8_Packed)
			{
				Mat pImg(height, width, CV_8UC3, pFrameBuf);
				Mat test_image = pImg;

	//resize(test_image,test_image,Size(test_image.cols/3,test_image.rows/3),0,0,INTER_LINEAR);
	Ptr<aruco::Dictionary> dictionary =aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
	vector<vector<Point2f>>corners, rejectedImgPoints;
	vector<int>ids(5,0);
	auto parameters = aruco::DetectorParameters::create();
	aruco::detectMarkers(test_image, dictionary, corners, ids, parameters, rejectedImgPoints);
	aruco::drawDetectedMarkers(test_image, corners, ids, Scalar(0, 255, 0));
	int t;
				t=ids[0];
				if(t==0)
				{
				c=0;
				message(&c);
				}
if(t==1)
				{
				c=1;
				message(&c);
				}
if(t==2)
				{
				c=2;
				message(&c);
				}
if(t==3)
				{
				c=3;
				message(&c);
				}
if(t==4)
				{
				c=4;
				message(&c);
				}
if(t==5)
				{
				c=5;
				message(&c);
				}
if(t==6)
				{
				c=6;
				message(&c);
				}
if(t==7)
				{
				c=7;
				message(&c);
				}
if(t==8)
				{
				c=8;
				message(&c);
				}
if(t==9)
				{
				c=9;
				message(&c);
				}
				//imshow("Image1",test_image);
				imshow("Image1",test_image);
				waitKey(1);
			}

			nTestFrameSize++;             
	}

    } 
    while (0);
    return 0;
}
