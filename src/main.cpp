/*****************************************************************************
*
* Freescale Confidential Proprietary
*
* Copyright (c) 2016 Freescale Semiconductor;
* All Rights Reserved
*
*****************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include "ImageStitching.h"
#include "uart_to_mcu.h"
#include "uart_to_android.h"
#include "fb_helper.h"
#include "network.h"
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include <errno.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#ifndef __STANDALONE__
  #include <signal.h>
  #include <termios.h>
#endif // #ifdef __STANDALONE__
#include <string.h>

#ifdef __STANDALONE__
  #include "frame_output_dcu.h"
  #define CHNL_CNT io::IO_DATA_CH3
#else // #ifdef __STANDALONE__
  #include "frame_output_v234fb.h"
  #define CHNL_CNT io::IO_DATA_CH3
#endif // else from #ifdef __STANDALONE__

#include "oal.h"
#include "vdb_log.h"
#include "sdi.hpp"

#include "max9286_96705_4_uyvy_c.h"

#include "vdb_log.h"

///////////////////////////////////new head files//////////////////////////////////////////////////////
//#include <stdio.h>
#include <fstream>

#include "CameraDevice.h"
#include "DrawScene.h"
#include "ImageBilt.h"
#include "FinalScene.h"

#include "keyboardInput.h"
#include "OGLEGL.h"
#include "PanoramaParam.h"
#include "Panorama3DScene.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "dump.h"
#include "utils/Timer.hpp"

//#include "json.hpp"
//using json = nlohmann::json;


bool fbo_on = false;

char path_pattern[256+1]={0};
int num_frames = 0;


using namespace keyboard_device;   //ʹ�ñ�׼�����豸

//for gl end

//****************************************************************************//
using namespace cv;
extern unsigned int McuTimeData;
extern unsigned long long Mcu_tt;
unsigned long long Soc_tt;
BasicData *BasicData_Ptr;
CamData *CamData_Ptr;
ParkingPlace *ParkingPlace_Ptr;
McuSendDada *McuSendDada_Ptr;
LanelineData *LanelineData_Ptr;
ObstacleData *ObstacleData_Ptr;

PCWRITEDATA *PCWRITEDATA_Ptr;

extern volatile unsigned int CounterTick;
unsigned long long GetTime(void);
unsigned long long GetNowTimeUs(void);
extern int num;
extern int McuSendOffset[10*3];
extern unsigned int McuSendCounter[10];
extern int reset_status_changed_num;
extern int reset_new_value_cnt;
extern int reset_new_value_cnt_flag;
extern int reset_chang_flag;

extern SOCREADDATA AndroidSend_SocReadData;
extern SOCWRITEDATA AndroidReceive_SocWriteData;

vsdk::SMat frame_map[4];
//unsigned char* frameDataPtr[4];
vsdk::SMat frame_map_out;

unsigned char  mem_tmp_T0[IMG_WIDTH*IMG_HEIGHT*2];
unsigned char  mem_tmp_T1[IMG_WIDTH*IMG_HEIGHT*2];
unsigned char  mem_tmp_T2[IMG_WIDTH*IMG_HEIGHT*2];
unsigned char  mem_tmp_T3[IMG_WIDTH*IMG_HEIGHT*2];

void *VideoCaptureTask(void * ptr);
void *GLTask(void * ptr);
int Ipc_Create();
//int saveframe(char *str, void *p, int length, int is_oneframe);
/**********************************************************Parking Management***************************/
typedef struct parkplace{
			float ParkPlace[4*2];
			int x;
			int y;
			int z;
			int park_confidence;
			struct parkplace *next;
		} LinkList;  //定义车位链表节点
static  int TimeCntTmp1;
LinkList *head; //定义头节点
ParkingPlace ParkingPlace_PtrCopy;

int disFront;    //后轴中心点到环视前视野边缘距离
int disBack;		//后轴中心点到环视后视野边缘距离
int disLF;       //后轴中心点到环视左/右视野边缘距离
int validNode;   //当前链表中在视野范围内的节点数（即车位数）

//#define Rear_Axle_Center_x  405
//#define Rear_Axle_Center_y  680
//#define Pixel_Ration 9.83
///////1200x1080 2D VIEW/////////////////////////////////////////////////
//车身坐标系：后轴中心点为原点，车头方向为X+，左侧为Y+
//屏幕坐标系，左上角为原点，水平为X，垂直为Y

int  Rear_Axle_Center_x = 1320;			//屏幕坐标系，后轴中心点X坐标
int  Rear_Axle_Center_y = 700;//654+20;		//屏幕坐标系，后轴中心点Y坐标
float  Pixel_Ration_x = 9.448;//9.62;				//屏幕坐标系，垂直方向
float  Pixel_Ration_y = 8.667;//9.5;				//屏幕坐标系，水平方向

///////720x900 2D VIEW/////////////////////////////////
int  Rear_Axle_Center_x2 = 1560;		//屏幕坐标系，后轴中心点X坐标
int  Rear_Axle_Center_y2 = 583;//564;//600;	//屏幕坐标系，后轴中心点Y坐标
float  Pixel_Ration2_x = 9.448;//9.62;  			//屏幕坐标系，垂直方向
float  Pixel_Ration2_y = 8.667;//9.5;	  			//屏幕坐标系，水平方向


////////////////////////////////////////////////////////////////////////
int disLRCam;
int Mcu_Send_Parking_Data_Fag;
int Parking_Select_Num ; //parking place selected
int Total_Parking_Num ;
int Car_Parking_Status;
int Parking_Place_Mode_Select_OK;
int Car_Speed_Flag;

int parking_place_detected_flag;
int parking_place_detected_num;
extern TARGETPLACEDATA ToMcuTargetData;

int parking_in_scan_direction;  	//1 scan left      2 scan right

static void coord_pts2veh(short int curr_coords[3], short int prev_coords[3], float prev_pts[8], float pts2veh[8]);
static void coord_cc2oc(short int coords[3], float cc[8], float oc[8]);
void insert_trail(LinkList * list,LinkList * newnode);
void deleteNode(LinkList *list, int n) ;
void deleteLinklist(LinkList *list);
void ParkingManagemet();

/******************************************************************************/

//***************************************************************************

// Possible to set input resolution (must be supported by the DCU)
#define WIDTH           1280 ///< width of DDR buffer in pixels
#define HEIGHT          960 ///< height of DDR buffer in pixels
#define DDR_BUFFER_CNT  3    ///< number of DDR buffers per ISP stream

static uint8_t          OutPutBufferUYVY[WIDTH*HEIGHT*2];
static uint8_t          RGB24Buffer[WIDTH*HEIGHT*3];


//***************************************************************************

#define FRM_TIME_MSR 300 ///< number of frames to measure the time and fps

#ifdef __STANDALONE__
//extern SEQ_Buf_t  producerless_buffer_1;
extern "C" {
  unsigned long get_uptime_microS(void);
}

#define GETTIME(time)   (*(time)=get_uptime_microS())
#else // ifdef __STANDALONE__
#define GETTIME(time) \
  { \
  struct timeval lTime; gettimeofday(&lTime,0); \
  *time=(lTime.tv_sec*1000000+lTime.tv_usec);   \
  }
#endif // else from #ifdef __STANDALONE__

/************************************************************************/
/** User defined call-back function for Sequencer event handling.
  *
  * \param  aEventType defines Sequencer event type
  * \param  apUserVal  pointer to any user defined object
  ************************************************************************/
void SeqEventCallBack(uint32_t aEventType, void* apUserVal);

#ifndef __STANDALONE__
/************************************************************************/
/** SIGINT handler.
  *
  * \param  aSigNo
  ************************************************************************/
void SigintHandler(int);

/************************************************************************/
/** SIGINT handler.
  *
  * \param  aSigNo
  *
  * \return SEQ_LIB_SUCCESS if all ok
  *         SEQ_LIB_FAILURE if failed
  ************************************************************************/
int32_t SigintSetup(void);

//***************************************************************************

static bool sStop = false; ///< to signal Ctrl+c from command line

#endif // #ifndef __STANDALONE__
/************************************************************************/
/** Allocates contiguous DDR buffers for one ISP stream.
  *
  * \param  appFbsVirtual array of virtual buffer pointers to be filled
  * \param  apFbsPhysical array of buffer physical addresses to be filled
  * \param  aMemSize      size of the array in bytes
  * \param  aBufferCnt    number of buffers to be allocated
  *
  * \return 0 if all OK
  *         < 0 otherwise
  ************************************************************************/
int32_t DdrBuffersAlloc(void** appFbsVirtual, uint32_t*  apFbsPhysical,
                        size_t aMemSize, uint32_t aBufferCnt);

/************************************************************************/
/** Frees DDR buffers for one ISP stream.
  *
  * \param  appFbsVirtual array of virtual buffer pointers to be filled
  * \param  apFbsPhysical array of buffer physical addresses to be filled
  * \param  aBufferCnt    number of buffers to be freed
  ************************************************************************/
void DdrBuffersFree(void** appFbsVirtual, uint32_t*  apFbsPhysical,
                    uint32_t aBufferCnt);


static void TermNonBlockSet()
{
  struct termios lTermState;

  // get current state of the terminal
  tcgetattr(STDIN_FILENO, &lTermState);

  // disable canonical mode
  lTermState.c_lflag &= ~ICANON;
  // set minimum number of input characters to read
  lTermState.c_cc[VMIN] = 1;

  // set the terminal attributes
  tcsetattr(STDIN_FILENO, TCSANOW, &lTermState);
} // TermNonBlockSet()



static int KeyDown()
{
  struct timeval lTv;
  fd_set lFdSet;
  lTv.tv_sec = 0;
  lTv.tv_usec = 50;
  FD_ZERO(&lFdSet);
  FD_SET(STDIN_FILENO, &lFdSet);
  select(STDIN_FILENO + 1, &lFdSet, NULL, NULL, &lTv);
  return FD_ISSET(STDIN_FILENO, &lFdSet);
} // KeyDown()


static int GetCharNonBlock()
{
  int lChar = EOF;

  usleep(1);
  if(KeyDown())
  {
    lChar = fgetc(stdin);
  } // if Key pressed

  return lChar;
} // KeyDown()

static char GetChar()
{
#ifdef __STANDALONE__
  return sci_0_testchar();
#else // #ifdef __STANDALONE__
  int lChar = GetCharNonBlock();
  return (char)((lChar < 0)? 0: lChar&0xff);
#endif // else from #ifdef __STANDALONE__
} // GetChar()
#if 1

//#define IMAGE_DEBUG_MODE

int main(int, char **)
{
	int ret = 0;
	pthread_t gltask,cltask,keytask,tertask,guitask,uartrxtask,uarttxtask,videotask,id1;
	pthread_t  net_staus_thread,tr_thread,rv_thread, uartandroidrxtask, uartandroidtxtask;

	Ipc_Create();
	//OpenglInit();
#ifndef IMAGE_DEBUG_MODE
	pthread_create(&videotask, NULL, VideoCaptureTask,NULL);
#endif

	sleep(1);
	printf("start GLTask\n");
#ifdef IMAGE_DEBUG_MODE
	ret = pthread_create(&gltask, NULL, GLTask,NULL);
#endif
	if(ret)
	{
		printf("Create GL Task error!\n");
		return 1;
	}

	//ret = pthread_create(&cltask, NULL, CLTask,NULL);
	if(ret)
	{
		printf("Create CLTask error!\n");
		return 1;
	}

	ret = pthread_create(&keytask, NULL, KeyTask,NULL);
	if(ret)
	{
		printf("Create keytask error!\n");
		return 1;
	}

	//ret = pthread_create(&tertask, NULL, TerminalTask,NULL);
	if(ret)
	{
		printf("Create TerminalTask error!\n");
		return 1;
	}

	//ret = pthread_create(&guitask, NULL, Gui_meg_thread,NULL);
	//pthread_create(&id1, NULL, Gui_draw_thread,NULL);
///	if(ret)
///	{
///		printf("Create Gui_meg_thread error!\n");
///		return 1;
///	}
#ifndef IMAGE_DEBUG_MODE
	ret = pthread_create(&uartrxtask, NULL, Uart_meg_thread,NULL);
	if(ret)
	{
		printf("Create Uart_meg_thread error!\n");
		return 1;
	}
	ret = pthread_create(&uarttxtask, NULL, Uart_TX_thread,NULL);
	if(ret)
	{
		printf("Create Uart_TX_thread error!\n");
		return 1;
	}
	ret = pthread_create(&tr_thread, NULL, net_tr_thread, NULL);
	if(ret)
	{
		printf("Create net_tr_thread error!\n");
		return 1;
	}
	ret = pthread_create(&uartandroidrxtask, NULL, Uart_to_Android_RX_thread,NULL);
    if(ret)
    {
        printf("Create Uart_to_Android_RX_thread error!\n");
        return 1;
    }

    ret = pthread_create(&uartandroidtxtask, NULL, Uart_to_Android_TX_thread,NULL);
    if(ret)
    {
        printf("Create Uart_to_Android_TX_thread error!\n");
        return 1;
    }
#endif

#ifndef IMAGE_DEBUG_MODE
	pthread_join(videotask,NULL);
#else
	pthread_join(gltask, NULL);
#endif
	//pthread_join(cltask,NULL);
	pthread_join(keytask,NULL);
	//pthread_join(tertask,NULL);
	//pthread_join(guitask,NULL);
	pthread_join(uartrxtask,NULL);
	pthread_join(uarttxtask,NULL);
	pthread_join(tr_thread,NULL);
	pthread_join(uartandroidrxtask,NULL);
    pthread_join(uartandroidtxtask,NULL);
	return 0;
}
#endif

void *VideoCaptureTask(void *ptr1)
{
  LIB_RESULT lRet = LIB_SUCCESS;
  LIB_RESULT lRes = LIB_SUCCESS;

  char cam_cmd[12];
  char cur_char;
  //vsdk::SMat frame_map_UYVY;

  OAL_Initialize();

  //*** Init DCU Output ***
#ifdef __STANDALONE__
  io::FrameOutputDCU
    lDcuOutput(WIDTH,
               HEIGHT,
               io::IO_DATA_DEPTH_08,
               io::IO_DATA_CH3,
               DCU_BPP_YCbCr422
               );
#else // #ifdef __STANDALONE__

  // setup Ctrl+C handler
  //if(SigintSetup() != SEQ_LIB_SUCCESS)
  //{
  //  VDB_LOG_ERROR("Failed to register Ctrl+C signal handler.");
    //return -1;
  //}

  //printf("Press Ctrl+C to terminate the demo.\n");

  // *** set terminal to nonblock input ***
  //TermNonBlockSet();

  /*
  io::FrameOutputV234Fb
    lDcuOutput(1280,
               720,
               io::IO_DATA_DEPTH_08,
               io::IO_DATA_CH3,
               DCU_BPP_YCbCr422
               );
  */
#endif // else from #ifdef __STANDALONE__
  //printf("main.cpp line135\n");
  //
  // *** Initialize SDI ***
  //
  lRes = sdi::Initialize(0);
  //printf("main.cpp line140\n");
  // *** create grabber ***
  sdi_grabber *lpGrabber = new(sdi_grabber);
  lpGrabber->ProcessSet(gpGraph, &gGraphMetadata);
  //printf("main.cpp line144\n");
  // *** set user defined Sequencer event handler ***
  int32_t lCallbackUserData = 12345;
  lpGrabber->SeqEventCallBackInstall(&SeqEventCallBack, &lCallbackUserData);

  // *** prepare IOs ***
  sdi_FdmaIO *lpFdma = (sdi_FdmaIO*)lpGrabber->IoGet(SEQ_OTHRIX_FDMA);

  // modify DDR frame geometry to fit display output
  SDI_ImageDescriptor lFrmDesc = SDI_ImageDescriptor(WIDTH, HEIGHT, YUV422Stream_UYVY);
  lpFdma->DdrBufferDescSet(0, lFrmDesc);

  lFrmDesc = SDI_ImageDescriptor(WIDTH, HEIGHT, YUV422Stream_UYVY);
  lpFdma->DdrBufferDescSet(1, lFrmDesc);

  lFrmDesc = SDI_ImageDescriptor(WIDTH, HEIGHT, YUV422Stream_UYVY);
  lpFdma->DdrBufferDescSet(2, lFrmDesc);

  lFrmDesc = SDI_ImageDescriptor(WIDTH, HEIGHT, YUV422Stream_UYVY);
  lpFdma->DdrBufferDescSet(3, lFrmDesc);

  //*** allocate DDR buffers ***
  lpFdma->DdrBuffersAlloc(DDR_BUFFER_CNT);
  //printf("main.cpp line154\n");

  TermNonBlockSet();


  // *** prestart grabber ***
  lpGrabber->PreStart();
  //printf("main.cpp line157\n");
  // fetched frame buffer storage
  SDI_Frame lFrame[4];

  // *** start grabbing ***
  lpGrabber->Start();
  //printf("main.cpp line163\n");
  unsigned long lTimeStart = 0, lTimeEnd = 0, lTimeDiff = 0;
  GETTIME(&lTimeStart);

  uint32_t actualBufferIndex = 2;

  uint32_t lFrmCnt = 0;
  printf("Complete init done.\n");
  uint8_t lLoop=0;
  uint8_t num=0;


  static int nn;
  static int numCopy;
  static int McuSendOffsetCopy[10*3];
  static unsigned int McuSendCounterCopy[10];
  int numtmp;
  //frameDataPtr[0] = mem_tmp_T0;
  //frameDataPtr[1] = mem_tmp_T1;
  //frameDataPtr[2] = mem_tmp_T2;
  //frameDataPtr[3] = mem_tmp_T3;
  ///////////////////////////////////////////////////////opengl init///////////////////
  	if(initialGL(1920,1080) != 0)          //初始化opengl es环境
    {
	     fprintf(stderr,"opengl init unsucess!\n");
	     //return -1;
	}
	printf("initialGL()::\n");
	auto glVer = glGetString(GL_VERSION);
	if (!glVer)
	{
		printf("glGetString failed: %d\n", glGetError());
	}
	else
	{
		printf("OpenGL version: %s\n", glGetString(GL_VERSION));
	}

	//if(getCameraDevice().initCamera() != 0)   //初始化视频采集设备
	{
	     printf("can not initialize the camera device！\n");
	     //return -1;
	}
	printf("getCameraDevice()::\n");

	if(initPanoramaParam() != 0){
	    printf("can not initialize the camera param");
	    //return -1;
	}

	printf("initPanoramaParam() success!\n");
	keyboard_device::initDevice();                            //使用标准键盘设备
	printf("keyboard_device::initDevice() sucess!\n");
	CFinalScene scene;
	scene.setBirdEyeViewport(0,
							 0,
							 810,
							 1080);
	scene.setSingleViewport(0,
							0,
							1056,
							1080);
	scene.set3DViewport(810,
							270,
							1110,
							810);
	scene.onStart();

	//std::cout << displaySetting << std::endl;
	//Calib_2D_Init();
  ///////////////////////////////////////////////////////////////////////////////////////
  while(1)
  {
	  for (int i = 0;i < 4;i++)
	  {
		  lFrame[i] = lpGrabber->FramePop(i);
		  /*IFrame[0] -->T1 ,IFrame[1] -->T2 ,IFrame[2] -->T3 ,IFrame[3] -->T4 ****wyf added 2017.11.29**/
		  if(lFrame[i].mUMat.empty())
		  {
			  printf("Failed to grab image number %u\n", i);
			  //break;
		  } // if pop failed
	  }
	  //GETTIME(&lTimeEnd2);
	  //time_elapsed = (lTimeEnd2-lTimeStart2)/1000;
	  //GETTIME(&lTimeStart2);
	  #if 1
	  for (int i = 0;i < 4;i++)
	  {
		  frame_map[i] = lFrame[i].mUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	  }
	  //frame_map_out = lFrame[3].mUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);

	  buf_camfront = (unsigned char *)frame_map[0].data;
	  buf_camback  = (unsigned char *)frame_map[1].data;
	  buf_camleft  = (unsigned char *)frame_map[2].data;
	  buf_camright = (unsigned char *)frame_map[3].data;
	  //memcpy(mem_tmp_T0,  (unsigned char *)frame_map[0].data, 1280*720*2 );
	  //memcpy(mem_tmp_T1,  (unsigned char *)frame_map[1].data,  1280*720*2 );
	  //memcpy(mem_tmp_T2,  (unsigned char *)frame_map[2].data,  1280*720*2 );
	  //memcpy(mem_tmp_T3,  (unsigned char *)frame_map[3].data, 1280*720*2 );
	  #endif

	  memcpy(CamData_Ptr->Mem_FrontCam, frame_map[0].data, 1280*960*2 );
	  memcpy(CamData_Ptr->Mem_BackCam, frame_map[1].data,  1280*960*2 );
	  memcpy(CamData_Ptr->Mem_LeftCam, frame_map[2].data,  1280*960*2 );
	  memcpy(CamData_Ptr->Mem_RightCam, frame_map[3].data, 1280*960*2 );

	  CounterTick = GetTime( ); //获取当前基准COUNTER值

	  unsigned char LA[4],LB[4],SA[4],SB[4];
	  float dT[8];

	  LA[0] = CamData_Ptr->Mem_FrontCam[9];
	  LB[0] = CamData_Ptr->Mem_FrontCam[8];
	  SA[0] = CamData_Ptr->Mem_FrontCam[11];
	  SB[0] = CamData_Ptr->Mem_FrontCam[10];
	  dT[0] = (LA[0]*256+LB[0])*42.708/1000;
	  dT[1] = (SA[0]*256+SB[0])*42.708/1000;

	  LA[1] = CamData_Ptr->Mem_BackCam[9];
	  LB[1] = CamData_Ptr->Mem_BackCam[8];
	  SA[1] = CamData_Ptr->Mem_BackCam[11];
	  SB[1] = CamData_Ptr->Mem_BackCam[10];
	  dT[2] = (LA[1]*256+LB[1])*42.708/1000;
	  dT[3] = (SA[1]*256+SB[1])*42.708/1000;

	  LA[2] = CamData_Ptr->Mem_LeftCam[9];
	  LB[2] = CamData_Ptr->Mem_LeftCam[8];
	  SA[2] = CamData_Ptr->Mem_LeftCam[11];
	  SB[2] = CamData_Ptr->Mem_LeftCam[10];
	  dT[4] = (LA[2]*256+LB[2])*42.708/1000;
	  dT[5] = (SA[2]*256+SB[2])*42.708/1000;

	  LA[3] = CamData_Ptr->Mem_RightCam[9];
	  LB[3] = CamData_Ptr->Mem_RightCam[8];
	  SA[3] = CamData_Ptr->Mem_RightCam[11];
	  SB[3] = CamData_Ptr->Mem_RightCam[10];
	  dT[6] = (LA[3]*256+LB[3])*42.708/1000;
	  dT[7] = (SA[3]*256+SB[3])*42.708/1000;

	  float dTmax = dT[0];
	  for(int i=0;i<8;i++)
	  {
		  if(dT[i]>dTmax)
			  dTmax = dT[i];
	  }
	  CamData_Ptr->TimeStamp = CounterTick-dTmax-10; //补偿摄像头的曝光时间
	  printf("TimeStamp==%d dTmax==%d\n",CamData_Ptr->TimeStamp,dTmax);
	  //printf("N7F %x,N8 %x\n",CamData_Ptr->Mem_FrontCam[6],CamData_Ptr->Mem_FrontCam[7]);
	  //printf("N7B %x,N8 %x\n",CamData_Ptr->Mem_BackCam[6],CamData_Ptr->Mem_BackCam[7]);
	  //printf("N7L %x,N8 %x\n",CamData_Ptr->Mem_LeftCam[6],CamData_Ptr->Mem_LeftCam[7]);
	  //printf("N7R %x,N8 %x\n",CamData_Ptr->Mem_RightCam[6],CamData_Ptr->Mem_RightCam[7]);
	  //printf("T0:%f,%f T1:%f,%f T2:%f,%f T3:%f,%f  dTmax=%f\n", dT[0],dT[1], dT[2],dT[3], dT[4],dT[5], dT[6],dT[7],dTmax);

	  if(num >0)
	  {
		  numCopy = num-1;
	  }
	  else if(num == 0)
	  {
		  numCopy=9;
	  }
	  for(int j=0;j<10;j++)
	  {
		  McuSendOffsetCopy[j*3]   = McuSendOffset[j*3];
		  McuSendOffsetCopy[j*3+1] = McuSendOffset[j*3+1];
		  McuSendOffsetCopy[j*3+2] = McuSendOffset[j*3+2];
		  McuSendCounterCopy[j]    = McuSendCounter[j];
		  printf("McuSendCounterCopy[%d]=%u xyz:%d %d %d\n",j,McuSendCounterCopy[j],McuSendOffsetCopy[j*3],McuSendOffsetCopy[j*3+1],McuSendOffsetCopy[j*3+2]);
	  }
	  /////////////////////////////////////////////////////////////////////////////////////
	  int maxtimecnt = McuSendCounterCopy[0];
	  for(int i=0;i<10;i++)
	  {
	  	if(maxtimecnt < McuSendCounterCopy[i])
			maxtimecnt = McuSendCounterCopy[i];
	  }
	  for(int i=0;i<10;i++)
	  {
	  	if(maxtimecnt == McuSendCounterCopy[i])
	  	{
	  		numCopy = i;
			printf("Max numCopy==%d\n",numCopy);
	  	}
	  }
	  for(int j=0;j<10;j++)  //找到缓冲数组中距离补偿过的Counter值最近的序号
	  {
		  if(j<9)
		  {
			  if( CamData_Ptr->TimeStamp >= McuSendCounterCopy[j] && CamData_Ptr->TimeStamp<McuSendCounterCopy[j+1])
			  {
				  numCopy = j;
				  printf("ok1 numCopy=%d\n",j);
				  break;
			  }
		  }
		  else
		  {
			  if( CamData_Ptr->TimeStamp >= McuSendCounterCopy[9] && CamData_Ptr->TimeStamp<McuSendCounterCopy[0])
			  {
				  numCopy = j;
				  printf("ok2 numCopy=%d\n",j);
				  break;
			  }
		  }
	  }

		if(numCopy>0)
		{
			numtmp = numCopy-1;
		}
		else if(numCopy ==0)
		{
			numtmp = 9;
		}
		printf("numCopy=%d numtmp=%d\n",numCopy,numtmp);
		float deltaX = McuSendOffsetCopy[numCopy*3]-McuSendOffsetCopy[numtmp*3];
		float deltaY = McuSendOffsetCopy[numCopy*3+1]-McuSendOffsetCopy[numtmp*3+1];
		float deltaZ = McuSendOffsetCopy[numCopy*3+2]-McuSendOffsetCopy[numtmp*3+2];
		float deltaCnt = McuSendCounterCopy[numCopy]-McuSendOffsetCopy[numtmp];
		float deltaCntNow = CamData_Ptr->TimeStamp-McuSendCounterCopy[numCopy];
		CamData_Ptr->x = deltaX/deltaCnt*deltaCntNow + McuSendOffsetCopy[numCopy*3];
		CamData_Ptr->y = deltaY/deltaCnt*deltaCntNow + McuSendOffsetCopy[numCopy*3+1];
		CamData_Ptr->z = deltaZ/deltaCnt*deltaCntNow + McuSendOffsetCopy[numCopy*3+2];
		printf("CamData_Ptr->x:%d  y:%d   z:%d TimeStamp=%d\n",CamData_Ptr->x,CamData_Ptr->y,CamData_Ptr->z,CamData_Ptr->TimeStamp);//
	  //lDcuOutput.PutFrame(lFrame[0].mUMat);
		for (int i = 0;i < 4;i++)  //*******;************DY****************
		{
			if(lpGrabber->FramePush(lFrame[i]) != LIB_SUCCESS)
			{
				printf("lpGrabber->FramePush(lFrame[i]) %d\n", i);
				break;
			} // if push failed
		}
		scene.encoderData.x = CamData_Ptr->x;//关闭对应双线程
		scene.encoderData.y = CamData_Ptr->y;//关闭对应双线程
		scene.encoderData.theta = CamData_Ptr->z;//关闭对应双线程
		scene.setSteeringAngleBySteeringWheel(McuSendDada_Ptr->actual_steering_wheel_angle);
		scene.setGearStatus(McuSend_PcReadData.gear_status_actual);

		// scene.clearObstacleFrames();
		// scene.addObstacleFrame(const_value::FRONT_CAM,
		// 					   {
		// 						   cv::Point(100, 100),
		// 						   cv::Point(100, 200),
		// 						   cv::Point(200, 200),
		// 						   cv::Point(200, 100)
		// 					   });


		// 雷达信号
		scene.radarSignals[0] = McuSend_PcReadData.rader2_alarm_level * 20.0;
		scene.radarSignals[1] = McuSend_PcReadData.rader3_alarm_level * 20.0;
		scene.radarSignals[2] = McuSend_PcReadData.rader4_alarm_level * 20.0;
		scene.radarSignals[3] = McuSend_PcReadData.rader5_alarm_level * 20.0;
		scene.radarSignals[4] = McuSend_PcReadData.rader6_alarm_level * 20.0;
		scene.radarSignals[5] = McuSend_PcReadData.rader7_alarm_level * 20.0;

		scene.radarSignals[6] = McuSend_PcReadData.rader10_alarm_level * 20.0;
		scene.radarSignals[7] = McuSend_PcReadData.rader11_alarm_level * 20.0;
		scene.radarSignals[8] = McuSend_PcReadData.rader12_alarm_level * 20.0;
		scene.radarSignals[9] = McuSend_PcReadData.rader13_alarm_level * 20.0;
		scene.radarSignals[10] = McuSend_PcReadData.rader14_alarm_level * 20.0;
		scene.radarSignals[11] = McuSend_PcReadData.rader15_alarm_level * 20.0;

		scene.radarSignals[12] = McuSend_PcReadData.rader1_alarm_level * 20.0;
		scene.radarSignals[13] = McuSend_PcReadData.rader16_alarm_level * 20.0;
		scene.radarSignals[14] = McuSend_PcReadData.rader8_alarm_level * 20.0;
		scene.radarSignals[15] = McuSend_PcReadData.rader9_alarm_level * 20.0;


		// keyboard
        int times = 0;
#if 1
        readFromDevice();
	#if 1
        if((times = getKeyTimes(MY_KEY_UP)) > 0)
            scene.onButtonUp(times);
        else if((times = getKeyTimes(MY_KEY_DOWN)) > 0)
            scene.onButtonDown(times);
        else if((times = getKeyTimes(MY_KEY_LEFT)) > 0)
            scene.onButtonLeft(times);
        else if((times = getKeyTimes(MY_KEY_RIGHT)) > 0)
            scene.onButtonRight(times);
		//        else if((times = getKeyTimes(MY_KEY_MENU)) > 0)
		//            scene.onButtonOK();
	#else
		if ((times = getKeyTimes('w')) > 0)
		{
			scene.onButtonMoveUp(times);
		}
		else if ((times = getKeyTimes('s')) > 0)
		{
			scene.onButtonMoveDown(times);
		}
		else if ((times = getKeyTimes('a')) > 0)
		{
			scene.onButtonMoveLeft(times);
		}
		else if ((times = getKeyTimes('d')) > 0)
		{
			scene.onButtonMoveRight(times);
		}
	#endif
		else if ((times = getKeyTimes('n')) > 0)
		{
			scene.onButtonNextMode();
		}
		else if ((times = getKeyTimes('p')) > 0)
		{
			scene.onButtonPrevMode();
		}
		else if ((times = getKeyTimes('[')) > 0)
		{
			scene.onButtonTurnLeft(times);
		}
		else if ((times = getKeyTimes(']')) > 0)
		{
			scene.onButtonTurnRight(times);
		}
        else if((times = getKeyTimes(MY_KEY_RETURN)) > 0)
            break;
        for(int i = 0;i < 9;i++){
            if(times = getKeyTimes('0' + i) > 0){
                scene.onMessage('0' + i);
            }
        }
#endif


	// start rendering
	Timer<> timer;
	timer.start();
	static int lastFPS = 0;
	scene.renderToSreen();
	scene.renderTextBox(std::string(std::to_string(lastFPS) + " FPS").c_str(), 0, 0, 32);
	flushToScreen();
	timer.stop();
	lastFPS = 1000.0 / timer.count();
	std::cout << "rendering: " << timer.get() << std::endl;
	  // usleep(300000);
	  //printf("wwwwwww50\n");
	  ParkingManagemet();
	  //printf("&&&&&&&&&ParkPlaceNum =%d car_paring_status=%d\n",ParkingPlace_Ptr->ParkPlaceNum,McuSendDada_Ptr->car_paring_status);
  }
} // main()

void *GLTask(void *ptr1)
{
	#if 0
	FILE* fp = fopen("config.txt", "r");
	if (fp == NULL)
	{
		printf("could not found config.txt\n");
		//return -1;
	}
	fscanf(fp, "path: %s\n", path_pattern);
	fscanf(fp, "num_frames: %d\n", &num_frames);
	fclose(fp);
	printf("path = %s", path_pattern);


	// load display settings
	std::ifstream displaySettingFile("param/display.json");
	json displaySetting;
	displaySettingFile >> displaySetting;
	#endif

	//if(initialGL(displaySetting["screen"]["width"], displaySetting["screen"]["height"]) != 0)          //初始化opengl es环境
	if(initialGL(1920,1080) != 0)          //初始化opengl es环境
    {
         fprintf(stderr,"opengl init unsucess!\n");
         //return -1;
    }
    printf("initialGL()::\n");
	auto glVer = glGetString(GL_VERSION);
	if (!glVer)
	{
		printf("glGetString failed: %d\n", glGetError());
	}
	else
	{
		printf("OpenGL version: %s\n", glGetString(GL_VERSION));
	}

    //if(getCameraDevice().initCamera() != 0)   //初始化视频采集设备
    {
         printf("can not initialize the camera device！\n");
         //return -1;
    }
    printf("getCameraDevice()::\n");

    if(initPanoramaParam() != 0){
        printf("can not initialize the camera param");
        //return -1;
    }


    printf("initPanoramaParam() success!\n");
    keyboard_device::initDevice();                            //使用标准键盘设备
    printf("keyboard_device::initDevice() sucess!\n");
    CFinalScene scene;
	scene.setBirdEyeViewport(0,
							 0,
							 810,
							 1080);
	scene.setSingleViewport(810,
							270,
							1110,
							810);
	scene.set3DViewport(810,
							270,
							1110,
							810);
    scene.onStart();


	//std::cout << displaySetting << std::endl;
    //Calib_2D_Init();
    //Fbo_2D_Init();

    while(1)
    {
        int times = 0;
#if 1

        readFromDevice();
	#if 1
        if((times = getKeyTimes(MY_KEY_UP)) > 0)
            scene.onButtonUp(times);
        else if((times = getKeyTimes(MY_KEY_DOWN)) > 0)
            scene.onButtonDown(times);
        else if((times = getKeyTimes(MY_KEY_LEFT)) > 0)
            scene.onButtonLeft(times);
        else if((times = getKeyTimes(MY_KEY_RIGHT)) > 0)
            scene.onButtonRight(times);
		//        else if((times = getKeyTimes(MY_KEY_MENU)) > 0)
		//            scene.onButtonOK();
	#else
		if ((times = getKeyTimes('w')) > 0)
		{
			scene.onButtonMoveUp(times);
		}
		else if ((times = getKeyTimes('s')) > 0)
		{
			scene.onButtonMoveDown(times);
		}
		else if ((times = getKeyTimes('a')) > 0)
		{
			scene.onButtonMoveLeft(times);
		}
		else if ((times = getKeyTimes('d')) > 0)
		{
			scene.onButtonMoveRight(times);
		}
	#endif
		else if ((times = getKeyTimes('n')) > 0)
		{
			scene.onButtonNextMode();
		}
		else if ((times = getKeyTimes('p')) > 0)
		{
			scene.onButtonPrevMode();
		}
		else if ((times = getKeyTimes('[')) > 0)
		{
			scene.onButtonTurnLeft(times);
		}
		else if ((times = getKeyTimes(']')) > 0)
		{
			scene.onButtonTurnRight(times);
		}
        else if((times = getKeyTimes(MY_KEY_RETURN)) > 0)
            break;
        for(int i = 0;i < 9;i++){
            if(times = getKeyTimes('0' + i) > 0){
                scene.onMessage('0' + i);
            }
        }
#endif

        //printf("scene.renderToSreen()\n");
	Timer<> timer;
	timer.start();
	static int lastFPS = 0;
	scene.renderToSreen();
	scene.renderTextBox(std::string(std::to_string(lastFPS) + " FPS").c_str(), 0, 0, 32);
	flushToScreen();
	timer.stop();
	lastFPS = 1000.0 / timer.count();
	std::cout << "rendering: " << timer.get() << std::endl;
    }
	//程序执行完毕，做一些释放资源的操作
    scene.onStop();
    //getCameraDevice().deinitCamera();
    //keyboard_device::deinitDevice();
    deinitGL();
    return 0;
	#if 0
	readParam();
    if(initialGL() != 0)         // //��ʼ��opengl es����
    {
         fprintf(stderr,"opengl init unsucess!\n");
         return 0;
    }
    printf("initialGL()::\n");
   //if(getCameraDevice().initCamera() != 0)   //��ʼ����Ƶ�ɼ��豸
   // {
   //     printf("can not initialize the camera device\n");
   //     return 0;
   //}
   //printf("getCameraDevice()::\n");
    if(initPanoramaParam() != 0){
        printf("can not initialize the camera param");
        return 0;
    }
    printf("initPanoramaParam() success!\n");
    //keyboard_device::initDevice();                           //ʹ�ñ�׼�����豸
    //printf("keyboard_device::initDevice() sucess!\n");
    CFinalScene scene;
    scene.onStart();

    Calib_2D_Init();

    while(1)
    {
		#if 0
        int times = 0;
        readFromDevice();
		usleep(20000);
        if((times = getKeyTimes(MY_KEY_UP)) > 0)
            scene.onButtonUp(times);
        if((times = getKeyTimes(MY_KEY_DOWN)) > 0)
            scene.onButtonDown(times);
        if((times = getKeyTimes(MY_KEY_LEFT)) > 0)
            scene.onButtonLeft(times);
        if((times = getKeyTimes(MY_KEY_RIGHT)) > 0)
            scene.onButtonRight(times);
        if((times = getKeyTimes(MY_KEY_MENU)) > 0)
            scene.onButtonOK();
        if((times = getKeyTimes(MY_KEY_RETURN)) > 0)
            break;
        for(int i = 0;i < 9;i++){
            if(times = getKeyTimes('0' + i) > 0){
                scene.onMessage('0' + i);
            }
        }
		#endif
        //printf("scene.renderToSreen()\n");
        scene.renderToSreen();
        flushToScreen();

    }
	  //����ִ����ϣ���һЩ�ͷ���Դ�Ĳ���
    scene.onStop();
    getCameraDevice().deinitCamera();
    keyboard_device::deinitDevice();
    deinitGL();
    return 0;
	#endif
}

int32_t DdrBuffersAlloc(void** appFbsVirtual,
                        uint32_t*  apFbsPhysical,
                        size_t aMemSize,
                        uint32_t aBufferCnt
                       )
{
  int32_t lRet    = 0;

  // allocate buffers & get physical addresses
  for(uint32_t i = 0; i < aBufferCnt; i++)
  {
    appFbsVirtual[i] = OAL_MemoryAllocFlag(
                          aMemSize,
                          OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|
                          OAL_MEMORY_FLAG_CONTIGUOUS);

    if( appFbsVirtual[i] == NULL)
    {
      lRet = -1;
      break;
    }
    apFbsPhysical[i] = (uint32_t)(uintptr_t)OAL_MemoryReturnAddress(
                          appFbsVirtual[i],
                          ACCESS_PHY); // get physical address

    memset(appFbsVirtual[i], 0x00, aMemSize);
#if defined(__STANDALONE__) && defined(__ARMV8)
    //flush_dcache_range(appFbsVirtual[i], aMemSize);
#endif // #ifdef __STANDALONE__
  } // for all framebuffers

  if(lRet != 0)
  {
    DdrBuffersFree(appFbsVirtual, apFbsPhysical, aBufferCnt);
  }

  return lRet;
} // DdrBuffersAlloc()

//***************************************************************************

void DdrBuffersFree(void** appFbsVirtual,
                    uint32_t*  apFbsPhysical,
                    uint32_t aBufferCnt
                   )
{
  for(uint32_t i = 0; i < aBufferCnt; i++)
  {
    if(appFbsVirtual[i] != NULL)
    {
      OAL_MemoryFree(appFbsVirtual[i]);
    } // if buffer allocated

    appFbsVirtual[i]   = NULL;
    apFbsPhysical[i]   = 0;
  } // for all framebuffers
} // DdrBuffersFree()



//***************************************************************************

void SeqEventCallBack(uint32_t aEventType, void* apUserVal)
{
  static uint32_t slFrmCnt = 0;

  if(aEventType == SEQ_MSG_TYPE_FRAMEDONE)
  {
    //printf("Frame done message arrived #%u. User value %d\n",
    //       slFrmCnt++,
    //       *((int32_t*)apUserVal));
  } // if frame done arrived
} // SeqEventCallBack()

//***************************************************************************

#ifndef __STANDALONE__
void SigintHandler(int)
{
  sStop = true;
} // SigintHandler()

//***************************************************************************

int32_t SigintSetup()
{
  int32_t lRet = SEQ_LIB_SUCCESS;

  // prepare internal signal handler
  struct sigaction lSa;
  memset(&lSa, 0, sizeof(lSa));
  lSa.sa_handler = SigintHandler;

  if( sigaction(SIGINT, &lSa, NULL) != 0)
  {
    VDB_LOG_ERROR("Failed to register signal handler.\n");
    lRet = SEQ_LIB_FAILURE;
  } // if signal not registered

  return lRet;
} // SigintSetup()

//***************************************************************************
#endif // #ifndef __STANDALONE__

int Ipc_Create()
{
  int	 shmid1,shmid2,shmid3,shmid4,shmid5,shmid6,shmid7;
  //���������ڎ� ���൱�ڎ��Č����Č��������򎎜�
  shmid1 = shmget(0x2234, sizeof(CamData), IPC_CREAT | 0666);
  if (shmid1 == -1)
  {
	  perror("shmget 1 err");
	  return errno;
  }
  printf("shmid1:%d \n", shmid1);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  CamData_Ptr =(CamData *)shmat(shmid1, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (CamData_Ptr == (void *)-1 )
  {
	  perror("shmat 1 err");
	  return errno;
  }

  shmid2 = shmget(0x3234, sizeof(ParkingPlace), IPC_CREAT | 0666);
  if (shmid2 == -1)
  {
	  perror("shmget 2 err");
	  return errno;
  }
  printf("shmid2:%d \n", shmid2);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  ParkingPlace_Ptr =(ParkingPlace *)shmat(shmid2, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (ParkingPlace_Ptr == (void *)-1 )
  {
	  perror("shmat 2 err");
	  return errno;
  }

  shmid3 = shmget(0x4234, sizeof(McuSendDada), IPC_CREAT | 0666);
  if (shmid3 == -1)
  {
	  perror("shmget 3 err");
	  return errno;
  }
  printf("shmid3:%d \n", shmid3);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  McuSendDada_Ptr =(McuSendDada *)shmat(shmid3, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (McuSendDada_Ptr == (void *)-1 )
  {
	  perror("shmat 3 err");
	  return errno;
  }

  shmid4 = shmget(0x5234, sizeof(PCWRITEDATA), IPC_CREAT | 0666);
  if (shmid4 == -1)
  {
	  perror("shmget 4 err");
	  return errno;
  }
  printf("shmid4:%d \n", shmid4);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  PCWRITEDATA_Ptr =(PCWRITEDATA *)shmat(shmid4, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (PCWRITEDATA_Ptr == (void *)-1 )
  {
	  perror("shmat 4 err");
	  return errno;
  }

  shmid5 = shmget(0x6234, sizeof(BasicData), IPC_CREAT | 0666);
  if (shmid5 == -1)
  {
	  perror("shmget 5 err");
	  return errno;
  }
  printf("shmid5:%d \n", shmid5);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  BasicData_Ptr =(BasicData *)shmat(shmid5, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (BasicData_Ptr == (void *)-1 )
  {
	  perror("shmat 5 err");
	  return errno;
  }

  shmid6 = shmget(0x7234, sizeof(LanelineData), IPC_CREAT | 0666);
  if (shmid6 == -1)
  {
	  perror("shmget 6 err");
	  return errno;
  }
  printf("shmid6:%d \n", shmid6);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  LanelineData_Ptr =(LanelineData *)shmat(shmid6, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (LanelineData_Ptr == (void *)-1 )
  {
	  perror("shmat 6 err");
	  return errno;
  }

  shmid7 = shmget(0x8234, sizeof(ObstacleData), IPC_CREAT | 0666);
  if (shmid7 == -1)
  {
	  perror("shmget 7 err");
	  return errno;
  }
  printf("shmid7:%d \n", shmid7);
  //�������ڎ�����ӵ����̵�ַ�Ռ�
  ObstacleData_Ptr =(ObstacleData *)shmat(shmid7, NULL, 0);//�ڶ�������shmaddrΪNULL�������Զ�ѡ��һ����ַ
  if (ObstacleData_Ptr == (void *)-1 )
  {
	  perror("shmat 7 err");
	  return errno;
  }

}

  unsigned long long GetTime(void)
{
	Soc_tt=GetNowTimeUs();
	Soc_tt=Soc_tt/1000;
	return McuTimeData + (Soc_tt - Mcu_tt);

}

unsigned long long GetNowTimeUs(void)
 {
	 struct timeval tv;
	 gettimeofday(&tv, NULL);
	 return (unsigned long long)tv.tv_sec * 1000000 + tv.tv_usec;
}
void ParkingManagemet()
{

	//printf("parkingmanagemetn\n");
	int ii;
	static int ss,tt;
	static  int TimeCntTmp;
	static float ParkPlaceCopy[10*4*2];
	float prev_pts[8];         // 车位坐标 {(x1, y1), (x2, y2), (x3, y3), (x4, y4)}
	short int prev_coords[3];  // 车位坐标对应车身位置 X,Y,Z
	short int curr_coords[3];   // 当前车身位置 X,Y,Z
	float pts2veh[8];          // 车位坐标相对当前车身坐标
    /*****************************************************/
	//////////////////////////////////////////////////////////////
	//static GLint box_num; //2018.10.19
	//static GLfloat Ver_Texture_Pos_Parking[10*4*2] = {0}; //2018.10.19
	//static GLfloat Ver_Screen_Pos_Parking[10*4*2];

	if(ToMcuTargetData.Dnn_Scan_Ok_Flag == 0)
	{
		if( (abs(ParkingPlace_Ptr->ParkPlace[0]) + abs(ParkingPlace_Ptr->ParkPlace[2]) )>0)
			ToMcuTargetData.Dnn_Scan_Ok_Flag = 1;
	}
	if( (abs(McuSend_PcReadData.parking_rect_point0_x) + abs(McuSend_PcReadData.parking_rect_point1_x) )>0)
		Mcu_Send_Parking_Data_Fag = 1;
	else
		Mcu_Send_Parking_Data_Fag = 0;
	//printf("!!!!!!!!!!!!!Mcu_Send_Parking_Data_Fag=%d\n",Mcu_Send_Parking_Data_Fag);

	static int vv;
	if(vv == 0) //计算环视视野边界值 单位：mm
	{
		disFront = Rear_Axle_Center_y*Pixel_Ration_x;
		disBack  = (1080-Rear_Axle_Center_y)*Pixel_Ration_x;
		disLF    = Rear_Axle_Center_x*Pixel_Ration_y;
		printf("disFront=%dmm\n",disFront);
		printf("disBack=%dmm\n",disBack);
		printf("disLF=%dmm\n",disLF);
		vv = 1;
	}
	if(  (Car_Parking_Status >= 4) ||  (Car_Parking_Status == 0)) //停止或异常，以及0初始状态下，清除相关标志位
	{
		//memset(&ToMcuTargetData,0,sizeof(TARGETPLACEDATA));
		memset(ParkingPlace_Ptr,0,sizeof(ParkingPlace));
		Parking_Place_Mode_Select_OK = 0;
		Parking_Select_Num = 0;
		ss = 0;
		tt = 0;
		Total_Parking_Num =0;
		//box_num = 0;
		if(ToMcuTargetData.Dnn_Scan_Ok_Flag == 1)
			deleteLinklist(head);//删除链表
		memset(&ToMcuTargetData,0,sizeof(TARGETPLACEDATA));
		memset(PCWRITEDATA_Ptr,0,sizeof(PCWRITEDATA));
		memset(&AndroidReceive_SocWriteData,0,sizeof(SOCWRITEDATA));

		parking_place_detected_flag = 0;
	}

	if(Car_Parking_Status>0 && Car_Parking_Status<4)
	{
		float Parking_Rect_Point0_x;
		float Parking_Rect_Point0_y;
		float Parking_Rect_Point1_x;
		float Parking_Rect_Point1_y;
		float Parking_Rect_Point2_x;
		float Parking_Rect_Point2_y;
		float Parking_Rect_Point3_x;
		float Parking_Rect_Point3_y;
		//////////////////////////////////////////////////////////////////////////////
		//printf("ParkingPlace_Ptr->TimeStamp=%d\n",ParkingPlace_Ptr->TimeStamp);
		//printf("ParkingPlace_Ptr->ParkPlaceNum=%d\n",ParkingPlace_Ptr->ParkPlaceNum);

		/////////////////////////////////////////////////////////////////////////////////////////
		if(Car_Parking_Status == 1) //scan status
		{
			////////////////////////////////车位管理/////////////////////////////////////////////////
			#if 1
			memcpy(ParkingPlace_PtrCopy.ParkPlace,ParkingPlace_Ptr->ParkPlace,80*sizeof(float));
			ParkingPlace_PtrCopy.ParkPlaceNum = ParkingPlace_Ptr->ParkPlaceNum;
			ParkingPlace_PtrCopy.x = ParkingPlace_Ptr->x;
			ParkingPlace_PtrCopy.y = ParkingPlace_Ptr->y;
			ParkingPlace_PtrCopy.z = ParkingPlace_Ptr->z;
			memcpy(ParkingPlace_PtrCopy.ParkConfidence,ParkingPlace_Ptr->ParkConfidence,10*sizeof(int));
			ParkingPlace_PtrCopy.TimeStamp = ParkingPlace_Ptr->TimeStamp;

			if(ToMcuTargetData.Dnn_Scan_Ok_Flag == 1) //DNN第一次扫描到车位
			{
				if(tt==0)   //开始创建链表
				{
					LinkList *node, *end;//定义普通节点，尾部节点；
					head = (LinkList*)malloc(sizeof(LinkList));//链表首地址
					end = head;         //若是空链表则头尾节点一样
					printf("ParkingPlace_PtrCopy.ParkPlaceNum=%d\n",ParkingPlace_PtrCopy.ParkPlaceNum);
					for (int i = 0; i < ParkingPlace_PtrCopy.ParkPlaceNum; i++) //根据扫描到的车位数，创建节点
					{
						node = (LinkList*)malloc(sizeof(LinkList));
						for(int j=0;j<8;j++)
							node->ParkPlace[j] = ParkingPlace_PtrCopy.ParkPlace[i*8+j];
						node->x = ParkingPlace_PtrCopy.x;
						node->y = ParkingPlace_PtrCopy.y;
						node->z = ParkingPlace_PtrCopy.z;
						node->park_confidence = ParkingPlace_PtrCopy.ParkConfidence[i];
						end->next = node;
						end = node;
					}
					end->next = NULL;//结束创建
					//printf("\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
					printf("creat link list success!\n");
					//printf("\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
					tt = 1;
					TimeCntTmp1 = ParkingPlace_PtrCopy.TimeStamp;
				}
			}
			#endif
			#if 1
			//printf("ParkingPlace_PtrCopy.ParkPlaceNum=%d\n",ParkingPlace_PtrCopy.ParkPlaceNum);
			if(TimeCntTmp1 != ParkingPlace_PtrCopy.TimeStamp) //若DNN有新扫描到的车位，每个车位都和己有车位进行比对
			{
				for (int i = 0; i < ParkingPlace_PtrCopy.ParkPlaceNum; i++)
				{
					LinkList *h = head;
					int listNum =0;
					int listTotal = 0;

					while (h->next != NULL)
					{
						listTotal++;
						h = h->next;

						prev_coords[0] = h->x;
						prev_coords[1] = h->y;
						prev_coords[2] = h->z;
						curr_coords[0] = ParkingPlace_PtrCopy.x;
						curr_coords[1] = ParkingPlace_PtrCopy.y;
						curr_coords[2] = ParkingPlace_PtrCopy.z;

						memcpy(prev_pts,h->ParkPlace,8*sizeof(float));
						coord_pts2veh(curr_coords, prev_coords, prev_pts, pts2veh); //物理坐标到绝对坐标转换
						//计算新车位和己有车位的中点距离，小于1200mm为重合车位，大于1200mm为新车位，创建新节点并加入链表
						float new_mid_point_x; //新车位中点X坐标
						float new_mid_point_y; //新车位中点y坐标
						float old_mid_point_x; //已有车位中点X坐标
						float old_mid_point_y; //已有车位中点y坐标
						float mid_point_distance;//两个车位中点之间的距离
						//***********计算新车位和己有车位的中点坐标
						new_mid_point_x = (ParkingPlace_PtrCopy.ParkPlace[i*8]+ParkingPlace_PtrCopy.ParkPlace[i*8+2]+ParkingPlace_PtrCopy.ParkPlace[i*8+4]+ParkingPlace_PtrCopy.ParkPlace[i*8+6])/4;
						new_mid_point_y = (ParkingPlace_PtrCopy.ParkPlace[i*8+1]+ParkingPlace_PtrCopy.ParkPlace[i*8+3]+ParkingPlace_PtrCopy.ParkPlace[i*8+5]+ParkingPlace_PtrCopy.ParkPlace[i*8+7])/4;
						old_mid_point_x = (pts2veh[0]+pts2veh[2]+pts2veh[4]+pts2veh[6])/4;
						old_mid_point_y = (pts2veh[1]+pts2veh[3]+pts2veh[5]+pts2veh[7])/4;

						//**********计算两个中点之间的距离**********如果距离大于1200mm，则认为是新车位，否则是重合车位**************
						mid_point_distance = sqrt( (new_mid_point_x-old_mid_point_x)*(new_mid_point_x-old_mid_point_x) +(new_mid_point_y-old_mid_point_y)*(new_mid_point_y-old_mid_point_y) );
						//if( (abs(pts2veh[0]-ParkingPlace_PtrCopy.ParkPlace[i*8]) >= 500) || (abs(pts2veh[1]-ParkingPlace_PtrCopy.ParkPlace[i*8+1]) >= 500) ) //新车位
						if(mid_point_distance>=1200)
						{
							listNum++;
						}
						else  //是重合车位，比较哪个离左/右摄像头更近
						{
							//int disLRCam = 1930; //左、右后视镜摄像头到后轴中心X方向上的距离 mm

							int deltOld = abs( ( h->ParkPlace[0] + h->ParkPlace[2] )/2 -disLRCam ); //计算当前车位P0x、P1x中点到后视镜摄像头距离
							int deltNew = abs( (ParkingPlace_PtrCopy.ParkPlace[i*8] + ParkingPlace_PtrCopy.ParkPlace[i*8+2] )/2 -disLRCam); //计算新车位P0x、P1x中点到后视镜摄像头距离
							//printf("deltOld =%d  deltNew=%d \n",deltOld,deltNew);
							if( deltOld > deltNew) //若新车位比旧车位距离更靠近后视镜，则更新车位信息，否则保持旧车位信息
							{
								for(int j=0;j<8;j++)
									h->ParkPlace[j] = ParkingPlace_PtrCopy.ParkPlace[i*8+j];
								h->x = ParkingPlace_PtrCopy.x;
								h->y = ParkingPlace_PtrCopy.y;
								h->z = ParkingPlace_PtrCopy.z;
								h->park_confidence = ParkingPlace_PtrCopy.ParkConfidence[i];
								//printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$new parkingplace replace\n");
							}
						}
					}
					printf("listTotal=%d  listNum =%d\n,",listTotal,listNum);
					if(listNum == listTotal)  //若没有重合车位，则添加新车位
					{
						LinkList * newnode = (LinkList*)malloc(sizeof(LinkList));
						for(int j=0;j<8;j++)
							newnode->ParkPlace[j] = ParkingPlace_PtrCopy.ParkPlace[i*8+j];
						newnode->x = ParkingPlace_PtrCopy.x;
						newnode->y = ParkingPlace_PtrCopy.y;
						newnode->z = ParkingPlace_PtrCopy.z;
						newnode->park_confidence = ParkingPlace_PtrCopy.ParkConfidence[i];
						newnode->next = NULL;
						//printf("add new node start\n");
						insert_trail(head,newnode);
						//printf("add new node end\n");
					}
				}
				TimeCntTmp1  = ParkingPlace_PtrCopy.TimeStamp;
			}
			#endif
			/////////////////////////////////////////////////////////////////////////////////////////
		    if(Mcu_Send_Parking_Data_Fag ==1 && ToMcuTargetData.Dnn_Scan_Ok_Flag == 0) //DNN扫到车位之前。超声波先扫到车位，则先显示超声波扫到的车位
		    {
		    	//memset(Ver_Pos_Parking,0,sizeof(GLfloat)*10*4*3);

				//Parking_Select_Num = 1;
				Total_Parking_Num =1;
				//box_num = Total_Parking_Num;
				validNode = 1;

				//Parking_Rect_Point0_x = Rear_Axle_Center_x-(McuSend_PcReadData.parking_rect_point0_y/Pixel_Ration_y);
		   		//Parking_Rect_Point0_y = Rear_Axle_Center_y-(McuSend_PcReadData.parking_rect_point0_x/Pixel_Ration_x);

		    	//Parking_Rect_Point1_x = Rear_Axle_Center_x-(McuSend_PcReadData.parking_rect_point1_y/Pixel_Ration_y);
		   		//Parking_Rect_Point1_y = Rear_Axle_Center_y-(McuSend_PcReadData.parking_rect_point1_x/Pixel_Ration_x);

		    	//Parking_Rect_Point2_x = Rear_Axle_Center_x-(McuSend_PcReadData.parking_rect_point2_y/Pixel_Ration_y);
		    	//Parking_Rect_Point2_y = Rear_Axle_Center_y-(McuSend_PcReadData.parking_rect_point2_x/Pixel_Ration_x);

		    	//Parking_Rect_Point3_x = Rear_Axle_Center_x-(McuSend_PcReadData.parking_rect_point3_y/Pixel_Ration_y);
		    	//Parking_Rect_Point3_y = Rear_Axle_Center_y-(McuSend_PcReadData.parking_rect_point3_x/Pixel_Ration_x);

				pts2veh[0] = McuSend_PcReadData.parking_rect_point0_x;
				pts2veh[1] = McuSend_PcReadData.parking_rect_point0_y;
				pts2veh[2] = McuSend_PcReadData.parking_rect_point1_x;
				pts2veh[3] = McuSend_PcReadData.parking_rect_point1_y;
				pts2veh[4] = McuSend_PcReadData.parking_rect_point2_x;
				pts2veh[5] = McuSend_PcReadData.parking_rect_point2_y;
				pts2veh[6] = McuSend_PcReadData.parking_rect_point3_x;
				pts2veh[7] = McuSend_PcReadData.parking_rect_point3_y;

				printf("pts2veh:%f %f , %f %f, %f %f,%f %f\n",pts2veh[0],pts2veh[1],pts2veh[2],pts2veh[3],pts2veh[4],pts2veh[5],pts2veh[6],pts2veh[7]);
				/////////////////////////////to screen display////////////////////////////////////////////////////
				int validPoingFlag = 0;  //车位坐标在视野范围内标志，1：在视野范围内，0：超出视野范围
				for(int i=0;i<4;i++)	//判断是否在视野范围内
				{
					if( (pts2veh[i*2]<disFront) && (pts2veh[i*2]> -disBack))
					{
						if (abs(pts2veh[i*2+1])<disLF)
							validPoingFlag = 1;
					}
				}
				if(validPoingFlag == 1)  //在视野范围内，送显示
				{
					memcpy(ParkPlaceCopy,pts2veh,8*sizeof(float));
					parking_place_detected_num = 1;
					parking_place_detected_flag = 1;

					AndroidReceive_SocWriteData.parking_1_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[1]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_1_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[0]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_1_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[3]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_1_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[2]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_1_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[5]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_1_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[4]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_1_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[7]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_1_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[6]/Pixel_Ration_x);

					printf("ParkPlaceCopy:%f %f , %f %f, %f %f,%f %f\n",ParkPlaceCopy[0],ParkPlaceCopy[1],ParkPlaceCopy[2],ParkPlaceCopy[3],ParkPlaceCopy[4],ParkPlaceCopy[5],ParkPlaceCopy[6],ParkPlaceCopy[7]);
					printf("radar chewei 1 P0:%d %d \n",AndroidReceive_SocWriteData.parking_1_point0_x,AndroidReceive_SocWriteData.parking_1_point0_y);
					printf("radar chewei 1 P1:%d %d \n",AndroidReceive_SocWriteData.parking_1_point1_x,AndroidReceive_SocWriteData.parking_1_point1_y);
					printf("radar chewei 1 P2:%d %d \n",AndroidReceive_SocWriteData.parking_1_point2_x,AndroidReceive_SocWriteData.parking_1_point2_y);
					printf("radar chewei 1 P3:%d %d \n",AndroidReceive_SocWriteData.parking_1_point3_x,AndroidReceive_SocWriteData.parking_1_point3_y);
					/*

					int k=0;
					int disPoint01= (ParkPlaceCopy[0]-ParkPlaceCopy[2])*(ParkPlaceCopy[0]-ParkPlaceCopy[2])+(ParkPlaceCopy[1]-ParkPlaceCopy[3])*(ParkPlaceCopy[1]-ParkPlaceCopy[3]);
					int disPoint03= (ParkPlaceCopy[0]-ParkPlaceCopy[6])*(ParkPlaceCopy[0]-ParkPlaceCopy[6])+(ParkPlaceCopy[1]-ParkPlaceCopy[7])*(ParkPlaceCopy[1]-ParkPlaceCopy[7]);
					if(disPoint01 <= disPoint03)
					{
						AndroidReceive_SocWriteData.parking_1_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[1]/Pixel_Ration_y);
						AndroidReceive_SocWriteData.parking_1_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[0]/Pixel_Ration_x);
						AndroidReceive_SocWriteData.parking_1_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[3]/Pixel_Ration_y);
						AndroidReceive_SocWriteData.parking_1_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[2]/Pixel_Ration_x);
						AndroidReceive_SocWriteData.parking_1_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[5]/Pixel_Ration_y);
						AndroidReceive_SocWriteData.parking_1_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[4]/Pixel_Ration_x);
						AndroidReceive_SocWriteData.parking_1_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[7]/Pixel_Ration_y);
						AndroidReceive_SocWriteData.parking_1_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[6]/Pixel_Ration_x);
					}
					else
					{
						AndroidReceive_SocWriteData.parking_1_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[7]/Pixel_Ration_y);
						AndroidReceive_SocWriteData.parking_1_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[6]/Pixel_Ration_x);
						AndroidReceive_SocWriteData.parking_1_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[1]/Pixel_Ration_y);
						AndroidReceive_SocWriteData.parking_1_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[0]/Pixel_Ration_x);
						AndroidReceive_SocWriteData.parking_1_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[3]/Pixel_Ration_y);
						AndroidReceive_SocWriteData.parking_1_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[2]/Pixel_Ration_x);
						AndroidReceive_SocWriteData.parking_1_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[5]/Pixel_Ration_y);
						AndroidReceive_SocWriteData.parking_1_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[4]/Pixel_Ration_x);
					}
					*/

				}
				else	//出视野，清除标志位
				{
					parking_place_detected_num = 0;
					parking_place_detected_flag = 0;
				}

				if(Parking_Place_Mode_Select_OK == 1)	//选择好车位，确定
				{
					ToMcuTargetData.Customer_Select_Ok_Flag = 1;
				}
				//////////////////////////////////////////////////////////////////////////////////////////////////
		    }
			else if(ToMcuTargetData.Dnn_Scan_Ok_Flag == 1)   //若DNN扫到车位，则显示DNN扫到的车位
			{
				////////////////////////////检测链表中的车位在当前时刻是否在UI显示视野范围内//////////////////////////
				#if 1
				//printf("check Node\n");
				LinkList *h1 = head;
				int nodeTotal=0,rr=0;
				int invalidNode[30]; //记录链表中超出视野范围的节点序号
				//int validNode = 0;   //当前链表中在视野范围内的节点数（即车位数）
				memset(invalidNode,0,30*sizeof(int));
				while (h1->next != NULL)
				{
					nodeTotal++;
					h1 = h1->next;
					prev_coords[0] = h1->x;
					prev_coords[1] = h1->y;
					prev_coords[2] = h1->z;
					curr_coords[0] = McuSend_PcReadData.TimeStampex[0];
					curr_coords[1] = McuSend_PcReadData.TimeStampex[1];
					curr_coords[2] = McuSend_PcReadData.TimeStampex[2];

					memcpy(prev_pts,h1->ParkPlace,8*sizeof(float));
					coord_pts2veh(curr_coords, prev_coords, prev_pts, pts2veh); //物理坐标到绝对坐标转换

					int validPoingFlag = 0;  //车位坐标在视野范围内标志，1：在视野范围内，0：超出视野范围

					for(int i=0;i<4;i++)
					{
						if( (pts2veh[i*2]<disFront) && (pts2veh[i*2]> -disBack))
						{
							if (abs(pts2veh[i*2+1])<disLF)
								validPoingFlag = 1;
						}
					}

					if( validPoingFlag == 0 ) //超出视野范围
					{
						invalidNode[rr]=nodeTotal;
						rr++;
						//printf("\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@111!!!!!!!!!!!!!!!rr=%d\n",rr);
					}
					//printf("\n&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
				}
				for(int i=0;i<rr;i++)
				{
					if(invalidNode[i]>0)
					{
						printf("invalidNode[%d]=%d\n",i,invalidNode[i]);
						deleteNode(head,invalidNode[i]-i*1);
					}
				}
				validNode = nodeTotal-rr;	//链表中符合要求的车位数
				printf("****Total valid parking place :%d\n",validNode);
				#endif
				///////////////////////////////////////////////////////////////////////////////
				if(Car_Speed_Flag >0)
					Parking_Select_Num = 0;
				Total_Parking_Num = validNode;
				//box_num = Total_Parking_Num;

				TimeCntTmp     = ParkingPlace_Ptr->TimeStamp;

				LinkList *h2 = head;
				int nn = 0;
				int nn_left = 0;
				int left_parking_id_record[10] = {0};
				int nn_right = 0;
				int right_parking_id_record[10] = {0};
				memset(&ParkPlaceCopy,0,10*4*2*sizeof(float));
				parking_place_detected_flag = 0;
				while (h2->next != NULL)
				{
					nn++;
					h2 = h2->next;
					prev_coords[0] = h2->x;
					prev_coords[1] = h2->y;
					prev_coords[2] = h2->z;
					curr_coords[0] = McuSend_PcReadData.TimeStampex[0];
					curr_coords[1] = McuSend_PcReadData.TimeStampex[1];
					curr_coords[2] = McuSend_PcReadData.TimeStampex[2];

					if(nn<10)   //目前显示最多支持9个车位，超过9个的不再显示
					{
						memcpy(prev_pts,h2->ParkPlace,8*sizeof(float));
						coord_pts2veh(curr_coords, prev_coords, prev_pts, pts2veh); //物理坐标到绝对坐标转换
						//memcpy(&ParkPlaceCopy[(nn-1)*8],pts2veh,8*sizeof(float));
						//printf("pts2veh %f,%f,  %f,%f,  %f,%f,  %f,%f\n",pts2veh[0],pts2veh[1],pts2veh[2],pts2veh[3],pts2veh[4],pts2veh[5],pts2veh[6],pts2veh[7]);
						//parking_place_detected_flag = 1;
						//parking_place_detected_num = nn;
						//printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@parking_place_detected_num=%d\n",parking_place_detected_num);

						/////////////////////////////////////left or right////////////////////////////////
						///*
						if(parking_in_scan_direction == 1)
						{
							if( pts2veh[1]>=900 && pts2veh[3]>=900 && pts2veh[5]>=900 && pts2veh[7]>=900) //是否左侧车位？
							{
								//int nn_left;
								//int left_parking_id_record[10];
								nn_left++;
								memcpy(&ParkPlaceCopy[(nn_left-1)*8],pts2veh,8*sizeof(float));
								left_parking_id_record[nn_left] = nn;
								parking_place_detected_flag = 1;
							}
						}
						else if(parking_in_scan_direction == 2)
						{
							 if( pts2veh[1]<=-900 && pts2veh[3]<=-900 && pts2veh[5]<=-900 && pts2veh[7]<=-900) //是否右侧车位？
							{
								//int nn_right;
								//int right_parking_id_record[10];
								nn_right++;
								memcpy(&ParkPlaceCopy[(nn_right-1)*8],pts2veh,8*sizeof(float));
								right_parking_id_record[nn_right] = nn;
								parking_place_detected_flag = 1;
							}
						}
						//*/
						/////////////////////////////////////////////////////////////////////////////////////
					}
					else
						break;
				}
				///*
				if(parking_place_detected_flag == 1)
				{
					if(parking_in_scan_direction == 1) //left scan
						parking_place_detected_num = nn_left;
					else if(parking_in_scan_direction == 2) //right scan
						parking_place_detected_num = nn_right;
					else
						parking_place_detected_num = 0;
					printf("direction=%d,parking_place_detected_num=%d\n",parking_in_scan_direction,parking_place_detected_num);
				}
				//*/
				//printf("prev_pts %f,%f,  %f,%f,  %f,%f,  %f,%f\n",prev_pts[0],prev_pts[1],prev_pts[2],prev_pts[3],prev_pts[4],prev_pts[5],prev_pts[6],prev_pts[7]);
				int k=0;
				int disPoint01= (ParkPlaceCopy[0]-ParkPlaceCopy[2])*(ParkPlaceCopy[0]-ParkPlaceCopy[2])+(ParkPlaceCopy[1]-ParkPlaceCopy[3])*(ParkPlaceCopy[1]-ParkPlaceCopy[3]);
				int disPoint03= (ParkPlaceCopy[0]-ParkPlaceCopy[6])*(ParkPlaceCopy[0]-ParkPlaceCopy[6])+(ParkPlaceCopy[1]-ParkPlaceCopy[7])*(ParkPlaceCopy[1]-ParkPlaceCopy[7]);
				if(disPoint01 <= disPoint03)
				{
					AndroidReceive_SocWriteData.parking_1_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[1]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_1_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[0]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_1_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[3]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_1_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[2]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_1_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[5]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_1_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[4]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_1_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[7]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_1_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[6]/Pixel_Ration_x);
				}
				else
				{
					AndroidReceive_SocWriteData.parking_1_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[7]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_1_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[6]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_1_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[1]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_1_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[0]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_1_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[3]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_1_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[2]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_1_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[5]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_1_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[4]/Pixel_Ration_x);
				}
				//printf("ParkPlaceCopy k=%d  P0:%f,%f, P1: %f,%f,\n",k,ParkPlaceCopy[k+0],ParkPlaceCopy[k+1],ParkPlaceCopy[k+2],ParkPlaceCopy[k+3]);
				//printf("ParkPlaceCopy k=%d  P2:%f,%f,  P3:%f,%f,\n",k,ParkPlaceCopy[k+4],ParkPlaceCopy[k+5],ParkPlaceCopy[k+6],ParkPlaceCopy[k+7]);
				///*
				k = 1*8;
				disPoint01= (ParkPlaceCopy[k+0]-ParkPlaceCopy[k+2])*(ParkPlaceCopy[k+0]-ParkPlaceCopy[k+2])+(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+3])*(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+3]);
				disPoint03= (ParkPlaceCopy[k+0]-ParkPlaceCopy[k+6])*(ParkPlaceCopy[k+0]-ParkPlaceCopy[k+6])+(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+7])*(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+7]);
				if(disPoint01 <= disPoint03)
				{
					AndroidReceive_SocWriteData.parking_2_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+1]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_2_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+0]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_2_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+3]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_2_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+2]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_2_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+5]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_2_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+4]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_2_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+7]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_2_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+6]/Pixel_Ration_x);
				}
				else
				{
					AndroidReceive_SocWriteData.parking_2_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+7]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_2_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+6]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_2_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+1]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_2_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+0]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_2_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+3]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_2_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+2]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_2_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+5]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_2_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+4]/Pixel_Ration_x);
				}
				//printf("ParkPlaceCopy k=%d  P0:%f,%f, P1: %f,%f,\n",k,ParkPlaceCopy[k+0],ParkPlaceCopy[k+1],ParkPlaceCopy[k+2],ParkPlaceCopy[k+3]);
				//printf("ParkPlaceCopy k=%d  P2:%f,%f,  P3:%f,%f,\n",k,ParkPlaceCopy[k+4],ParkPlaceCopy[k+5],ParkPlaceCopy[k+6],ParkPlaceCopy[k+7]);

				k = 2*8;
				disPoint01= (ParkPlaceCopy[k+0]-ParkPlaceCopy[k+2])*(ParkPlaceCopy[k+0]-ParkPlaceCopy[k+2])+(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+3])*(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+3]);
				disPoint03= (ParkPlaceCopy[k+0]-ParkPlaceCopy[k+6])*(ParkPlaceCopy[k+0]-ParkPlaceCopy[k+6])+(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+7])*(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+7]);
				if(disPoint01 <= disPoint03)
				{
					AndroidReceive_SocWriteData.parking_3_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+1]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_3_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+0]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_3_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+3]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_3_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+2]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_3_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+5]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_3_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+4]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_3_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+7]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_3_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+6]/Pixel_Ration_x);
				}
				else
				{
					AndroidReceive_SocWriteData.parking_3_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+7]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_3_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+6]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_3_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+1]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_3_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+0]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_3_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+3]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_3_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+2]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_3_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+5]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_3_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+4]/Pixel_Ration_x);
				}
				//printf("ParkPlaceCopy k=%d  P0:%f,%f, P1: %f,%f,\n",k,ParkPlaceCopy[k+0],ParkPlaceCopy[k+1],ParkPlaceCopy[k+2],ParkPlaceCopy[k+3]);
				//printf("ParkPlaceCopy k=%d  P2:%f,%f,  P3:%f,%f,\n",k,ParkPlaceCopy[k+4],ParkPlaceCopy[k+5],ParkPlaceCopy[k+6],ParkPlaceCopy[k+7]);
				k = 3*8;
				disPoint01= (ParkPlaceCopy[k+0]-ParkPlaceCopy[k+2])*(ParkPlaceCopy[k+0]-ParkPlaceCopy[k+2])+(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+3])*(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+3]);
				disPoint03= (ParkPlaceCopy[k+0]-ParkPlaceCopy[k+6])*(ParkPlaceCopy[k+0]-ParkPlaceCopy[k+6])+(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+7])*(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+7]);
				if(disPoint01 <= disPoint03)
				{
					AndroidReceive_SocWriteData.parking_4_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+1]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_4_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+0]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_4_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+3]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_4_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+2]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_4_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+5]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_4_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+4]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_4_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+7]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_4_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+6]/Pixel_Ration_x);
				}
				else
				{
					AndroidReceive_SocWriteData.parking_4_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+7]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_4_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+6]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_4_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+1]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_4_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+0]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_4_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+3]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_4_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+2]/Pixel_Ration_x);
					AndroidReceive_SocWriteData.parking_4_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+5]/Pixel_Ration_y);
					AndroidReceive_SocWriteData.parking_4_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+4]/Pixel_Ration_x);
				}
				//printf("ParkPlaceCopy k=%d  P0:%f,%f, P1: %f,%f,\n",k,ParkPlaceCopy[k+0],ParkPlaceCopy[k+1],ParkPlaceCopy[k+2],ParkPlaceCopy[k+3]);
				//printf("ParkPlaceCopy k=%d  P2:%f,%f,  P3:%f,%f,\n",k,ParkPlaceCopy[k+4],ParkPlaceCopy[k+5],ParkPlaceCopy[k+6],ParkPlaceCopy[k+7]);
				//*/
				//printf("p0:%f,%f \n",ParkPlaceCopy[0],ParkPlaceCopy[1]);
				//printf("p1:%f,%f \n",ParkPlaceCopy[8],ParkPlaceCopy[9]);
				//printf("p2:%f,%f \n",ParkPlaceCopy[16],ParkPlaceCopy[17]);
				//printf("p3:%f,%f \n",ParkPlaceCopy[24],ParkPlaceCopy[25]);
				//for(ii=0;ii<box_num*8;ii=ii+2)
				//{
				//	Ver_Screen_Pos_Parking[ii]   = Rear_Axle_Center_x-(ParkPlaceCopy[ii+1]/Pixel_Ration);
				//	Ver_Screen_Pos_Parking[ii+1] = Rear_Axle_Center_y-(ParkPlaceCopy[ii]/Pixel_Ration);
				//}
				#if 1
				if(Car_Speed_Flag==0)  //停车静止状态下，按键选择目标车位，并下发MCU
				{
					if(Parking_Place_Mode_Select_OK == 1) //目标车位己确定，发送给MCU
					{
						//Parking_Select_Num=AndroidSend_SocReadData.auto_parking_in_stallID;
						/////////////////////////////////////////////////////////////////////////////
						///*
						if(parking_in_scan_direction == 1)
							Parking_Select_Num = left_parking_id_record[AndroidSend_SocReadData.auto_parking_in_stallID];
						else if(parking_in_scan_direction == 2)
							Parking_Select_Num = right_parking_id_record[AndroidSend_SocReadData.auto_parking_in_stallID];
						//	*/
						/////////////////////////////////////////////////////////////////////////////
						if(Parking_Select_Num>0){
						if(ss==0)
						{
							int n1=0;
							LinkList *h3 = head;
							while (h3->next != NULL)
							{
								n1++;
								h3 = h3->next;

								//Parking_Select_Num=AndroidSend_SocReadData.auto_parking_in_stallID;
								printf("*******&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&***********************************selected %d parking********************\n",Parking_Select_Num);
								printf("*******&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&7***********************************selected %d parking********************\n",Parking_Select_Num);
								printf("*******&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&***********************************selected %d parking********************\n",Parking_Select_Num);
								printf("*******&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&7***********************************selected %d parking********************\n",Parking_Select_Num);
							//if(Parking_Select_Num>0){
								if( n1== Parking_Select_Num)   //选中车位
								{
									memcpy(ParkingPlace_Ptr->TargetPlace,h3->ParkPlace,8*sizeof(float));
									ParkingPlace_Ptr->Targetx = h3->x;
									ParkingPlace_Ptr->Targety = h3->y;
									ParkingPlace_Ptr->Targetz = h3->z;
									ParkingPlace_Ptr->TargetConfidence = h3->park_confidence;
									ParkingPlace_Ptr->Target_TimeStamp = ParkingPlace_Ptr->TimeStamp;

									ToMcuTargetData.Target_CarPark_P0Point[0] = ParkingPlace_Ptr->TargetPlace[0];
									ToMcuTargetData.Target_CarPark_P0Point[1] = ParkingPlace_Ptr->TargetPlace[1];
									ToMcuTargetData.Target_CarPark_P1Point[0] = ParkingPlace_Ptr->TargetPlace[2];
									ToMcuTargetData.Target_CarPark_P1Point[1] = ParkingPlace_Ptr->TargetPlace[3];
									ToMcuTargetData.Target_CarPark_P2Point[0] = ParkingPlace_Ptr->TargetPlace[4];
									ToMcuTargetData.Target_CarPark_P2Point[1] = ParkingPlace_Ptr->TargetPlace[5];
									ToMcuTargetData.Target_CarPark_P3Point[0] = ParkingPlace_Ptr->TargetPlace[6];
									ToMcuTargetData.Target_CarPark_P3Point[1] = ParkingPlace_Ptr->TargetPlace[7];

									ToMcuTargetData.TargetConfidence = ParkingPlace_Ptr->TargetConfidence;
									ToMcuTargetData.Targetx = ParkingPlace_Ptr->Targetx;
									ToMcuTargetData.Targety = ParkingPlace_Ptr->Targety;
									ToMcuTargetData.Targetz = ParkingPlace_Ptr->Targetz;
									ToMcuTargetData.Customer_Select_Ok_Flag = 1;

									printf("Target_ParkingPlace_Ptr p0:%f %f \n",ParkingPlace_Ptr->TargetPlace[0],ParkingPlace_Ptr->TargetPlace[1]);
									printf("Target_ParkingPlace_Ptr p1:%f %f \n",ParkingPlace_Ptr->TargetPlace[2],ParkingPlace_Ptr->TargetPlace[3]);
									printf("Target_ParkingPlace_Ptr p2:%f %f \n",ParkingPlace_Ptr->TargetPlace[4],ParkingPlace_Ptr->TargetPlace[5]);
									printf("Target_ParkingPlace_Ptr p3:%f %f \n",ParkingPlace_Ptr->TargetPlace[6],ParkingPlace_Ptr->TargetPlace[7]);
									printf("ParkingPlace_Ptr->TimeStamp = %d\n",ParkingPlace_Ptr->TimeStamp);
									printf("Target_ x:%d y:%d  z:%d  TimeStamp=%d\n",ParkingPlace_Ptr->Targetx,ParkingPlace_Ptr->Targety,ParkingPlace_Ptr->Targetz,ParkingPlace_Ptr->Target_TimeStamp);
									break;
								}
							  //}
							}
							ss = 1;
						}

							}
						/*
						ToMcuTargetData.Target_CarPark_P0Point[0] = ParkingPlace_Ptr->TargetPlace[0];
						ToMcuTargetData.Target_CarPark_P0Point[1] = ParkingPlace_Ptr->TargetPlace[1];
						ToMcuTargetData.Target_CarPark_P1Point[0] = ParkingPlace_Ptr->TargetPlace[2];
						ToMcuTargetData.Target_CarPark_P1Point[1] = ParkingPlace_Ptr->TargetPlace[3];
						ToMcuTargetData.Target_CarPark_P2Point[0] = ParkingPlace_Ptr->TargetPlace[4];
						ToMcuTargetData.Target_CarPark_P2Point[1] = ParkingPlace_Ptr->TargetPlace[5];
						ToMcuTargetData.Target_CarPark_P3Point[0] = ParkingPlace_Ptr->TargetPlace[6];
						ToMcuTargetData.Target_CarPark_P3Point[1] = ParkingPlace_Ptr->TargetPlace[7];

						ToMcuTargetData.TargetConfidence = ParkingPlace_Ptr->TargetConfidence;
						ToMcuTargetData.Targetx = ParkingPlace_Ptr->Targetx;
						ToMcuTargetData.Targety = ParkingPlace_Ptr->Targety;
						ToMcuTargetData.Targetz = ParkingPlace_Ptr->Targetz;
						ToMcuTargetData.Customer_Select_Ok_Flag = 1;
						*/
					}
				}
				#endif
			}
		}
		else //进入 READY 和RUN status，实时显示MCU上传的车位信息
		{
			//memset(Ver_Pos_Parking,0,sizeof(GLfloat)*10*4*3);
			//box_num = 1;
			Parking_Select_Num = 1;

			#if 0
			Parking_Rect_Point0_x = Rear_Axle_Center_x2-(McuSend_PcReadData.parking_rect_point0_y/Pixel_Ration2);
	   		Parking_Rect_Point0_y = Rear_Axle_Center_y2-(McuSend_PcReadData.parking_rect_point0_x/Pixel_Ration2);

	    	Parking_Rect_Point1_x = Rear_Axle_Center_x2-(McuSend_PcReadData.parking_rect_point1_y/Pixel_Ration2);
	   		Parking_Rect_Point1_y = Rear_Axle_Center_y2-(McuSend_PcReadData.parking_rect_point1_x/Pixel_Ration2);

	    	Parking_Rect_Point2_x = Rear_Axle_Center_x2-(McuSend_PcReadData.parking_rect_point2_y/Pixel_Ration2);
	    	Parking_Rect_Point2_y = Rear_Axle_Center_y2-(McuSend_PcReadData.parking_rect_point2_x/Pixel_Ration2);

	    	Parking_Rect_Point3_x = Rear_Axle_Center_x2-(McuSend_PcReadData.parking_rect_point3_y/Pixel_Ration2);
	    	Parking_Rect_Point3_y = Rear_Axle_Center_y2-(McuSend_PcReadData.parking_rect_point3_x/Pixel_Ration2);
			#endif
			AndroidReceive_SocWriteData.target_parking_point0_x = Rear_Axle_Center_x2-(McuSend_PcReadData.parking_rect_point0_y/Pixel_Ration2_y);
	   		AndroidReceive_SocWriteData.target_parking_point0_y = Rear_Axle_Center_y2-(McuSend_PcReadData.parking_rect_point0_x/Pixel_Ration2_x);

	    	AndroidReceive_SocWriteData.target_parking_point1_x = Rear_Axle_Center_x2-(McuSend_PcReadData.parking_rect_point1_y/Pixel_Ration2_y);
	   		AndroidReceive_SocWriteData.target_parking_point1_y = Rear_Axle_Center_y2-(McuSend_PcReadData.parking_rect_point1_x/Pixel_Ration2_x);

	    	AndroidReceive_SocWriteData.target_parking_point2_x = Rear_Axle_Center_x2-(McuSend_PcReadData.parking_rect_point2_y/Pixel_Ration2_y);
	    	AndroidReceive_SocWriteData.target_parking_point2_y = Rear_Axle_Center_y2-(McuSend_PcReadData.parking_rect_point2_x/Pixel_Ration2_x);

	    	AndroidReceive_SocWriteData.target_parking_point3_x = Rear_Axle_Center_x2-(McuSend_PcReadData.parking_rect_point3_y/Pixel_Ration2_y);
	    	AndroidReceive_SocWriteData.target_parking_point3_y = Rear_Axle_Center_y2-(McuSend_PcReadData.parking_rect_point3_x/Pixel_Ration2_x);


			AndroidReceive_SocWriteData.virtual_parking_point0_x = Rear_Axle_Center_x2-(McuSend_PcReadData.travel_parking_rect_point0_y/Pixel_Ration2_y);
			AndroidReceive_SocWriteData.virtual_parking_point0_y = Rear_Axle_Center_y2-(McuSend_PcReadData.travel_parking_rect_point0_x/Pixel_Ration2_x);

			AndroidReceive_SocWriteData.virtual_parking_point1_x = Rear_Axle_Center_x2-(McuSend_PcReadData.travel_parking_rect_point1_y/Pixel_Ration2_y);
			AndroidReceive_SocWriteData.virtual_parking_point1_y = Rear_Axle_Center_y2-(McuSend_PcReadData.travel_parking_rect_point1_x/Pixel_Ration2_x);

			AndroidReceive_SocWriteData.virtual_parking_point2_x = Rear_Axle_Center_x2-(McuSend_PcReadData.travel_parking_rect_point2_y/Pixel_Ration2_y);
			AndroidReceive_SocWriteData.virtual_parking_point2_y = Rear_Axle_Center_y2-(McuSend_PcReadData.travel_parking_rect_point2_x/Pixel_Ration2_x);

			AndroidReceive_SocWriteData.virtual_parking_point3_x = Rear_Axle_Center_x2-(McuSend_PcReadData.travel_parking_rect_point3_y/Pixel_Ration2_y);
			AndroidReceive_SocWriteData.virtual_parking_point3_y = Rear_Axle_Center_y2-(McuSend_PcReadData.travel_parking_rect_point3_x/Pixel_Ration2_x);

			printf("MCU PO:%d  %d  P1:%d %d  P2:%d  %d  P3:%d %d\n",McuSend_PcReadData.parking_rect_point0_x,McuSend_PcReadData.parking_rect_point0_y,McuSend_PcReadData.parking_rect_point1_x,McuSend_PcReadData.parking_rect_point1_y,McuSend_PcReadData.parking_rect_point2_x,McuSend_PcReadData.parking_rect_point2_y,McuSend_PcReadData.parking_rect_point3_x,McuSend_PcReadData.parking_rect_point3_y);
			printf("MCU XYZ:%d,%d,%d\n",McuSend_PcReadData.TimeStampex[0],McuSend_PcReadData.TimeStampex[1],McuSend_PcReadData.TimeStampex[2]);
			#if 1
			printf("ssssssssssssssssssssssssssssss^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
			printf("MCU PO:%d  %d  P1:%d %d  P2:%d	%d	P3:%d %d\n",McuSend_PcReadData.parking_rect_point0_x,McuSend_PcReadData.parking_rect_point0_y,McuSend_PcReadData.parking_rect_point1_x,McuSend_PcReadData.parking_rect_point1_y,McuSend_PcReadData.parking_rect_point2_x,McuSend_PcReadData.parking_rect_point2_y,McuSend_PcReadData.parking_rect_point3_x,McuSend_PcReadData.parking_rect_point3_y);
						printf("travee: P0:%d  %d  \n",McuSend_PcReadData.travel_parking_rect_point0_x,McuSend_PcReadData.travel_parking_rect_point0_y);
						printf("travee: P1:%d  %d  \n",McuSend_PcReadData.travel_parking_rect_point1_x,McuSend_PcReadData.travel_parking_rect_point1_y);
						printf("travee: P2:%d  %d  \n",McuSend_PcReadData.travel_parking_rect_point2_x,McuSend_PcReadData.travel_parking_rect_point2_y);
						printf("travee: P3:%d  %d  \n",McuSend_PcReadData.travel_parking_rect_point3_x,McuSend_PcReadData.travel_parking_rect_point3_y);

						printf("to ANDROID: P0:%d  %d  \n",AndroidReceive_SocWriteData.target_parking_point0_x,AndroidReceive_SocWriteData.target_parking_point0_y);
						printf("to ANDROID: P1:%d  %d  \n",AndroidReceive_SocWriteData.target_parking_point1_x,AndroidReceive_SocWriteData.target_parking_point1_y);
						printf("to ANDROID: P2:%d  %d  \n",AndroidReceive_SocWriteData.target_parking_point2_x,AndroidReceive_SocWriteData.target_parking_point2_y);
						printf("to ANDROID: P3:%d  %d  \n",AndroidReceive_SocWriteData.target_parking_point3_x,AndroidReceive_SocWriteData.target_parking_point3_y);

						printf("to ANDROID xingcheng: P0:%d  %d  \n",AndroidReceive_SocWriteData.virtual_parking_point0_x,AndroidReceive_SocWriteData.virtual_parking_point0_y);
						printf("to ANDROID xingcheng: P1:%d  %d  \n",AndroidReceive_SocWriteData.virtual_parking_point1_x,AndroidReceive_SocWriteData.virtual_parking_point1_y);
						printf("to ANDROID xingcheng: P2:%d  %d  \n",AndroidReceive_SocWriteData.virtual_parking_point2_x,AndroidReceive_SocWriteData.virtual_parking_point2_y);
						printf("to ANDROID xingcheng: P3:%d  %d  \n",AndroidReceive_SocWriteData.virtual_parking_point3_x,AndroidReceive_SocWriteData.virtual_parking_point3_y);
			#endif
		}


	}
	else
	{
		//memset(Ver_Pos_Parking,0,sizeof(GLfloat)*10*4*3);
	}
	///////////////////////////////////////////////////////////
	#if 0
				parking_place_detected_flag = 1;
				ParkPlaceCopy[0]=6410;
				ParkPlaceCopy[1]=1840;
				ParkPlaceCopy[2]=4277;
				ParkPlaceCopy[3]=1873;
				ParkPlaceCopy[4]=4197;
				ParkPlaceCopy[5]=6733;
				ParkPlaceCopy[6]=6380;
				ParkPlaceCopy[7]=6746;
				//#endif
				//#if 0
				//ParkPlaceCopy[0]=6410;
				//ParkPlaceCopy[1]=-1840;
				//ParkPlaceCopy[2]=4277;
				//ParkPlaceCopy[3]=-1873;
				//ParkPlaceCopy[4]=4197;
				//ParkPlaceCopy[5]=-6733;
				//ParkPlaceCopy[6]=6380;
				//ParkPlaceCopy[7]=-6746;
				//#endif
				ParkPlaceCopy[8+0]=6410;
				ParkPlaceCopy[8+1]=-1840;

				ParkPlaceCopy[8+2]=1277;
				ParkPlaceCopy[8+3]=-1873;

				ParkPlaceCopy[8+4]=1197;
				ParkPlaceCopy[8+5]=-4733;

				ParkPlaceCopy[8+6]=6380;
				ParkPlaceCopy[8+7]=-4746;



				ParkPlaceCopy[2*8+0]=-6410;
				ParkPlaceCopy[2*8+1]=-1840;

				ParkPlaceCopy[2*8+2]=-1277;
				ParkPlaceCopy[2*8+3]=-1873;

				ParkPlaceCopy[2*8+4]=-1197;
				ParkPlaceCopy[2*8+5]=-4733;

				ParkPlaceCopy[2*8+6]=-6380;
				ParkPlaceCopy[2*8+7]=-4746;



				ParkPlaceCopy[3*8+0]=6410;
				ParkPlaceCopy[3*8+1]=-1840;

				ParkPlaceCopy[3*8+2]=1277;
				ParkPlaceCopy[3*8+3]=-1873;

				ParkPlaceCopy[3*8+4]=1197;
				ParkPlaceCopy[3*8+5]=-4733;

				ParkPlaceCopy[3*8+6]=6380;
				ParkPlaceCopy[3*8+7]=-4746;
				int disPoint01= (ParkPlaceCopy[0]-ParkPlaceCopy[2])*(ParkPlaceCopy[0]-ParkPlaceCopy[2])+(ParkPlaceCopy[1]-ParkPlaceCopy[3])*(ParkPlaceCopy[1]-ParkPlaceCopy[3]);
				int disPoint03= (ParkPlaceCopy[0]-ParkPlaceCopy[6])*(ParkPlaceCopy[0]-ParkPlaceCopy[6])+(ParkPlaceCopy[1]-ParkPlaceCopy[7])*(ParkPlaceCopy[1]-ParkPlaceCopy[7]);
				if(disPoint01 <= disPoint03)
				{
					AndroidReceive_SocWriteData.parking_1_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[1]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[0]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[3]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[2]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[5]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[4]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[7]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[6]/Pixel_Ration);
				}
				else
				{
					AndroidReceive_SocWriteData.parking_1_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[7]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[6]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[1]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[0]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[3]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[2]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[5]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_1_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[4]/Pixel_Ration);
				}
				int k = 1*8;
				disPoint01= (ParkPlaceCopy[k+0]-ParkPlaceCopy[k+2])*(ParkPlaceCopy[k+0]-ParkPlaceCopy[k+2])+(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+3])*(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+3]);
				disPoint03= (ParkPlaceCopy[k+0]-ParkPlaceCopy[k+6])*(ParkPlaceCopy[k+0]-ParkPlaceCopy[k+6])+(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+7])*(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+7]);
				if(disPoint01 <= disPoint03)
				{
					AndroidReceive_SocWriteData.parking_2_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+1]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+0]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+3]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+2]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+5]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+4]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+7]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+6]/Pixel_Ration);
				}
				else
				{
					AndroidReceive_SocWriteData.parking_2_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+7]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+6]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+1]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+0]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+3]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+2]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+5]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_2_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+4]/Pixel_Ration);
				}

				k = 2*8;
				disPoint01= (ParkPlaceCopy[k+0]-ParkPlaceCopy[k+2])*(ParkPlaceCopy[k+0]-ParkPlaceCopy[k+2])+(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+3])*(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+3]);
				disPoint03= (ParkPlaceCopy[k+0]-ParkPlaceCopy[k+6])*(ParkPlaceCopy[k+0]-ParkPlaceCopy[k+6])+(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+7])*(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+7]);
				if(disPoint01 <= disPoint03)
				{
					AndroidReceive_SocWriteData.parking_3_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+1]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+0]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+3]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+2]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+5]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+4]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+7]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+6]/Pixel_Ration);
				}
				else
				{
					AndroidReceive_SocWriteData.parking_3_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+7]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+6]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+1]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+0]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+3]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+2]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+5]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_3_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+4]/Pixel_Ration);
				}

				k = 3*8;
				disPoint01= (ParkPlaceCopy[k+0]-ParkPlaceCopy[k+2])*(ParkPlaceCopy[k+0]-ParkPlaceCopy[k+2])+(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+3])*(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+3]);
				disPoint03= (ParkPlaceCopy[k+0]-ParkPlaceCopy[k+6])*(ParkPlaceCopy[k+0]-ParkPlaceCopy[k+6])+(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+7])*(ParkPlaceCopy[k+1]-ParkPlaceCopy[k+7]);
				if(disPoint01 <= disPoint03)
				{
					AndroidReceive_SocWriteData.parking_4_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+1]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+0]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+3]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+2]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+5]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+4]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+7]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+6]/Pixel_Ration);
				}
				else
				{
					AndroidReceive_SocWriteData.parking_4_point0_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+7]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point0_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+6]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point1_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+1]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point1_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+0]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point2_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+3]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point2_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+2]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point3_x = Rear_Axle_Center_x-(ParkPlaceCopy[k+5]/Pixel_Ration);
					AndroidReceive_SocWriteData.parking_4_point3_y = Rear_Axle_Center_y-(ParkPlaceCopy[k+4]/Pixel_Ration);
				}
				#endif

}

void insert_trail(LinkList * list,LinkList * newnode) //链表尾部添加节点
{
	LinkList *h = list;
	while (h->next != NULL)
	{
		h = h->next;
		//printf("%d  ", h->score);
	}
	h->next = newnode;
}
void deleteNode(LinkList *list, int n) //删除链表第n个节点
{
	LinkList *t = list, *in;
	int i = 0;
	while (i < n && t != NULL)
	{
		in = t;
		t = t->next;
		i++;
	}
	if (t != NULL)
	{
		in->next = t->next;
		free(t);
	}
	else
	{
		puts("节点不存在");
	}
}
void deleteLinklist(LinkList *list)  //删除整个链表，释放空间
{
	LinkList * pNext;
	LinkList * pHead = list;
	while(pHead != NULL)
	{
		pNext = pHead->next;
		free(pHead);
		pHead = pNext;
	}
}

static void coord_cc2oc(short int coords[3], float cc[8], float oc[8])
{
    float radian = (float) coords[2] / 10000;

    for (int i=0; i<4; i++)
    {
        //float x =  cc[i * 2 + 1];
       // float y = -cc[i * 2];
       	float y =  cc[i * 2 + 1];
      	float x =  cc[i * 2];
        oc[i * 2]     = x * cos(radian) - y * sin(radian) + coords[0];
        oc[i * 2 + 1] = x * sin(radian) + y * cos(radian) + coords[1];
    }
}

static void coord_pts2veh(short int curr_coords[3], short int prev_coords[3], float prev_pts[8], float pts2veh[8])
{
    // convert CC to OC for previous points
    float prev_oc[8];
    coord_cc2oc(prev_coords, prev_pts, prev_oc);

    // convert OC to coordinates relative to vehicle
    float radian = (float) curr_coords[2] / 10000;
    for (int i=0; i<4; i++)
    {
        float delta_x = curr_coords[0] - prev_oc[i * 2];
        float delta_y = curr_coords[1] - prev_oc[i * 2+1];
        pts2veh[i * 2]    = -cos(radian) * delta_x - sin(radian) * delta_y;
        pts2veh[i * 2+ 1] =  sin(radian) * delta_x - cos(radian) * delta_y;
    }
}
