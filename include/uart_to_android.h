#ifndef __UART_TO_ANDROID_H__
#define __UART_TO_ANDROID_H__

#include <semaphore.h>
#include <pthread.h>
#include <sys/time.h>
#include "unistd.h"

typedef struct {
	unsigned char f_start;
	unsigned char id_flag;
	unsigned char d_len;
	unsigned char *data;
	unsigned char xor_flag;
}android_soc_frame_t;

typedef struct SocReadData
{
	char function_mode;					//功能模式
	char avm_parking_mode;				//AVM模式下功能 泊入，泊出等
	char transparency;					//透明度
	char viewID;						//视图ID
	char buttonID;						//按钮ID
	char click_mode;					//点击模式
	short int click_point_x;			//单击和双击触屏位置X坐标
	short int click_point_y;			//单击和双击触屏位置Y坐标
	short int slide_point_start_x;		//拖动模式下触摸点起始位置X坐标
	short int slide_point_start_y;		//拖动模式下触摸点起始位置Y坐标
	short int slide_point_current_x;	//拖动模式下当前触摸点位置X坐标
	short int slide_point_current_y;	//拖动模式下当前触摸点位置Y坐标

	char parking_in_mode;				//泊入模式
	char auto_parking_in_stallID;		//自动泊入模式下的车位ID
	short int optional_point0_x;		//自选目标车位P0点X坐标
	short int optional_point0_y;		//自选目标车位P0点Y坐标
	short int optional_point1_x;		//自选目标车位P1点X坐标
	short int optional_point1_y;		//自选目标车位P1点Y坐标
	short int optional_point2_x;		//自选目标车位P2点X坐标
	short int optional_point2_y;		//自选目标车位P2点Y坐标
	short int optional_point3_x;		//自选目标车位P3点X坐标
	short int optional_point3_y;		//自选目标车位P3点Y坐标
	
}SOCREADDATA;

typedef struct SocWriteData
{
	unsigned char Soc_Write_Flag ;	//Soc写数据标志     
	/**		   bit7.6.5.4.3.2.1.0					
				  ｜｜｜｜｜｜｜｜--- 第一组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜｜｜｜｜—---- 第二组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜｜｜｜—----—— 第三组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜｜｜—------—— 第四组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜｜—--------—— 第五组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜—----------—— 第六组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜—------------—— 第七组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜—--------------—— 第八组数据发送标志位 1：执行写数据  0：不执行写数据
	**********************************************************/
	short int CarSpeed;					//车速信号
	char gear_status_actual;			//实际档位信息
	char parking_progress_bar;			//泊车进度条(进度百分比)
	char parking_progress_current;		//当前泊车步骤
	char parking_progress_total;		//总泊车步骤
	char turn_signal;					//转向灯信号
	char optional_parking_status; 		//1 自选车位可泊        2自选车位不可泊           0 默认
	
	short int parking_1_point0_x;		//车位1 P0点X坐标
	short int parking_1_point0_y;		//车位1 P0点Y坐标
	short int parking_1_point1_x;		//车位1 P1点X坐标
	short int parking_1_point1_y;		//车位1 P1点Y坐标
	short int parking_1_point2_x;		//车位1 P2点X坐标
	short int parking_1_point2_y;		//车位1 P2点Y坐标
	short int parking_1_point3_x;		//车位1 P3点X坐标
	short int parking_1_point3_y;		//车位1 P3点Y坐标
	
	short int parking_2_point0_x;		//车位2 P0点X坐标
	short int parking_2_point0_y;		//车位2 P0点Y坐标
	short int parking_2_point1_x;		//车位2 P1点X坐标
	short int parking_2_point1_y;		//车位2 P1点Y坐标
	short int parking_2_point2_x;		//车位2 P2点X坐标
	short int parking_2_point2_y;		//车位2 P2点Y坐标
	short int parking_2_point3_x;		//车位2 P3点X坐标
	short int parking_2_point3_y;		//车位2 P3点Y坐标
	
	short int parking_3_point0_x;		//车位3 P0点X坐标
	short int parking_3_point0_y;		//车位3 P0点Y坐标
	short int parking_3_point1_x;		//车位3 P1点X坐标
	short int parking_3_point1_y;		//车位3 P1点Y坐标
	short int parking_3_point2_x;		//车位3 P2点X坐标
	short int parking_3_point2_y;		//车位3 P2点Y坐标
	short int parking_3_point3_x;		//车位3 P3点X坐标
	short int parking_3_point3_y;		//车位3 P3点Y坐标
	
	short int parking_4_point0_x;		//车位4 P0点X坐标
	short int parking_4_point0_y;		//车位4 P0点Y坐标
	short int parking_4_point1_x;		//车位4 P1点X坐标
	short int parking_4_point1_y;		//车位4 P1点Y坐标
	short int parking_4_point2_x;		//车位4 P2点X坐标
	short int parking_4_point2_y;		//车位4 P2点Y坐标
	short int parking_4_point3_x;		//车位4 P3点X坐标
	short int parking_4_point3_y;		//车位4 P3点Y坐标
	
	short int virtual_parking_point0_x;	//行程虚拟车位P0点X坐标
	short int virtual_parking_point0_y;	//行程虚拟车位P0点Y坐标
	short int virtual_parking_point1_x;	//行程虚拟车位P1点X坐标
	short int virtual_parking_point1_y;	//行程虚拟车位P1点Y坐标
	short int virtual_parking_point2_x;	//行程虚拟车位P2点X坐标
	short int virtual_parking_point2_y;	//行程虚拟车位P2点Y坐标
	short int virtual_parking_point3_x;	//行程虚拟车位P3点X坐标
	short int virtual_parking_point3_y;	//行程虚拟车位P3点Y坐标

	short int target_parking_point0_x;	//目标车位P0点X坐标
	short int target_parking_point0_y;	//目标车位P0点Y坐标
	short int target_parking_point1_x;	//目标车位P1点X坐标
	short int target_parking_point1_y;	//目标车位P1点Y坐标
	short int target_parking_point2_x;	//目标车位P2点X坐标
	short int target_parking_point2_y;	//目标车位P2点Y坐标
	short int target_parking_point3_x;	//目标车位P3点X坐标
	short int target_parking_point3_y;	//目标车位P3点Y坐标

	char change_view_flag;				//视图切换标志位
	char viewID;						//视图ID
	char exception_flag;				//异常提示标志位
	char exceptionID;					//异常提示ID

}SOCWRITEDATA;

char android_printf_flag;
#define F_START_SOC_ANDROID			    0xa0
#define F_START_ANDROID		    0xb0

#ifdef __cplusplus
extern "C" {
#endif
 void *Uart_to_Android_RX_thread(void *t);
 void *Uart_to_Android_TX_thread(void *t);
 extern unsigned char Add_Verify(unsigned char *p,unsigned char length);

#ifdef __cplusplus
}
#endif

#endif


