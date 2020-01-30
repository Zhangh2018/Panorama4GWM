#include <stdio.h>
#include <linux/input.h>
#include <fcntl.h>
#include <sys/time.h>
#include <unistd.h>
#include "ImageStitching.h"
#include "uart_to_mcu.h"

//void keyboard_test()
extern int parking_mode_select;
extern int parking_mode_ok;
extern int parking_mode_ok_flag;

/////////////////////////////////////wyf added//////////////////////////////////
#if 0
extern int Parking_Select_Num ; //parking place selected
extern int Car_Parking_Status ;
extern int Parking_Place_Mode_Select;
extern int Parking_Place_Mode_Select_OK;

extern int Parking_Place_Mode_Left_Vert_Select;
extern int Parking_Place_Mode_Rght_Vert_Select;
extern int Parking_Place_Mode_Left_Vert_Select_Ok;
extern int Parking_Place_Mode_Right_Vert_Select_Ok;
extern int SwitchChannelNum;
extern int Total_Parking_Num;
extern TARGETPLACEDATA ToMcuTargetData;
extern int Car_Speed_Flag;
extern int AvmVideoImageSwitchFlag;
extern int Mcu_Send_Parking_Data_Fag;
#endif 
extern int SwitchChannelNum;
extern float yangjiao,xuanzhuan;

//////////////////////////////////////////////////////////////////////////////
void *KeyTask(void *ptr1)
{
    int fd=open("/dev/input/event0",O_RDWR);
    if( fd <= 0 )
    {
        printf("Can not open keyboard input file\n");
    }
   printf("This is a button  KeyTask\n");
 //  struct input_event *event;
    char buf[128] = {0};
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
	cha16 = 2;
    while(1)
    {
        int ret = select(fd + 1,&rfds, NULL,NULL,NULL);
        if(ret < 0)
            continue;
        if( FD_ISSET(fd, &rfds) )
        {
            int readn = read(fd, buf, sizeof(struct input_event));
            if (readn <= 0)
            {
                printf("uart read error %d\n", readn);
                continue;
            }
            struct input_event *key_event=(struct input_event*)buf;
            if(key_event->type == EV_KEY)      //ok
            {
                if((key_event->code == 59) && !key_event->value)
                {
                	#if 0
					cha16++;
					console_cmd = cha16;
				//	if(cha16>3)
				//	cha16 = 2;
					console_cmd = cha16;
					parking_mode_ok_flag = 1;
					//printf("key16--mode: parking_mode_ok_flag=%d\n",parking_mode_ok_flag);	
                        	//printf("This is a button:%d %d\n", key_event->code,key_event->value);
					#endif
					////////////////////////wyf added//////////////////////////////////////////
					#if 0
					if(Car_Parking_Status>0)
					{
						if(Parking_Place_Mode_Select == 9)
							Parking_Place_Mode_Left_Vert_Select_Ok = 1;
						else if(Parking_Place_Mode_Select == 10)
							Parking_Place_Mode_Right_Vert_Select_Ok = 1;

						//Parking_Place_Mode_Select_OK = 1;
					}
					#endif
					#if 0
					if(Car_Parking_Status ==1 )
					{
						if(Parking_Select_Num>0)
						Parking_Place_Mode_Select_OK = 1;
					}
					
					//Car_Parking_Status = 1;
					//printf("Car_Parking_Status=%d\n",Car_Parking_Status);

					if(Car_Parking_Status <1 || Car_Parking_Status>3)
					{
						AvmVideoImageSwitchFlag++;
						AvmVideoImageSwitchFlag = AvmVideoImageSwitchFlag%2;
					}
					printf("key16:POWER_key::\n");
					#endif
					xuanzhuan=xuanzhuan+30*3.141592654 / 180.0;
					printf("key16--mode: parking_mode_ok\n");
					

					///////////////////////////////////////////////////////////////////////////
				}
				else if((key_event->code == 60) && !key_event->value) //mode select
				{
					
					//channel_select++;
					//if(channel_select>3)
					//channel_select = 0;	
					
					//if(parking_mode_ok_flag == 0)
					//	parking_mode_select++;
					//if(parking_mode_select>4)
					//	parking_mode_select = 1;

					//////////////////////////////wyf added 2018.10.19///////////////////////
					#if 0
					if(Parking_Place_Mode_Select<9 || Parking_Place_Mode_Left_Vert_Select_Ok==1 || Parking_Place_Mode_Right_Vert_Select_Ok==1 )
						Car_Parking_Status ++;
					if(Car_Parking_Status>5)
						Car_Parking_Status = 0;
					printf("key17 MODE_key--Car_Parking_Status=%d\n",Car_Parking_Status);
					#endif
					//Car_Parking_Status = 1;
					#if 0
					if(Car_Parking_Status == 0)
					{
						Parking_Place_Mode_Left_Vert_Select_Ok = 0;
						Parking_Place_Mode_Right_Vert_Select_Ok = 0;
						Parking_Place_Mode_Select = 0;
						Parking_Select_Num = 0;
						Parking_Place_Mode_Left_Vert_Select = 0;
						Parking_Place_Mode_Rght_Vert_Select = 0;
						Parking_Place_Mode_Select_OK = 0;
						
					}
					
					if(Parking_Place_Mode_Select<9 || Parking_Place_Mode_Left_Vert_Select_Ok==1 || Parking_Place_Mode_Right_Vert_Select_Ok==1 )
						SwitchChannelNum++;
					if(SwitchChannelNum>3)
						SwitchChannelNum = 0;
					printf("key17 MODE_key--SwitchChannelNum=%d\n",SwitchChannelNum);

					if(Car_Parking_Status==2)
					{
						if(Parking_Place_Mode_Select == 9 && Parking_Place_Mode_Left_Vert_Select_Ok==0)
							Parking_Place_Mode_Left_Vert_Select=(Parking_Place_Mode_Left_Vert_Select+1)%2;
						else if(Parking_Place_Mode_Select == 10 && Parking_Place_Mode_Right_Vert_Select_Ok==0)
							Parking_Place_Mode_Rght_Vert_Select=(Parking_Place_Mode_Rght_Vert_Select+1)%2;
					}
					#endif
					////////////////////////////////////////////////////////////////////////
					#if 1
					

					//if(Car_Parking_Status <1 || Car_Parking_Status>3)
					{
						SwitchChannelNum++;
						if(SwitchChannelNum>20)//;12)//9)
						SwitchChannelNum = 0;
						printf("key 17::SwitchChannelNum==%d\n",SwitchChannelNum);
					}
					#endif
					if (SwitchChannelNum<8)
						xuanzhuan = 0;
					printf("key17--Mode_key\n");
					//////////////////////////////////////////////////////////////////////
					
					//printf("key17--camera %d  parking_mode_select=%d\n",channel_select,parking_mode_select);
				}
				else if((key_event->code == 61) && !key_event->value) //clean
				{
					//parking_mode_select = 5;
					//parking_mode_ok_flag = 0;
					printf("key18--down\n");

					//////////////////////////////wyf added 2018.10.19///////////////////////
					/*(Car_Parking_Status == 2)
					{
						if((Parking_Place_Mode_Select == 9) && (Parking_Place_Mode_Left_Vert_Select_Ok == 0))
						{
							Parking_Place_Mode_Left_Vert_Select = (Parking_Place_Mode_Left_Vert_Select+1)%2 ;
							printf("key18--Parking_Place_Mode_Left_Vert_Select ==%d\n",Parking_Place_Mode_Left_Vert_Select);
						}
						else if((Parking_Place_Mode_Select == 10) && (Parking_Place_Mode_Right_Vert_Select_Ok == 0))
						{
							Parking_Place_Mode_Rght_Vert_Select=(Parking_Place_Mode_Rght_Vert_Select+1)%2;
							printf("key18--Parking_Place_Mode_Rght_Vert_Select ==%d\n",Parking_Place_Mode_Rght_Vert_Select);
						}
					}
					*/
					#if 0
					if(Car_Parking_Status == 2)
					{
						
						if(Parking_Place_Mode_Select_OK==0)
						{
							Parking_Select_Num++;
							if(Parking_Select_Num>Total_Parking_Num)
								Parking_Select_Num = 1;
							Parking_Place_Mode_Select = Parking_Select_Num;
						}
						
					}
					printf("key18--down Parking_Select_Num=%d,,Parking_Place_Mode_Select=%d\n",Parking_Select_Num,Parking_Place_Mode_Select);
					#endif
					/////////////////////////////////////////////////////////////////////////
				}
           }
        }
		usleep(50000);
    }
}

