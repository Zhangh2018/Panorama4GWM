#include "stdio.h"
#include "string.h"

#include "fcntl.h"
#include "errno.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "stdlib.h"
#include "stdarg.h"
#include "termios.h"
#include <unistd.h>

#include"uart_to_android.h"

android_soc_frame_t android_frame;
android_soc_frame_t soc_frame;

char Android_receive_FLAG_tmp; 
char Android_receive_FLAG = 1;
char Android_receive_FLAG_02 = 1;
char Android_receive_FLAG_03 = 1;
char Android_receive_FLAG_04 = 1;

char Soc_Tx_Android_Nums      =0;
char Soc_Tx_Android_Nums_02   =0;
char Soc_Tx_Android_Nums_03   =0;
char Soc_Tx_Android_Nums_04   =0;

#define AndroidOk 1
#define AndroidErr 2

SOCREADDATA AndroidSend_SocReadData;
SOCWRITEDATA AndroidReceive_SocWriteData;

ssize_t fd_uart_android;
unsigned char socrxBuffer[100];
unsigned char soctxBuffer[40];
unsigned char soctx2Buffer[70];
unsigned char soctx3Buffer[40];
unsigned char soctx4Buffer[38];

extern int parking_place_detected_flag;
extern int parking_place_detected_num;
extern int SwitchChannelNum;
extern int angle;
extern float yangjiao;
extern int angle_yangjiao;
extern int Car_Parking_Status;
extern int parking_in_scan_direction;

int uart_to_screen_comm_status;	//1: uart communication success  0:uart communication fail
int cnt2 = 100;
extern int phone_control_park_enable_status;

int get_coordinate_area(int x, int y);

void frames_get_data(unsigned char *r_buffer, unsigned char len);

int read_n_bytes(int fd,char* buf,int n)
{

/*read_len用来记录每次读到的长度，len用来记录当前已经读取的长度*/
	int read_len = 0,len = 0;

/*read_buf用来存储读到的数据*/
	char* read_buf = buf;
	while(len < n)
	{
		if((read_len = read(fd,read_buf,n)) > 0)
		{
			len += read_len;
			read_buf += read_len;
			read_len = 0;
		}
		usleep(5);

	}
	return n;
}


void *Uart_to_Android_RX_thread(void *t)
{
	printf("create Uart_to_Android_RX_thread success\n");

	struct termios tty_attributes;
	ssize_t rd_count = 0;
	if ((fd_uart_android = open("/dev/ttyLF0",O_RDWR|O_NOCTTY))>=0)
	{
	    tcgetattr(fd_uart_android, &tty_attributes);
 
        // c_cflag
        // Enable receiver
        tty_attributes.c_cflag |= CREAD;        
 
        // 8 data bit
        tty_attributes.c_cflag |= CS8;          

		//tty_attributes.c_cflag |= CLOCAL; //add hukai
		//tty_attributes.c_cflag &=~(PARODD|CSTOPB|CSIZE); //add hukai
		
        // c_iflag
        // Ignore framing errors and parity errors. 
        tty_attributes.c_iflag |= IGNPAR;  
		tty_attributes.c_iflag &=~(ICRNL|IXON); //add hukai

		//tty_attributes.c_iflag &=~(ICRNL|IXON|BRKINT|INPCK|ISTRIP); //add hukai
 
        // c_lflag
        // DISABLE canonical mode.
        // Disables the special characters EOF, EOL, EOL2, 
        // ERASE, KILL, LNEXT, REPRINT, STATUS, and WERASE, and buffers by lines.
 
        // DISABLE this: Echo input characters.
		
        tty_attributes.c_lflag &= ~(ICANON);     
 
        tty_attributes.c_lflag &= ~(ECHO);      
 
        // DISABLE this: If ICANON is also set, the ERASE character erases the preceding input  
        // character, and WERASE erases the preceding word.
        tty_attributes.c_lflag &= ~(ECHOE);     
 
        // DISABLE this: When any of the characters INTR, QUIT, SUSP, or DSUSP are received, generate the corresponding signal. 
        tty_attributes.c_lflag &= ~(ISIG);  
		
		
		tty_attributes.c_oflag &= ~OPOST;   //ADDED BY CHE
		
 		// Timeout in deciseconds for non-canonical read.
        tty_attributes.c_cc[VTIME]=0; 
		
        // Minimum number of characters for non-canonical read.
        tty_attributes.c_cc[VMIN]= 32;//29;//16;            
 
                  
 
        // Set the baud rate
        cfsetospeed(&tty_attributes,B115200);     
        cfsetispeed(&tty_attributes,B115200);
		
//		cfsetospeed(&tty_attributes,B19200);     
//        cfsetispeed(&tty_attributes,B19200);
 
        tcsetattr(fd_uart_android, TCSANOW, &tty_attributes);
		//tcflush(fd_uart,TCIFLUSH);

		//McuSend_PcReadData.Pc_Read_Flag = 0;//for test to be del 

		printf("**********************uart \n");
		//tcflush(fd_uart_android,TCIFLUSH);
		while(1)
		{	
			rd_count = read(fd_uart_android, socrxBuffer,32);
					tcflush(fd_uart_android,TCIFLUSH);
			android_printf_flag =1;
			if(android_printf_flag)
			//printf("rd_count = %d\n", rd_count);
	      	if ((rd_count > 0)&&(android_printf_flag))
			{
				int i;
				for (i=0; i<rd_count; i++)
				//for (i=0; i<2; i++)
				{
					printf("%02x ", socrxBuffer[i]);
				}
				printf("\n");

			}
			//pthread_mutex_lock(&mcu_data_mutex);
			frames_get_data(socrxBuffer, rd_count);
			//pthread_mutex_unlock(&mcu_data_mutex);		
		}
	} 
	else 
	{
	    fprintf (stderr,"Open error on %s\n", strerror(errno));
	    //exit(EXIT_FAILURE);
	} 	
	 
	    close(fd_uart_android);
}

void *Uart_to_Android_TX_thread(void *t)
{
	printf("create Uart_to_Android_TX_thread success\n");
	int i=0;
	short int y = 0;
	while(1)
	{
		
		usleep(1000);////0
		static int cnt;
		cnt++;
		//if(((Android_receive_FLAG != AndroidOk)&&(Soc_Tx_Android_Nums <= 30))||((AndroidReceive_SocWriteData.Soc_Write_Flag&0x01) == 1))
		if(1)
		{
			if(cnt>6)
			{
				soctxBuffer[0] = 	F_START_SOC_ANDROID;//0xa0
				soctxBuffer[1] = 	0x10;       
				soctxBuffer[2] = 	28;
				soctxBuffer[3] = 	0;
				soctxBuffer[4] = 	0;
				soctxBuffer[5] = 	0;
				soctxBuffer[6] = 	0;
				soctxBuffer[7] = 	0;
				soctxBuffer[8] = 	0;
				soctxBuffer[9] = 	0;
				soctxBuffer[10] = 	0;
				soctxBuffer[11] = 	0;
				soctxBuffer[12] = 	0;
				soctxBuffer[13] = 	0;
				soctxBuffer[14] = 	0;
				soctxBuffer[15] = 	0;
				soctxBuffer[16] = 	0;
				soctxBuffer[17] = 	0;
				soctxBuffer[18] = 	0;
				soctxBuffer[19] = 	0;
				soctxBuffer[20] = 	0;
				soctxBuffer[21] = 	0;
				soctxBuffer[22] = 	0;
				soctxBuffer[23] = 	0;
				soctxBuffer[24] = 	0;
				soctxBuffer[25] = 	0;
				soctxBuffer[26] = 	0;
				soctxBuffer[27] = 	0;
				soctxBuffer[28] = 	0;
				soctxBuffer[29] = 	0;
				soctxBuffer[30] = 	0;
				soctxBuffer[31] = Add_Verify(&soctxBuffer[3],28);
				write(fd_uart_android, soctxBuffer, 32);
				for(int i=0;i<32;i++)
							printf("%02x ",soctxBuffer[i]);
						printf("\n");
				usleep(10000);
				cnt = 0;
				if(cnt2>0)
					cnt2--;
			}
			if(cnt2 > 100)
			{
				uart_to_screen_comm_status = 1;
			}
			else
			{
				uart_to_screen_comm_status = 0;
			}	
		#if 0
			if((AndroidReceive_SocWriteData.Soc_Write_Flag&0x01) == 1)
			{
				Soc_Tx_Android_Nums = 0;
				AndroidReceive_SocWriteData.Soc_Write_Flag&= 0xfe;
				Android_receive_FLAG = AndroidErr;
			}
			else
			{
				Soc_Tx_Android_Nums++;		
				if(Soc_Tx_Android_Nums>30)
					Soc_Tx_Android_Nums = 200;
			}
		#endif
			//printf("Soc sending.........................\n");
			soctxBuffer[0] = 	F_START_SOC_ANDROID;//0xa0
			soctxBuffer[1] = 	0x20;       
			soctxBuffer[2] = 	28;
			soctxBuffer[3] = 	AndroidReceive_SocWriteData.CarSpeed;
			soctxBuffer[4] = 	AndroidReceive_SocWriteData.gear_status_actual;
			soctxBuffer[5] = 	AndroidReceive_SocWriteData.parking_progress_bar;
			soctxBuffer[6] = 	AndroidReceive_SocWriteData.parking_progress_current;
			soctxBuffer[7] = 	AndroidReceive_SocWriteData.parking_progress_total;
			soctxBuffer[8] = 	AndroidReceive_SocWriteData.turn_signal;
			soctxBuffer[9] = 	Car_Parking_Status;
			soctxBuffer[10] = 	AndroidReceive_SocWriteData.optional_parking_status;
			soctxBuffer[11] = 	phone_control_park_enable_status;
			soctxBuffer[12] = 	0;
			soctxBuffer[13] = 	0;
			soctxBuffer[14] = 	0;
			soctxBuffer[15] = 	0;
			soctxBuffer[16] = 	0;
			soctxBuffer[17] = 	0;
			soctxBuffer[18] = 	0;
			soctxBuffer[19] = 	0;
			soctxBuffer[20] = 	0;
			soctxBuffer[21] = 	0;
			soctxBuffer[22] = 	0;
			soctxBuffer[23] = 	0;
			soctxBuffer[24] = 	0;
			soctxBuffer[25] = 	0;
			soctxBuffer[26] = 	0;
			soctxBuffer[27] = 	0;
			soctxBuffer[28] = 	0;
			soctxBuffer[29] = 	0;
			soctxBuffer[30] = 	0;
			
			soctxBuffer[31] = Add_Verify(&soctxBuffer[3],28);

			//tcflush(fd_uart_android,TCOFLUSH);
			write(fd_uart_android, soctxBuffer, 32);	
			for(int i=0;i<32;i++)
						printf("%02x ",soctxBuffer[i]);
					printf("\n");
			printf("____________to android gear=%d   progress_bar=%d currentstep=%d total=%d\n",soctxBuffer[4],soctxBuffer[5],soctxBuffer[6],soctxBuffer[7]);
			//printf("&&&&&&&DNN p0:%d %d  P1:%d,%d  P2:%d,%d  P3:%d,%d \n",PCWRITEDATA_Ptr->PC_CarPark_P0Point[0],PCWRITEDATA_Ptr->PC_CarPark_P0Point[1],PCWRITEDATA_Ptr->PC_CarPark_P1Point[0],PCWRITEDATA_Ptr->PC_CarPark_P1Point[1],PCWRITEDATA_Ptr->PC_CarPark_P2Point[0],PCWRITEDATA_Ptr->PC_CarPark_P2Point[1],PCWRITEDATA_Ptr->PC_CarPark_P3Point[0],PCWRITEDATA_Ptr->PC_CarPark_P3Point[1]);
			//printf("&&&&&&&DNN %d,%d,%d\n",PCWRITEDATA_Ptr->PC_TimeStampeX,PCWRITEDATA_Ptr->PC_TimeStampeY,PCWRITEDATA_Ptr->PC_TimeStampeZ);
			usleep(20000);	//24500		
		}
		#if 0
		else if(((Android_receive_FLAG_02 != AndroidOk)&&(Soc_Tx_Android_Nums_02 <= 30))||((AndroidReceive_SocWriteData.Soc_Write_Flag&0x02) == 2))
		{
			if((AndroidReceive_SocWriteData.Soc_Write_Flag&0x02) == 2)
			{
				Soc_Tx_Android_Nums_02 = 0;
				AndroidReceive_SocWriteData.Soc_Write_Flag&= 0xfd;
				Android_receive_FLAG_02 = AndroidErr;
			}
			else
			{
				Soc_Tx_Android_Nums_02++;		
				if(Soc_Tx_Android_Nums_02>30)
					Soc_Tx_Android_Nums_02 = 200;
			}			
	#endif
	
	#if 1 
		if(Car_Parking_Status < 3)
		{
			soctx2Buffer[0] = 	F_START_SOC_ANDROID;//0xa0
			soctx2Buffer[1] = 	0x21;       
			soctx2Buffer[2] = 	64;			

			#if 0		//两个斜车位
			static int aa;
			//aa=aa+4;
			if(aa>800)
				aa =0;
			AndroidReceive_SocWriteData.parking_2_point0_x = 1133;
			AndroidReceive_SocWriteData.parking_2_point0_y = 554+aa;

			AndroidReceive_SocWriteData.parking_2_point1_x = 1147;
			AndroidReceive_SocWriteData.parking_2_point1_y = 282+aa;

			AndroidReceive_SocWriteData.parking_2_point2_x = 625;
			AndroidReceive_SocWriteData.parking_2_point2_y = -3+aa;

			AndroidReceive_SocWriteData.parking_2_point3_x = 606;
			AndroidReceive_SocWriteData.parking_2_point3_y = 268+aa;

/*****************************************************************************************/
			
			AndroidReceive_SocWriteData.parking_1_point0_x = 1387;//1147;
			AndroidReceive_SocWriteData.parking_1_point0_y = 397+aa;//880;

			AndroidReceive_SocWriteData.parking_1_point1_x = 1405;//1136;
			AndroidReceive_SocWriteData.parking_1_point1_y = 667+aa;//576;

			AndroidReceive_SocWriteData.parking_1_point2_x = 1827;//622;
			AndroidReceive_SocWriteData.parking_1_point2_y = 336+aa;//324;

			AndroidReceive_SocWriteData.parking_1_point3_x = 1827;//625;
			AndroidReceive_SocWriteData.parking_1_point3_y = 67+aa;//626;

			parking_place_detected_flag = 1;
			parking_place_detected_num = 2;
			parking_in_scan_direction = 1;
			AndroidReceive_SocWriteData.CarSpeed = 1;
			#endif

			#if 0		//4个车位
			static int aa;
			aa=aa+3;
			if(aa>800)
				aa =0;
			AndroidReceive_SocWriteData.parking_2_point0_x = 1110;
			AndroidReceive_SocWriteData.parking_2_point0_y = 1019+aa;

			AndroidReceive_SocWriteData.parking_2_point1_x = 1114;
			AndroidReceive_SocWriteData.parking_2_point1_y = 798+aa;

			AndroidReceive_SocWriteData.parking_2_point2_x = 599;
			AndroidReceive_SocWriteData.parking_2_point2_y = 769+aa;

			AndroidReceive_SocWriteData.parking_2_point3_x = 598;
			AndroidReceive_SocWriteData.parking_2_point3_y = 986+aa;

/*****************************************************************************************/
			
			AndroidReceive_SocWriteData.parking_1_point0_x = 1134;
			AndroidReceive_SocWriteData.parking_1_point0_y = 433+aa;

			AndroidReceive_SocWriteData.parking_1_point1_x = 1142;
			AndroidReceive_SocWriteData.parking_1_point1_y = 207+aa;

			AndroidReceive_SocWriteData.parking_1_point2_x = 646;
			AndroidReceive_SocWriteData.parking_1_point2_y = 204+aa;

			AndroidReceive_SocWriteData.parking_1_point3_x = 635;
			AndroidReceive_SocWriteData.parking_1_point3_y = 426+aa;

			AndroidReceive_SocWriteData.parking_3_point0_x = 1109;
			AndroidReceive_SocWriteData.parking_3_point0_y = 1274+aa;

			AndroidReceive_SocWriteData.parking_3_point1_x = 1114;
			AndroidReceive_SocWriteData.parking_3_point1_y = 1036+aa;

			AndroidReceive_SocWriteData.parking_3_point2_x = 597;
			AndroidReceive_SocWriteData.parking_3_point2_y = 1016+aa;

			AndroidReceive_SocWriteData.parking_3_point3_x = 594;
			AndroidReceive_SocWriteData.parking_3_point3_y = 1249+aa;

			AndroidReceive_SocWriteData.parking_4_point0_x = 1125;
			AndroidReceive_SocWriteData.parking_4_point0_y = 680+aa;

			AndroidReceive_SocWriteData.parking_4_point1_x = 1131;
			AndroidReceive_SocWriteData.parking_4_point1_y = 461+aa;

			AndroidReceive_SocWriteData.parking_4_point2_x = 633;
			AndroidReceive_SocWriteData.parking_4_point2_y = 440+aa;

			AndroidReceive_SocWriteData.parking_4_point3_x = 627;
			AndroidReceive_SocWriteData.parking_4_point3_y = 655+aa;
			
			parking_place_detected_flag = 1;
			parking_place_detected_num = 3;
			parking_in_scan_direction = 1;
			AndroidReceive_SocWriteData.CarSpeed = 1;
			#endif
			//////////车位1//////////
			soctx2Buffer[3] = 	AndroidReceive_SocWriteData.parking_1_point0_x>>8;
			soctx2Buffer[4] = 	AndroidReceive_SocWriteData.parking_1_point0_x;
			soctx2Buffer[5] = 	AndroidReceive_SocWriteData.parking_1_point0_y>>8;
			soctx2Buffer[6] = 	AndroidReceive_SocWriteData.parking_1_point0_y;

			soctx2Buffer[7] = 	AndroidReceive_SocWriteData.parking_1_point1_x>>8;
			soctx2Buffer[8] = 	AndroidReceive_SocWriteData.parking_1_point1_x;
			soctx2Buffer[9] = 	AndroidReceive_SocWriteData.parking_1_point1_y>>8;
			soctx2Buffer[10] = AndroidReceive_SocWriteData.parking_1_point1_y;

			soctx2Buffer[11] = AndroidReceive_SocWriteData.parking_1_point2_x>>8;
			soctx2Buffer[12] = AndroidReceive_SocWriteData.parking_1_point2_x;
			soctx2Buffer[13] = AndroidReceive_SocWriteData.parking_1_point2_y>>8;
			soctx2Buffer[14] = AndroidReceive_SocWriteData.parking_1_point2_y;

			soctx2Buffer[15] = AndroidReceive_SocWriteData.parking_1_point3_x>>8;
			soctx2Buffer[16] = AndroidReceive_SocWriteData.parking_1_point3_x;
			soctx2Buffer[17] = AndroidReceive_SocWriteData.parking_1_point3_y>>8;
			soctx2Buffer[18] = AndroidReceive_SocWriteData.parking_1_point3_y;
			
			//////////车位2//////////
			soctx2Buffer[19] = AndroidReceive_SocWriteData.parking_2_point0_x>>8;
			soctx2Buffer[20] = AndroidReceive_SocWriteData.parking_2_point0_x;
			soctx2Buffer[21] = AndroidReceive_SocWriteData.parking_2_point0_y>>8;
			soctx2Buffer[22] = AndroidReceive_SocWriteData.parking_2_point0_y;

			soctx2Buffer[23] = AndroidReceive_SocWriteData.parking_2_point1_x>>8;
			soctx2Buffer[24] = AndroidReceive_SocWriteData.parking_2_point1_x;
			soctx2Buffer[25] = AndroidReceive_SocWriteData.parking_2_point1_y>>8;
			soctx2Buffer[26] = AndroidReceive_SocWriteData.parking_2_point1_y;

			soctx2Buffer[27] = AndroidReceive_SocWriteData.parking_2_point2_x>>8;
			soctx2Buffer[28] = AndroidReceive_SocWriteData.parking_2_point2_x;
			soctx2Buffer[29] = AndroidReceive_SocWriteData.parking_2_point2_y>>8;
			soctx2Buffer[30] = AndroidReceive_SocWriteData.parking_2_point2_y;

			soctx2Buffer[31] = AndroidReceive_SocWriteData.parking_2_point3_x>>8;
			soctx2Buffer[32] = AndroidReceive_SocWriteData.parking_2_point3_x;
			soctx2Buffer[33] = AndroidReceive_SocWriteData.parking_2_point3_y>>8;
			soctx2Buffer[34] = AndroidReceive_SocWriteData.parking_2_point3_y;
				
			//////////车位3//////////
			soctx2Buffer[35] = AndroidReceive_SocWriteData.parking_3_point0_x>>8;
			soctx2Buffer[36] = AndroidReceive_SocWriteData.parking_3_point0_x;
			soctx2Buffer[37] = AndroidReceive_SocWriteData.parking_3_point0_y>>8;
			soctx2Buffer[38] = AndroidReceive_SocWriteData.parking_3_point0_y;

			soctx2Buffer[39] = AndroidReceive_SocWriteData.parking_3_point1_x>>8;
			soctx2Buffer[40] = AndroidReceive_SocWriteData.parking_3_point1_x;
			soctx2Buffer[41] = AndroidReceive_SocWriteData.parking_3_point1_y>>8;
			soctx2Buffer[42] = AndroidReceive_SocWriteData.parking_3_point1_y;

			soctx2Buffer[43] = AndroidReceive_SocWriteData.parking_3_point2_x>>8;
			soctx2Buffer[44] = AndroidReceive_SocWriteData.parking_3_point2_x;
			soctx2Buffer[45] = AndroidReceive_SocWriteData.parking_3_point2_y>>8;
			soctx2Buffer[46] = AndroidReceive_SocWriteData.parking_3_point2_y;

			soctx2Buffer[47] = AndroidReceive_SocWriteData.parking_3_point3_x>>8;
			soctx2Buffer[48] = AndroidReceive_SocWriteData.parking_3_point3_x;
			soctx2Buffer[49] = AndroidReceive_SocWriteData.parking_3_point3_y>>8;
			soctx2Buffer[50] = AndroidReceive_SocWriteData.parking_3_point3_y;
				
			//////////车位4//////////
			soctx2Buffer[51] = AndroidReceive_SocWriteData.parking_4_point0_x>>8;
			soctx2Buffer[52] = AndroidReceive_SocWriteData.parking_4_point0_x;
			soctx2Buffer[53] = AndroidReceive_SocWriteData.parking_4_point0_y>>8;
			soctx2Buffer[54] = AndroidReceive_SocWriteData.parking_4_point0_y;

			soctx2Buffer[55] = AndroidReceive_SocWriteData.parking_4_point1_x>>8;
			soctx2Buffer[56] = AndroidReceive_SocWriteData.parking_4_point1_x;
			soctx2Buffer[57] = AndroidReceive_SocWriteData.parking_4_point1_y>>8;
			soctx2Buffer[58] = AndroidReceive_SocWriteData.parking_4_point1_y;

			soctx2Buffer[59] = AndroidReceive_SocWriteData.parking_4_point2_x>>8;
			soctx2Buffer[60] = AndroidReceive_SocWriteData.parking_4_point2_x;
			soctx2Buffer[61] = AndroidReceive_SocWriteData.parking_4_point2_y>>8;
			soctx2Buffer[62] = AndroidReceive_SocWriteData.parking_4_point2_y;

			soctx2Buffer[63] = AndroidReceive_SocWriteData.parking_4_point3_x>>8;
			soctx2Buffer[64] = AndroidReceive_SocWriteData.parking_4_point3_x;
			soctx2Buffer[65] = AndroidReceive_SocWriteData.parking_4_point3_y>>8;
			soctx2Buffer[66] = AndroidReceive_SocWriteData.parking_4_point3_y;
			
			//soctx2Buffer[67] = Add_Verify(&soctx2Buffer[3],64);
			
			if(parking_in_scan_direction > 0)
			{
				if(parking_place_detected_flag ==1)
				{
					if(parking_place_detected_num ==1)
					{
						for(int j= 0;j<16*3;j++)
							soctx2Buffer[j+19] = 0;
					}
					else if(parking_place_detected_num ==2)
					{
						for(int j= 0;j<16*2;j++)
							soctx2Buffer[j+35] = 0;
					}
					else if(parking_place_detected_num ==3)
					{
						for(int j= 0;j<16*1;j++)
							soctx2Buffer[j+51] = 0;
					}
					soctx2Buffer[67] = Add_Verify(&soctx2Buffer[3],64);
					write(fd_uart_android, soctx2Buffer, 68);	
					for(int i=0;i<68;i++)
						printf("%02x  ",soctx2Buffer[i]);
					printf("\n");
					//if(aa%200 == 0)
					//	{
					//	aa=aa+40;
					//	printf("\n\n\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
					//	printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@aa==%d\n",aa);
					//	printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
					//}
				}
				else
				{
						for(int j= 0;j<16*4;j++)
							soctx2Buffer[j+3] = 0;
						soctx2Buffer[67] = Add_Verify(&soctx2Buffer[3],64);
						write(fd_uart_android, soctx2Buffer, 68);
						printf("clean all parking place\n");
				}
			}
			else
			{
				for(int j= 0;j<16*4;j++)
					soctx2Buffer[j+3] = 0;
				soctx2Buffer[67] = Add_Verify(&soctx2Buffer[3],64);
				write(fd_uart_android, soctx2Buffer, 68);	
				printf("clean all parking place\n");
			}
			#if 1
			printf("scaned chewei 1 P0:%d %d \n",AndroidReceive_SocWriteData.parking_1_point0_x,AndroidReceive_SocWriteData.parking_1_point0_y);
			printf("scaned chewei 1 P1:%d %d \n",AndroidReceive_SocWriteData.parking_1_point1_x,AndroidReceive_SocWriteData.parking_1_point1_y);
			printf("scaned chewei 1 P2:%d %d \n",AndroidReceive_SocWriteData.parking_1_point2_x,AndroidReceive_SocWriteData.parking_1_point2_y);
			printf("scaned chewei 1 P3:%d %d \n",AndroidReceive_SocWriteData.parking_1_point3_x,AndroidReceive_SocWriteData.parking_1_point3_y);
			
			printf("scaned chewei 2 P0:%d %d \n",AndroidReceive_SocWriteData.parking_2_point0_x,AndroidReceive_SocWriteData.parking_2_point0_y);
			printf("scaned chewei 2 P1:%d %d \n",AndroidReceive_SocWriteData.parking_2_point1_x,AndroidReceive_SocWriteData.parking_2_point1_y);
			printf("scaned chewei 2 P2:%d %d \n",AndroidReceive_SocWriteData.parking_2_point2_x,AndroidReceive_SocWriteData.parking_2_point2_y);
			printf("scaned chewei 2 P3:%d %d \n",AndroidReceive_SocWriteData.parking_2_point3_x,AndroidReceive_SocWriteData.parking_2_point3_y);

			printf("scaned chewei 3 P0:%d %d \n",AndroidReceive_SocWriteData.parking_3_point0_x,AndroidReceive_SocWriteData.parking_3_point0_y);
			printf("scaned chewei 3 P1:%d %d \n",AndroidReceive_SocWriteData.parking_3_point1_x,AndroidReceive_SocWriteData.parking_3_point1_y);
			printf("scaned chewei 3 P2:%d %d \n",AndroidReceive_SocWriteData.parking_3_point2_x,AndroidReceive_SocWriteData.parking_3_point2_y);
			printf("scaned chewei 3 P3:%d %d \n",AndroidReceive_SocWriteData.parking_3_point3_x,AndroidReceive_SocWriteData.parking_3_point3_y);

			printf("scaned chewei 4 P0:%d %d \n",AndroidReceive_SocWriteData.parking_4_point0_x,AndroidReceive_SocWriteData.parking_4_point0_y);
			printf("scaned chewei 4 P1:%d %d \n",AndroidReceive_SocWriteData.parking_4_point1_x,AndroidReceive_SocWriteData.parking_4_point1_y);
			printf("scaned chewei 4 P2:%d %d \n",AndroidReceive_SocWriteData.parking_4_point2_x,AndroidReceive_SocWriteData.parking_4_point2_y);
			printf("scaned chewei 4 P3:%d %d \n",AndroidReceive_SocWriteData.parking_4_point3_x,AndroidReceive_SocWriteData.parking_4_point3_y);
			#endif
			usleep(20000);	//27500					
		}
		#endif
		#if 1
		///else if(((Android_receive_FLAG_03 != AndroidOk)&&(Soc_Tx_Android_Nums_03 <= 30))||((AndroidReceive_SocWriteData.Soc_Write_Flag&0x04) == 4))
		if(Car_Parking_Status ==3)
		{					
			soctx3Buffer[0] = 	F_START_SOC_ANDROID;//0xb0
			soctx3Buffer[1] = 	0x22;       
			soctx3Buffer[2] = 	32;
			////////行程虚拟车位坐标////////
			soctx3Buffer[3] = 	AndroidReceive_SocWriteData.virtual_parking_point0_x>>8;
			soctx3Buffer[4] =   AndroidReceive_SocWriteData.virtual_parking_point0_x;
			soctx3Buffer[5] = 	AndroidReceive_SocWriteData.virtual_parking_point0_y>>8;
			soctx3Buffer[6] = 	AndroidReceive_SocWriteData.virtual_parking_point0_y;
			
			soctx3Buffer[7] = 	AndroidReceive_SocWriteData.virtual_parking_point1_x>>8;
			soctx3Buffer[8] =   AndroidReceive_SocWriteData.virtual_parking_point1_x;
			soctx3Buffer[9] = 	AndroidReceive_SocWriteData.virtual_parking_point1_y>>8;
			soctx3Buffer[10] =  AndroidReceive_SocWriteData.virtual_parking_point1_y;
			
			soctx3Buffer[11] = AndroidReceive_SocWriteData.virtual_parking_point2_x>>8;
			soctx3Buffer[12] = AndroidReceive_SocWriteData.virtual_parking_point2_x;
			soctx3Buffer[13] = AndroidReceive_SocWriteData.virtual_parking_point2_y>>8;
			soctx3Buffer[14] = AndroidReceive_SocWriteData.virtual_parking_point2_y;
			
			soctx3Buffer[15] = AndroidReceive_SocWriteData.virtual_parking_point3_x>>8;
			soctx3Buffer[16] = AndroidReceive_SocWriteData.virtual_parking_point3_x;
			soctx3Buffer[17] = AndroidReceive_SocWriteData.virtual_parking_point3_y>>8;
			soctx3Buffer[18] = AndroidReceive_SocWriteData.virtual_parking_point3_y;

			////////目标车位坐标////////
			soctx3Buffer[19] = AndroidReceive_SocWriteData.target_parking_point0_x>>8;
			soctx3Buffer[20] = AndroidReceive_SocWriteData.target_parking_point0_x;
			soctx3Buffer[21] = AndroidReceive_SocWriteData.target_parking_point0_y>>8;
			soctx3Buffer[22] = AndroidReceive_SocWriteData.target_parking_point0_y;
			
			soctx3Buffer[23] = AndroidReceive_SocWriteData.target_parking_point1_x>>8;
			soctx3Buffer[24] = AndroidReceive_SocWriteData.target_parking_point1_x;
			soctx3Buffer[25] = AndroidReceive_SocWriteData.target_parking_point1_y>>8;
			soctx3Buffer[26] = AndroidReceive_SocWriteData.target_parking_point1_y;
			
			soctx3Buffer[27] = AndroidReceive_SocWriteData.target_parking_point2_x>>8;
			soctx3Buffer[28] = AndroidReceive_SocWriteData.target_parking_point2_x;
			soctx3Buffer[29] = AndroidReceive_SocWriteData.target_parking_point2_y>>8;
			soctx3Buffer[30] = AndroidReceive_SocWriteData.target_parking_point2_y;
			
			soctx3Buffer[31] = AndroidReceive_SocWriteData.target_parking_point3_x>>8;
			soctx3Buffer[32] = AndroidReceive_SocWriteData.target_parking_point3_x;
			soctx3Buffer[33] = AndroidReceive_SocWriteData.target_parking_point3_y>>8;
			soctx3Buffer[34] = AndroidReceive_SocWriteData.target_parking_point3_y;

			soctx3Buffer[35] = Add_Verify(&soctx3Buffer[3],32);
			//tcflush(fd_uart_android,TCOFLUSH);
			//if(Car_Parking_Status ==3)
			//{
				write(fd_uart_android, soctx3Buffer, 36);	
			for(int i=0;i<36;i++)
						printf("%02x ",soctx3Buffer[i]);
					printf("\n");
				usleep(20000);	//27500	
			//}
		}

		if(((Android_receive_FLAG_04 != AndroidOk)&&(Soc_Tx_Android_Nums_04 <= 30))||((AndroidReceive_SocWriteData.Soc_Write_Flag&0x08) == 8))
		{							
			soctx4Buffer[0] = 	F_START_SOC_ANDROID;//0xb0
			soctx4Buffer[1] = 	0x23;       
			soctx4Buffer[2] = 	4;
			soctx4Buffer[3] = 	AndroidReceive_SocWriteData.change_view_flag;
			soctx4Buffer[4] = 	AndroidReceive_SocWriteData.viewID;
			soctx4Buffer[5] = 	AndroidReceive_SocWriteData.exception_flag;
			soctx4Buffer[6] = 	AndroidReceive_SocWriteData.exceptionID;
			
			soctx4Buffer[7] = Add_Verify(&soctx4Buffer[3],4);
			//tcflush(fd_uart_android,TCOFLUSH);
			//write(fd_uart_android, soctx4Buffer, 8);	
			//printf("tx4Buffer[19]=%d\n",tx4Buffer[19]);
			//printf("Customer_Select_Ok_Flag=%d\n",ToMcuTargetData.Customer_Select_Ok_Flag);
			//printf("ToMcuTargetData.Target x y z %d,%d,%d\n",ToMcuTargetData.Targetx,ToMcuTargetData.Targety,ToMcuTargetData.Targetz);
			//printf("TxPoint:P0 %d,%d,p1 %d,%d, P2 %d,%d, P3 %d,%d\n",ToMcuTargetData.Target_CarPark_P0Point[0],ToMcuTargetData.Target_CarPark_P0Point[1],ToMcuTargetData.Target_CarPark_P1Point[0],ToMcuTargetData.Target_CarPark_P1Point[1],ToMcuTargetData.Target_CarPark_P2Point[0],ToMcuTargetData.Target_CarPark_P2Point[1],ToMcuTargetData.Target_CarPark_P3Point[0],ToMcuTargetData.Target_CarPark_P3Point[1]);
			usleep(10000);	//27500
		}
		#endif

		if(SwitchChannelNum == 6 || SwitchChannelNum == 7)
		{
			if(AndroidReceive_SocWriteData.gear_status_actual == 1)
						  	 SwitchChannelNum = 6 ;
			else if(AndroidReceive_SocWriteData.gear_status_actual == 2)
						  	 SwitchChannelNum = 7 ;
		}
	}
}
	
int Area = 0;

short int slide_start_x;
short int slide_start_y;

short int slide_current_x;
short int slide_current_y;

void frames_get_data(unsigned char *r_buffer, unsigned char len)
{
	short int tmp = 0;
	short int tmp1 = 0;
	android_frame.f_start = r_buffer[0];
	android_frame.id_flag = r_buffer[1];
    android_frame.d_len   = r_buffer[2];
	android_frame.data	  = &r_buffer[3];
	if(android_frame.f_start == F_START_ANDROID&&(r_buffer[3+android_frame.d_len]==Add_Verify(&r_buffer[3],android_frame.d_len)))
	{
		switch(android_frame.id_flag)
		{
			case 0x01:
				cnt2 = 120;
				break;
			case 0x80:
				
				AndroidSend_SocReadData.function_mode = 	r_buffer[3];
				AndroidSend_SocReadData.avm_parking_mode =  r_buffer[4];
				AndroidSend_SocReadData.transparency = 		r_buffer[5];
				AndroidSend_SocReadData.viewID = 			r_buffer[6];
				AndroidSend_SocReadData.buttonID = 			r_buffer[7];
				AndroidSend_SocReadData.click_mode = 		r_buffer[8];
				
				tmp = 0;
				tmp |= r_buffer[9];
				tmp  = tmp << 8;
				tmp |= r_buffer[10];
				AndroidSend_SocReadData.click_point_x = tmp;
				
				tmp = 0;
				tmp |= r_buffer[11];
				tmp  = tmp << 8;
				tmp |= r_buffer[12];
				AndroidSend_SocReadData.click_point_y = tmp;

				tmp = 0;
				tmp |= r_buffer[13];
				tmp  = tmp << 8;
				tmp |= r_buffer[14];
				AndroidSend_SocReadData.slide_point_start_x = tmp;

				tmp = 0;
				tmp |= r_buffer[15];
				tmp  = tmp << 8;
				tmp |= r_buffer[16];
				AndroidSend_SocReadData.slide_point_start_y = tmp;

				tmp = 0;
				tmp |= r_buffer[17];
				tmp  = tmp << 8;
				tmp |= r_buffer[18];
				AndroidSend_SocReadData.slide_point_current_x = tmp;

				tmp = 0;
				tmp |= r_buffer[19];
				tmp  = tmp << 8;
				tmp |= r_buffer[20];
				AndroidSend_SocReadData.slide_point_current_y = tmp;

				AndroidSend_SocReadData.auto_parking_in_stallID = r_buffer[26];
				printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n0x80\n");
				printf("function_mode=%d, avm_parking_mode=%d, viewID=%d, slide_point_current_x=%d\n",AndroidSend_SocReadData.function_mode,
				AndroidSend_SocReadData.avm_parking_mode, AndroidSend_SocReadData.viewID, AndroidSend_SocReadData.slide_point_current_x);
				printf("AndroidSend_SocReadData.auto_parking_in_stallID=%d\n",AndroidSend_SocReadData.auto_parking_in_stallID);
				printf("avm_parking_mode=%d,click_mode=%d,click_x=%d, click_y=%d\n",AndroidSend_SocReadData.avm_parking_mode, AndroidSend_SocReadData.click_mode, AndroidSend_SocReadData.click_point_x, AndroidSend_SocReadData.click_point_y);
				printf("AndroidSend_SocReadData.buttonID=%d\n",AndroidSend_SocReadData.buttonID);
				Area = get_coordinate_area(AndroidSend_SocReadData.click_point_x, AndroidSend_SocReadData.click_point_y);
				//printf("Area=%d\n", Area);
				if(AndroidSend_SocReadData.function_mode == 1)//APA
				{
					if(AndroidSend_SocReadData.viewID <= 0x06 || AndroidSend_SocReadData.viewID ==0x0a)
						SwitchChannelNum = 5 ;
					else if(AndroidSend_SocReadData.viewID == 0x08 ||AndroidSend_SocReadData.viewID == 0x09 ||AndroidSend_SocReadData.viewID == 0x12)
					{ 
							 SwitchChannelNum = 6 ;
						  if(AndroidReceive_SocWriteData.gear_status_actual == 1)
						  	 SwitchChannelNum = 6 ;
						  else if(AndroidReceive_SocWriteData.gear_status_actual == 2)
						  	 SwitchChannelNum = 7 ;
					}
				}
				else if(AndroidSend_SocReadData.function_mode == 0)//AVM
				{
					if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.buttonID == 0x01))//2D
						SwitchChannelNum = 1;//front
					else if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.buttonID == 0x02))
						SwitchChannelNum = 2;//rear
					else if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.buttonID == 0x03))
						SwitchChannelNum = 3;//left
					else if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.buttonID == 0x04))
						SwitchChannelNum = 4;//right

					//3D固定角度
					if(AndroidSend_SocReadData.avm_parking_mode == 0x02 && AndroidSend_SocReadData.buttonID == 0x01)
					{
						angle = 90;//front
						angle_yangjiao = 30;
						SwitchChannelNum = 11;
					}
					else if(AndroidSend_SocReadData.avm_parking_mode == 0x02 && AndroidSend_SocReadData.buttonID == 0x02)
					{	
						angle = 0;//rear
						angle_yangjiao = 30;
					}
					else if(AndroidSend_SocReadData.avm_parking_mode == 0x02 && AndroidSend_SocReadData.buttonID == 0x03)
					{
						angle = 45;//left
						angle_yangjiao = 30;
					}
					else if(AndroidSend_SocReadData.avm_parking_mode == 0x02 && AndroidSend_SocReadData.buttonID == 0x04)
					{	
						angle = 135;//right
						angle_yangjiao = 30;
					}	
					//3D旋转
					#if 1
					//GETTIME(&lTimeEnd_point);
					if(AndroidSend_SocReadData.click_mode == 0x03)
					{//第一次进行拖动或者抬手后再次拖动，对记录的当前坐标值进行更新，避免出现因slide_current_x，slide_current_y初值为0或者当前坐标与记录的当前坐标差值较大而出现自动旋转一个角度的问题
						if((AndroidSend_SocReadData.slide_point_start_x != slide_start_x) || (AndroidSend_SocReadData.slide_point_start_y != slide_start_y))
						{//如果起始坐标发生变化（包含第一次拖动slide_start_x，slide_start_y为0，start不为0的情况），即做了抬手动作后重新滑动屏幕，把新起始坐标值传给上次当前坐标值slide_current_x slide_current_y
							slide_current_x = AndroidSend_SocReadData.slide_point_start_x;
							slide_current_y = AndroidSend_SocReadData.slide_point_start_y;
						}
					}
					#endif
					
					if((AndroidSend_SocReadData.avm_parking_mode == 0x02) && (AndroidSend_SocReadData.click_mode == 0x03))
					{
						if(AndroidSend_SocReadData.slide_point_current_y - AndroidSend_SocReadData.slide_point_start_y < 50)
						{
							
							if(AndroidSend_SocReadData.slide_point_current_x > slide_current_x)//右滑
							{
								if(AndroidSend_SocReadData.slide_point_start_x < AndroidSend_SocReadData.slide_point_current_x)
									angle += (AndroidSend_SocReadData.slide_point_current_x - AndroidSend_SocReadData.slide_point_start_x)/60;
								else if(AndroidSend_SocReadData.slide_point_start_x > AndroidSend_SocReadData.slide_point_current_x)
									angle += (AndroidSend_SocReadData.slide_point_start_x - AndroidSend_SocReadData.slide_point_current_x)/60;
							}
							else if(AndroidSend_SocReadData.slide_point_current_x < slide_current_x)//左滑
							{
								if(AndroidSend_SocReadData.slide_point_start_x > AndroidSend_SocReadData.slide_point_current_x)
									angle += (AndroidSend_SocReadData.slide_point_current_x - AndroidSend_SocReadData.slide_point_start_x)/60;
								else if(AndroidSend_SocReadData.slide_point_start_x < AndroidSend_SocReadData.slide_point_current_x)
									angle += (AndroidSend_SocReadData.slide_point_start_x - AndroidSend_SocReadData.slide_point_current_x)/60;
							}
						}
					#if 0
						if(AndroidSend_SocReadData.slide_point_current_x - AndroidSend_SocReadData.slide_point_start_x < 50)
						{
							if(AndroidSend_SocReadData.slide_point_current_y > slide_current_y)//下滑
								{
									if(AndroidSend_SocReadData.slide_point_start_y < AndroidSend_SocReadData.slide_point_current_y)
										angle_yangjiao += (AndroidSend_SocReadData.slide_point_start_y - AndroidSend_SocReadData.slide_point_current_y)/80;
									else if(AndroidSend_SocReadData.slide_point_start_y > AndroidSend_SocReadData.slide_point_current_y)
										angle_yangjiao += (AndroidSend_SocReadData.slide_point_current_y - AndroidSend_SocReadData.slide_point_start_y)/80;
								}
								else if(AndroidSend_SocReadData.slide_point_current_y < slide_current_y)//上滑
								{
									if(AndroidSend_SocReadData.slide_point_start_y > AndroidSend_SocReadData.slide_point_current_y)
										angle_yangjiao += (AndroidSend_SocReadData.slide_point_start_y - AndroidSend_SocReadData.slide_point_current_y)/80;
									else if(AndroidSend_SocReadData.slide_point_start_y < AndroidSend_SocReadData.slide_point_current_y)
										angle_yangjiao += (AndroidSend_SocReadData.slide_point_current_y - AndroidSend_SocReadData.slide_point_start_y)/80;
								}
						}
					#endif
						slide_start_x = AndroidSend_SocReadData.slide_point_start_x;
						slide_start_y = AndroidSend_SocReadData.slide_point_start_y;
						slide_current_x = AndroidSend_SocReadData.slide_point_current_x;
						slide_current_y = AndroidSend_SocReadData.slide_point_current_y;
					}
					
					if(angle_yangjiao >= 63)//上滑边界
						angle_yangjiao = 63;
					else if(angle_yangjiao <= 5)//下滑边界
						angle_yangjiao = 5;
					//printf("-----------------------------------------------------------angle_yangjiao=%d\n",angle_yangjiao);
					//printf("-----------------------------------------------------------angle=%d\n",angle);

					if(SwitchChannelNum != 0)//防止大2D情况下双击其它区域切换视图
					{
						if((AndroidSend_SocReadData.avm_parking_mode == 0x02) && (AndroidSend_SocReadData.click_mode == 0x02) && (Area == 5))
							SwitchChannelNum = 12;//2D+3D
						if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.click_mode == 0x02) && (Area == 1))//前近距
							SwitchChannelNum = 17;//前广角					SwitchChannelNum = 8;//前近距
						else if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.click_mode == 0x02) && (Area == 2))//后近距
							SwitchChannelNum = 18;//后广角					SwitchChannelNum = 9;//后近距
						else if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.click_mode == 0x02) && (Area == 3))//左右近距
							SwitchChannelNum = 10;//左右近距
						else if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.click_mode == 0x02) && (Area == 4))//大2D
							SwitchChannelNum = 0;//大2D
						else if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.click_mode == 0x02) && (Area == 7))
							SwitchChannelNum = 13;//左A柱盲区
						else if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.click_mode == 0x02) && (Area == 8))
							SwitchChannelNum = 14;//右A柱盲区
						else if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.click_mode == 0x02) && (Area == 9))
							SwitchChannelNum = 15;//左B柱盲区
						else if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.click_mode == 0x02) && (Area == 10))
							SwitchChannelNum = 16;//右B柱盲区
						else if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.click_mode == 0x02) && (Area == 11))
							SwitchChannelNum = 10;//前轮左右近距
						else if((AndroidSend_SocReadData.avm_parking_mode == 0x01) && (AndroidSend_SocReadData.click_mode == 0x02) && (Area == 12))
							SwitchChannelNum = 20;//后轮左右近距

					}

					if((SwitchChannelNum == 0) && (AndroidSend_SocReadData.click_mode == 0x02) && (Area == 5))
						SwitchChannelNum = 1;//2D+单视图前
					if((SwitchChannelNum == 12) && (AndroidSend_SocReadData.click_mode == 0x02) && (Area == 6))
						SwitchChannelNum = 11;//大3D前
				}

				break;

			case 0x81:

				AndroidSend_SocReadData.parking_in_mode = r_buffer[3];
				AndroidSend_SocReadData.auto_parking_in_stallID = r_buffer[4];
				printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
				printf("AndroidSend_SocReadData.auto_parking_in_stallID = %d\n",AndroidSend_SocReadData.auto_parking_in_stallID);
				printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
				tmp = 0;
				tmp |= r_buffer[5];
				tmp  = tmp << 8;
				tmp |= r_buffer[6];
				AndroidSend_SocReadData.optional_point0_x = tmp;

				tmp = 0;
				tmp |= r_buffer[7];
				tmp  = tmp << 8;
				tmp |= r_buffer[8];
				AndroidSend_SocReadData.optional_point0_y = tmp;

				tmp = 0;
				tmp |= r_buffer[9];
				tmp  = tmp << 8;
				tmp |= r_buffer[10];
				AndroidSend_SocReadData.optional_point1_x = tmp;

				tmp = 0;
				tmp |= r_buffer[11];
				tmp  = tmp << 8;
				tmp |= r_buffer[12];
				AndroidSend_SocReadData.optional_point1_y = tmp;

				tmp = 0;
				tmp |= r_buffer[13];
				tmp  = tmp << 8;
				tmp |= r_buffer[14];
				AndroidSend_SocReadData.optional_point2_x = tmp;

				tmp = 0;
				tmp |= r_buffer[15];
				tmp  = tmp << 8;
				tmp |= r_buffer[16];
				AndroidSend_SocReadData.optional_point2_y = tmp;

				tmp = 0;
				tmp |= r_buffer[17];
				tmp  = tmp << 8;
				tmp |= r_buffer[18];
				AndroidSend_SocReadData.optional_point3_x = tmp;

				tmp = 0;
				tmp |= r_buffer[19];
				tmp  = tmp << 8;
				tmp |= r_buffer[20];
				AndroidSend_SocReadData.optional_point3_y = tmp;
				printf("p0x=%d,p0y=%d,p1x=%d,py=%d,p2x=%d,p2y=%d,p3x=%d,p3y=%d\n", 
					AndroidSend_SocReadData.optional_point0_x,
					AndroidSend_SocReadData.optional_point0_y, 
					AndroidSend_SocReadData.optional_point1_x,
					AndroidSend_SocReadData.optional_point1_y,
					AndroidSend_SocReadData.optional_point2_x,
					AndroidSend_SocReadData.optional_point2_y,
					AndroidSend_SocReadData.optional_point3_x,
					AndroidSend_SocReadData.optional_point3_y);
				printf("0x81\n");
				break;

			default:
				break;
		}
	}
	else
	{
		printf("error:android uart rev f_start = %x\n", android_frame.f_start);
		tcflush(fd_uart_android,TCIFLUSH);
	}
}

int get_coordinate_area(int x, int y)
{
	int area = 0;
	if((x >= 1056+332) && (x <= 1056+532))
	{
		if(y <= 271)
		area = 1;//front
		else if((y >= 371) && (y <= 271+438))
		area = 4;//full screen 2D
		else if(y >= 271+538)
		area = 2;//rear
	}
	
	if((y >= 271) && (y <= 271+538))
	{
		if(((x >= 1056) && (x <= 1056+338)) || (x >= 1056+338+118))
		area = 3;//l=left and right
	}

	if((x >= 960-250) && (x <= 960+250) && (y >= 540-250) && (y <= 540+250))
		area = 5;

	if((x >= 528-250) && (x <= 528+250) && (y >= 540-250) && (y <= 540+250))
		area = 6;//full screen 3D

	if((x >= 1056) && (x <= 1056+116))
	{
		if(y <= 135)
		area = 7;//左A柱盲区
		if(y >= 1080-135)
		area = 9;//左B柱盲区
	}

	if(x >= 1920-116)
	{
		if(y <= 135)
		area = 8;//右A柱盲区
		if(y >= 1080-135)
		area = 10;//左B柱盲区
	}

	if((x >= 1056+116) && (x <= 1056+332))
	{
		if((y >= 135) && (y <= 271))
		area = 11;//前轮左右近距
		if((y >= 1080-271) && (y <= 1080-135))
		area = 12;//后轮左右近距
	}

	if((x >= 1920-332) && (x <= 1920-116))
	{
		if((y >= 135) && (y <= 271))
		area = 11;//前轮左右近距
		if((y >= 1080-271) && (y <= 1080-135))
		area = 12;//后轮左右近距
	}
	return area;
}
