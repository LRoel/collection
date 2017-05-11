
#include "UartDriver.h"

/******************************************************************* 
* 名称：                  UART0_Open 
* 功能：                打开串口并返回串口设备文件描述 
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2) 
*出口参数：              正确返回为TRUE(0), 错误返回为FALSE(-1)
*******************************************************************/  
int UART0_Open(int fd, const char* port)
{   
	fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY); 	
	if (FALSE == fd)  
	{  
		perror("Can't Open Serial Port");  
		return(FALSE);  
	}  
 	//恢复串口为阻塞状态                                 
 	if(fcntl(fd, F_SETFL, 0) < 0)  
	{  
		printf("fcntl failed!\n");  
		return(FALSE);  
	}       
	else  
	{  
		printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));  
	}  
	//测试是否为终端设备      
	if(0 == isatty(STDIN_FILENO))  
	{  
		printf("standard input is not a terminal device\n");  
		return(FALSE);  
	}  
	else  
	{  
		printf("isatty success!\n");  
	}                
	printf("fd->open=%d\n",fd);  
	return fd;  
}  


/******************************************************************* 
* 名称：                UART0_Close 
* 功能：                关闭串口并返回串口设备文件描述 
* 入口参数：        		fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2) 
* 出口参数：        		void 
*******************************************************************/  

void UART0_Close(int fd)  
{  
	close(fd);  
}  

/******************************************************************* 
* 名称：                UART0_Set 
* 功能：                设置串口数据位，停止位和效验位 
* 入口参数：        		fd        串口文件描述符 
*                       speed     串口速度 
*                       flow_ctrl   数据流控制 
*                       databits   数据位   取值为 7 或者8 
*                       stopbits   停止位   取值为 1 或者2 
*                       parity     效验类型 取值为N,E,O,,S 
*出口参数：          	正确返回为TRUE(0), 错误返回为FALSE(-1)
*******************************************************************/  
int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)  
{  
	int   i;  
	int   status;  
	int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};  
	int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};  

	struct termios options;  

	/*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1. 
	*/  
	if  ( tcgetattr( fd,&options)  !=  0)  
	{  
		perror("SetupSerial 1");      
		return(FALSE);   
	}  

	//设置串口输入波特率和输出波特率  
	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)  
	{  
		if  (speed == name_arr[i])  
		{               
			cfsetispeed(&options, speed_arr[i]);   
			cfsetospeed(&options, speed_arr[i]);    
		}  
	}       

	//修改控制模式，保证程序不会占用串口  
	options.c_cflag |= CLOCAL;  
	//修改控制模式，使得能够从串口中读取输入数据  
	options.c_cflag |= CREAD;  

	//设置数据流控制  
	switch(flow_ctrl)  
	{  
		case 0 ://不使用流控制  
			options.c_cflag &= ~CRTSCTS;  
			break;     

		case 1 ://使用硬件流控制  
			options.c_cflag |= CRTSCTS;  
			break;  
		case 2 ://使用软件流控制  
			options.c_cflag |= IXON | IXOFF | IXANY;  
			break;  
	}  
	//设置数据位  
	//屏蔽其他标志位  
	options.c_cflag &= ~CSIZE;  
	switch (databits)  
	{    
		case 5    :  
			options.c_cflag |= CS5;  
			break;  
		case 6    :  
			options.c_cflag |= CS6;  
			break;  
		case 7    :      
			options.c_cflag |= CS7;  
			break;  
		case 8:      
			options.c_cflag |= CS8;  
			break;    
		default:     
			fprintf(stderr,"Unsupported data size\n");  
			return (FALSE);   
	}  
	//设置校验位  
	switch (parity)  
	{    
		case 'n':  
		case 'N': //无奇偶校验位。  
			options.c_cflag &= ~PARENB;   
			options.c_iflag &= ~INPCK;      
			break;   
		case 'o':    
		case 'O'://设置为奇校验      
			options.c_cflag |= (PARODD | PARENB);   
			options.c_iflag |= INPCK;               
			break;   
		case 'e':   
		case 'E'://设置为偶校验    
			options.c_cflag |= PARENB;         
			options.c_cflag &= ~PARODD;         
			options.c_iflag |= INPCK;        
			break;  
		case 's':  
		case 'S': //设置为空格   
			options.c_cflag &= ~PARENB;  
			options.c_cflag &= ~CSTOPB;  
			break;   
		default:    
			fprintf(stderr,"Unsupported parity\n");      
			return (FALSE);   
	}   
	// 设置停止位   
	switch (stopbits)  
	{    
		case 1:     
			options.c_cflag &= ~CSTOPB; break;   
		case 2:     
			options.c_cflag |= CSTOPB; break;  
		default:     
			fprintf(stderr,"Unsupported stop bits\n");   
			return (FALSE);  
	}  

	//修改输出模式，原始数据输出  
	options.c_oflag &= ~OPOST;  

	options.c_iflag &= ~(IXON | ICRNL); 

	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//我加的  
	//options.c_lflag &= ~(ISIG | ICANON);  

	//设置等待时间和最小接收字符  
	options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */    
	options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */  

	//如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读  
	tcflush(fd,TCIFLUSH);  

	//激活配置 (将修改后的termios数据设置到串口中）  
	if (tcsetattr(fd,TCSANOW,&options) != 0)    
	{  
		perror("com set error!\n");    
		return (FALSE);   
	}  
	return (TRUE);   
}  
/******************************************************************* 
* 名称：                UART0_Init() 
* 功能：                串口初始化 
* 入口参数：        		fd       :  文件描述符    
*               		speed  :  串口速度 
*                       flow_ctrl  数据流控制 
*               		databits   数据位   取值为 7 或者8 
*          				stopbits   停止位   取值为 1 或者2 
*                     	parity     效验类型 取值为N,E,O,,S 
*                       
*出口参数：              正确返回为TRUE(0), 错误返回为FALSE(-1)
*******************************************************************/  
int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)  
{  
	int err;  
	//设置串口数据帧格式  
	if (UART0_Set(fd,speed,flow_ctrl,databits,stopbits,parity) == FALSE)  
	{                                                           
		return FALSE;  
	}  
	else  
	{  
		return  TRUE;  
	}  
}  

/******************************************************************* 
* 名称：                  	UART0_Recv 
* 功能：               		接收串口数据 
* 入口参数：        			fd                  :文件描述符     
*                          	rcv_buf     :接收串口中数据存入rcv_buf缓冲区中 
*                       	data_len    :一帧数据的长度 
* 出口参数：        			正确返回为1，错误返回为0 
*******************************************************************/  
int UART0_Recv(int fd, unsigned char *rcv_buf,int data_len)  
{  
	int len,fs_sel;  
	fd_set fs_read;  

	struct timeval time;  

	FD_ZERO(&fs_read);  
	FD_SET(fd,&fs_read);  

	time.tv_sec = 10;  
	time.tv_usec = 0;  

	//使用select实现串口的多路通信  
	fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);  
	if(fs_sel)  
	{	  
		len = read(fd,rcv_buf,data_len);  
		printf("I am right!(version1.2) len = %d fs_sel = %d\n",len,fs_sel);  
		return len;  
	}  
	else  
	{  
		printf("Sorry,I am wrong!");  
		return FALSE;  
	}       
}  
/******************************************************************** 
* 名称：                  	UART0_Send 
* 功能：                		发送数据 
* 入口参数：        			fd                  :文件描述符     
*                      		send_buf    :存放串口发送数据 
*                       	data_len    :一帧数据的个数 
*出口参数：              正确返回为TRUE(0), 错误返回为FALSE(-1)
*******************************************************************/  
int UART0_Send(int fd, char *send_buf,int data_len)  
{  
	int len = 0;  

	len = write(fd,send_buf,data_len);  
	if (len == data_len )  
	{  
		return len;  
	}       
		else     
	{  

		tcflush(fd,TCOFLUSH);  
		return FALSE;  
	}  

}  
/******************************************************************** 
* 名称：                  	UART0_Encode 
* 功能：                		 
* 入口参数：        			fd                  :文件描述符     
*                      		buf   
*                       	 
* 出口参数：        		正确返回为1，错误返回为0 	 
*******************************************************************/  
int UART0_Encode(int fd, int *buf)
{	
	static   int num = 0;
	unsigned char recv_len = 0;
        unsigned char temp[1];
	unsigned char RS485_RX_BUF[24] = {0x00};
	unsigned short crcvalue = 0;
        int length = 0;
        bool bfindhead  = false;
        int nIndex = -1;
       
        while(length<24)
        {
              recv_len = read(fd, temp , 1);
              if(recv_len == 1)
              {
                RS485_RX_BUF[length]=temp[0];
                length ++;
              }
        }	
       	
       num++;


	for(int i =0; i<12; i++)   
        {
		if(RS485_RX_BUF[i] == 0x58 && RS485_RX_BUF[i+1] == 0x58)  
	        {
			bfindhead  = true;
	                nIndex = i;
	                break;
  		} 
        }          
        
        
        if(bfindhead)
        {
		bfindhead = false;
                printf("%d", num); 
		crcvalue = crc_chk(&RS485_RX_BUF[nIndex],10)&0xFFFF; //计算crc校验值
                
                for(int j=0; j<12; j++)
                { 
			printf("%0x ", RS485_RX_BUF[nIndex+j]);
                }
		if(crcvalue == ((RS485_RX_BUF[nIndex+10]<<8) | RS485_RX_BUF[nIndex+11]))
		{
                        printf("\r\n");
			//memcpy(&buf[0], &RS485_RX_BUF[nIndex+2],4);
			//memcpy(&buf[1], &RS485_RX_BUF[nIndex+6],4);
			buf[0] = (RS485_RX_BUF[nIndex+2]<<24) | (RS485_RX_BUF[nIndex+3]<<16) | (RS485_RX_BUF[nIndex+4]<<8) | (RS485_RX_BUF[nIndex+5]);  //获取左轮编码值
			buf[1] = (RS485_RX_BUF[nIndex+6]<<24) | (RS485_RX_BUF[nIndex+7]<<16) | (RS485_RX_BUF[nIndex+8]<<8) | (RS485_RX_BUF[nIndex+9]);  //获取右轮编码值
		}
		else //crc出错
		{
			printf("UART0_Recv crc_chk false!"); 
			return FALSE; 
		}
	
	}
        else
        {
                return FALSE;
        }

       
           
	/*if(recv_len >= 12 )                            //编码器接收完成
	{
                
		if(RS485_RX_BUF[0] == 0x58 && RS485_RX_BUF[1] == 0x58 )                //检查接收数据头
		{  
                        printf("%d", num); 
			crcvalue = crc_chk(RS485_RX_BUF,10)&0xFFFF; //计算crc校验值
			if(crcvalue == ((RS485_RX_BUF[10]<<8) | RS485_RX_BUF[11]))
			{
				//memcpy(&buf[0], &RS485_RX_BUF[1],4);
				//memcpy(&buf[1], &RS485_RX_BUF[5],4);
                                //buf[0] = 0;
				buf[0] = (RS485_RX_BUF[2]<<24) | (RS485_RX_BUF[3]<<16) | (RS485_RX_BUF[4]<<8) | (RS485_RX_BUF[5]);  //获取左轮编码值
				buf[1] = (RS485_RX_BUF[6]<<24) | (RS485_RX_BUF[7]<<16) | (RS485_RX_BUF[8]<<8) | (RS485_RX_BUF[9]);  //获取右轮编码值
			}
			else //crc出错
			{
				printf("UART0_Recv crc_chk false!"); 
				return FALSE; 
			}					
		}
		else  //检查接收数据头出错
		{
			printf("UART0_Recv RS485_RX_BUF[0] false! RS485_RX_BUF[0] = %d.\n",RS485_RX_BUF[0]);  
		    return FALSE;
		}
	}
	else  //编码器接收数据长度出错
	{
		printf("UART0_Recv recv_len false! recv_len = %d.\n",recv_len);  
		return FALSE; 		
	}*/
	
		
	return  TRUE;	
}


/******************************************************************** 
* 名称：                  	crc_chk 
* 功能：                		 
* 入口参数：        		data                  :文件描述符     
*                      		length    :
* 出口参数：        			 返回校验值
*******************************************************************/  
//*
unsigned short crc_chk(unsigned char *data,int length)
{
	 
	 unsigned short reg_crc=0;
	 
	 while(length--)
	 {
		  reg_crc += (*data++);
	 }
	return reg_crc;
}
//*/
