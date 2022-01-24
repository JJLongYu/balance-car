# -两轮自平衡小车
# 1.概述
***
本项目基于STM32F10X系列单片机，使用MPU6050作为姿态传感器，对小车当前状态原始值进行进行读取后通过二维卡尔曼滤波进行滤波处理。采用PID算法处理数据，通过单片机输出PWM来控制电机的转速和方向，从而使小车保持平衡，采用OLED12864将数据进行显示。![enter image description here](https://github.com/JJLongYu/balance-car/blob/main/%E5%9B%BE%E7%89%87/balance%20car.jpg?raw=true)


# 2.实现方法及步骤
## 2.1 硬件部分
***
系统以STM32F103RCT6为主控芯片，使用姿态传感器MPU6050读取小车的当前倾角，使用滤波算法处理数据。然后进行PID算法计算，输出PWM值，通过PWM控制小车的电机转动，并且将PWM,角度，转速值在OLED12864上显示，并上传到匿名上位机软件里，显示三个数据对应曲线变化。供电部分系统采用三节锂电池，DC-DC和1117-3.3V稳压器件。
![enter image description here](https://github.com/JJLongYu/balance-car/blob/main/%E5%9B%BE%E7%89%87/banlancecar%20-%20V2.0.png?raw=true)
## 2.2软件部分
***
小车程序流程图
![enter image description here](https://github.com/JJLongYu/balance-car/blob/main/%E5%9B%BE%E7%89%87/%E8%BD%AF%E4%BB%B6%E6%B5%81%E7%A8%8B.png?raw=true)

### 2.2.1  数据链路传输
***
1. 通过控制2401无线通信模块的收发实现小车和遥控器之间的通信
```c
/*********************************************************************************************************
*函数：nRF24L01_RxPacket(unsigned char* rx_buf)
*输入：unsigned char* rx_buf 要发送的数据
*输出：无
*功能：发送 tx_buf中数据
*********************************************************************************************************/
void nRF24L01_TxPacket(unsigned char * tx_buf)
{
	NRF2401_CE_LOW;			//StandBy I模式	
	SPI_Write_Buf(NRF24L01_WRITE_REG + RX_ADDR_P0,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH); // 装载接收端地址
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // 装载数据	
	SPI_RW_Reg(NRF24L01_WRITE_REG + CONFIG, 0x0e);   		 // IRQ收发完成中断响应，16位CRC，主发送
	NRF2401_CE_HIGH;		 //置高CE，激发数据发送
	delay_us(10);
}

/*********************************************************************************************************
*函数：unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
*输入：unsigned char* rx_buf 要放入数据缓冲区的数据
*输出：revale 读取完成标志
*功能：数据读取后放如rx_buf接收缓冲区中
*********************************************************************************************************/
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
{
  unsigned char revale=0,sta=0;
	sta=SPI_Read(STATUS);	// 读取状态寄存其来判断数据接收状况
	if((sta&0x40)==0x40)				// 判断是否接收到数据
	{
		NRF2401_CE_LOW ; 			//使能待机
		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
		revale =1;			//读取数据完成标志
		
		SPI_RW_Reg(NRF24L01_WRITE_REG+STATUS,sta);   //接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标志
		SPI_RW_Reg(FLUSH_RX,0xff);	//清除接收缓冲器
		NRF2401_CE_HIGH; 			
	}
	return revale;
}
```
2. 通过串口实现遥控器与上位机的通信
```c
void ANOTCV6_Data32_10(int32_t Data[],int32_t Kalman[],int32_t FIR[],int32_t Aver[])
{
	uint8_t sum=0,i;
	uint8_t buf[48]={0};
	for(i=0;i<48;i=i+16)
	{
		buf[i]  =BYTE3(Data[i/16]);
		buf[i+1]=BYTE2(Data[i/16]);//原始数据
		buf[i+2]=BYTE1(Data[i/16]);
		buf[i+3]=BYTE0(Data[i/16]);//原始数据		
		
		buf[i+4]=BYTE3(Kalman[i/16]);
		buf[i+5]=BYTE2(Kalman[i/16]);//卡尔曼滤波
		buf[i+6]=BYTE1(Kalman[i/16]);
		buf[i+7]=BYTE0(Kalman[i/16]);//卡尔曼滤波			
	}	

	sum=sum+0xAA+0x05+0XAF+0XF1+48;
	
	USART_SendData(USART1,0xAA);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	USART_SendData(USART1,0x05);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	USART_SendData(USART1,0XAF);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);	
	USART_SendData(USART1,0XF1);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);	
	USART_SendData(USART1,48);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);	

	for(i=0;i<48;i++)
	{
		USART_SendData(USART1,buf[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
		sum=sum+buf[i];
	}
	USART_SendData(USART1,sum);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
}
```
### 2.2.2 PID控制
***
 ![enter image description here](https://github.com/JJLongYu/balance-car/blob/main/%E5%9B%BE%E7%89%87/pid.png?raw=true)
```c
/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle；Gyro：angular velocity
Output  : balance：Vertical control PWM
函数功能：直立PD控制		
入口参数：Angle:角度；Gyro：角速度
返回  值：balance：直立控制PWM
**************************************************************************/	

float Balance_Kp=Balance_Kp_Preset,Balance_Ki=Balance_Ki_Preset,Balance_Kd=Balance_Kd_Preset,Velocity_Kp=Velocity_Kp_Preset,Velocity_Ki=Velocity_Ki_Preset,Turn_Kp=4200,Turn_Kd=50;//PID参数（放大100倍）

int Balance(float Angle,float Gyro)
{  
	float Angle_bias,Gyro_bias,Angle_Integral;
	int balance;
	Angle_bias=Middle_angle-Angle;                       				//求出平衡的角度中值 和机械相关
	Gyro_bias=0-Gyro; 
	Angle_Integral+=Angle_bias;
	balance=Balance_Kp*Angle_bias+Angle_Integral*Balance_Ki-Gyro_bias*Balance_Kd; //计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	return balance;
}

	  int go=0;
/**************************************************************************
Function: Speed PI control
Input   : encoder_left：Left wheel encoder reading；encoder_right：Right wheel encoder reading
Output  : Speed control PWM
函数功能：速度控制PWM		
入口参数：encoder_left：左轮编码器读数；encoder_right：右轮编码器读数
返回  值：速度控制PWM
**************************************************************************/
//修改前进后退速度，请修改Target_Velocity，比如，改成60就比较慢了
int Velocity(int encoder_left,int encoder_right)
{  
    static float velocity,Encoder_Least,Encoder_bias,Movement;
	  static float Encoder_Integral,Target_Velocity;

		if(go_flag==1)
		{
			go = -movement;
			turn_xx = 0;
			Velocity_Kp = 500;
		}
		else if(go_flag==3)
		{
			go = movement;
			turn_xx = 0;
			Velocity_Kp = 500;
		}
	  //================遥控前进后退部分====================// 
#if 0
//		if(Flag_follow==1||Flag_avoid==1) Target_Velocity = 55; //如果进入跟随/避障模式,降低速度
//		else 											        Target_Velocity = 110;
//		if(Flag_front==1)    	Movement=Target_Velocity/Flag_velocity;	  //收到前进信号
//		else if(Flag_back==1)	Movement=-Target_Velocity/Flag_velocity;  //收到后退信号
//	  else  Movement=0;	
//	
//   //=============超声波功能（跟随/避障）==================// 
//	  if(Flag_follow==1&&(Distance>200&&Distance<500)&&Flag_Left!=1&&Flag_Right!=1) //跟随
//			 Movement=Target_Velocity/Flag_velocity;
//		if(Flag_follow==1&&Distance<200&&Flag_Left!=1&&Flag_Right!=1) 
//			 Movement=-Target_Velocity/Flag_velocity;
//		if(Flag_avoid==1&&Distance<450&&Flag_Left!=1&&Flag_Right!=1)  //超声波避障
//			 Movement=-Target_Velocity/Flag_velocity;
#endif		
   //================速度PI控制器=====================//	
		Encoder_Least =0-(encoder_left+encoder_right);                    //获取最新速度偏差=目标速度（此处为零）-测量速度（左右编码器之和） 
		Encoder_bias *= 0.8;		                                          //一阶低通滤波器       
		Encoder_bias += Encoder_Least*0.2;	                              //一阶低通滤波器，减缓速度变化 
		Encoder_Integral +=Encoder_bias;                                  //积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral+go;                       //接收遥控器数据，控制前进后退
		if(Encoder_Integral>5000)  	Encoder_Integral=5000;             //积分限幅
		if(Encoder_Integral<-5000)	  Encoder_Integral=-5000;            //积分限幅	
		velocity=+Encoder_bias*Velocity_Kp+Encoder_Integral*Velocity_Ki;  //速度控制	
//		if(Turn_Off(Angle_Balance,Voltage)==1) Encoder_Integral=0;//电机关闭后清除积分
	  return velocity;
}

```
# 3.实现效果
***
还没来的及拍视频，去学校了在拍。。。。
