#include <stdio.h>
#include <stdarg.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_adc.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_rtc.h>


#include "./usb_cdc_device/usbd_usr.h"
#include "./usb_cdc_device/usbd_cdc_core.h"
#include "./usb_cdc_device/usb_conf.h"
#include "./usb_cdc_device/usbd_desc.h"
#include "./usb_cdc_device/usbd_cdc_vcp.h"

#include "tiny_printf.h"

#define PCA9685_adrr 0x80
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE


#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

/*
 * 500~2500 us to 0~180 Angle
 */
long in_min = 500;
long in_max = 2500;
long out_min = 0;
long out_max = 180;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

volatile uint32_t TimingDelay;

volatile uint32_t micros = 0;

void Delay(__IO uint32_t nTime)
{
   TimingDelay = nTime;
   while(TimingDelay){
   }
}

void SysTick_Handler(void)
{
   if(TimingDelay){
      --TimingDelay;
   }
   ++micros;
}

/* Private */
#define USB_VCP_RECEIVE_BUFFER_LENGTH		128
uint8_t INT_USB_VCP_ReceiveBuffer[USB_VCP_RECEIVE_BUFFER_LENGTH];
uint32_t int_usb_vcp_buf_in, int_usb_vcp_buf_out, int_usb_vcp_buf_num;
USB_VCP_Result USB_VCP_INT_Status;
//extern LINE_CODING linecoding;
uint8_t USB_VCP_INT_Init = 0;
USB_OTG_CORE_HANDLE	USB_OTG_dev;

extern uint8_t INT_USB_VCP_ReceiveBuffer[USB_VCP_RECEIVE_BUFFER_LENGTH];

USB_VCP_Result USBVCPInit(void)
{
   USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_FS
	 USB_OTG_FS_CORE_ID,
#else
	 USB_OTG_HS_CORE_ID,
#endif
	 &USR_desc, 
	 &USBD_CDC_cb, 
	 &USR_cb);   

   /* Reset buffer counters */
   int_usb_vcp_buf_in = 0;
   int_usb_vcp_buf_out = 0;
   int_usb_vcp_buf_num = 0;

   /* Initialized */
   USB_VCP_INT_Init = 1;

   return USB_VCP_OK;
}

USB_VCP_Result USB_VCP_GetStatus(void) {
   if (USB_VCP_INT_Init) {
      return USB_VCP_INT_Status;
   }
   return USB_VCP_ERROR;
}

USB_VCP_Result USB_VCP_Getc(uint8_t* c) {
   /* Any data in buffer */
   if (int_usb_vcp_buf_num > 0) {
      /* Check overflow */
      if (int_usb_vcp_buf_out >= USB_VCP_RECEIVE_BUFFER_LENGTH) {
	 int_usb_vcp_buf_out = 0;
      }
      *c = INT_USB_VCP_ReceiveBuffer[int_usb_vcp_buf_out];
      INT_USB_VCP_ReceiveBuffer[int_usb_vcp_buf_out] = 0;

      /* Set counters */
      int_usb_vcp_buf_out++;
      int_usb_vcp_buf_num--;

      /* Data OK */
      return USB_VCP_DATA_OK;
   }
   *c = 0;
   /* Data not ready */
   return USB_VCP_DATA_EMPTY;
}

USB_VCP_Result USB_VCP_Putc(volatile char c) {
   uint8_t ce = (uint8_t)c;

   /* Send data over USB */
   VCP_DataTx(&ce, 1);

   /* Return OK */
   return USB_VCP_OK;
}

USB_VCP_Result USB_VCP_Puts(char* str) {
   while (*str) {
      USB_VCP_Putc(*str++);
   }

   /* Return OK */
   return USB_VCP_OK;
}

USB_VCP_Result INT_USB_VCP_AddReceived(uint8_t c) {
   /* Still available data in buffer */
   if (int_usb_vcp_buf_num < USB_VCP_RECEIVE_BUFFER_LENGTH) {
      /* Check for overflow */
      if (int_usb_vcp_buf_in >= USB_VCP_RECEIVE_BUFFER_LENGTH) {
	 int_usb_vcp_buf_in = 0;
      }
      /* Add character to buffer */
      INT_USB_VCP_ReceiveBuffer[int_usb_vcp_buf_in] = c;
      /* Increase counters */
      int_usb_vcp_buf_in++;
      int_usb_vcp_buf_num++;

      /* Return OK */
      return USB_VCP_OK;
   }

   /* Return Buffer full */
   return USB_VCP_RECEIVE_BUFFER_FULL;
}

void Init_I2C2(){

   GPIO_InitTypeDef GPIO_InitStructure;
   I2C_InitTypeDef I2C_InitStruct;

   RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

   I2C_InitStruct.I2C_ClockSpeed = 100000;
   I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
   I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; 
   I2C_InitStruct.I2C_OwnAddress1 = 0x00;
   I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
   I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
   I2C_Init(I2C2, &I2C_InitStruct);

   I2C_Cmd(I2C2, ENABLE);

}

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
   // wait until I2C1 is not busy anymore
   while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

   // Send I2C1 START condition 
   I2C_GenerateSTART(I2Cx, ENABLE);

   // wait for I2C1 EV5 --> Slave has acknowledged start condition
   while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

   // Send slave Address for write 
   I2C_Send7bitAddress(I2Cx, address, direction);

   /* wait for I2C1 EV6, check if 
    * either Slave has acknowledged Master transmitter or
    * Master receiver mode, depending on the transmission
    * direction
    */ 
   if(direction == I2C_Direction_Transmitter){
      while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
   } else if(direction == I2C_Direction_Receiver){
      while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
   }
}

/* This function transmits one byte to the slave device
 * Parameters:
 * I2Cx --> the I2C peripheral e.g. I2C1 
 * data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
   I2C_SendData(I2Cx, data);
   // wait for I2C1 EV8_2 --> byte has been transmitted
   while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/* This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
   uint8_t data;
   // enable acknowledge of recieved data
   I2C_AcknowledgeConfig(I2Cx, ENABLE);
   // wait until one byte has been received
   while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
   // read data from I2C data register and return data byte
   data = I2C_ReceiveData(I2Cx);
   return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data 
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
   uint8_t data;
   // disabe acknowledge of received data
   // nack also generates stop condition after last byte received
   // see reference manual for more info
   I2C_AcknowledgeConfig(I2Cx, DISABLE);
   I2C_GenerateSTOP(I2Cx, ENABLE);
   // wait until one byte has been received
   while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
   // read data from I2C data register and return data byte
   data = I2C_ReceiveData(I2Cx);
   return data;
}

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
   // Send I2C1 STOP Condition 
   I2C_GenerateSTOP(I2Cx, ENABLE);
}

void I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data) {
   I2C_start(I2Cx, address, I2C_Direction_Transmitter);
   I2C_write(I2Cx, reg);
   I2C_write(I2Cx, data);
   I2C_stop(I2Cx);
}

void I2C_WriteMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
   I2C_start(I2Cx, address, I2C_Direction_Transmitter);
   I2C_write(I2Cx, reg);
   while (count--) {
      I2C_write(I2Cx, *data++);
   }
   I2C_stop(I2Cx);
}

void PCA9685_Write(unsigned char reg,unsigned char data)
{
   I2C_start(I2C2,PCA9685_adrr,I2C_Direction_Transmitter);
   I2C_write(I2C2,reg);
   I2C_write(I2C2,data);
   I2C_stop(I2C2);
}

u8 PCA9685_Read(unsigned char reg)
{
   uint8_t res;
   I2C_start(I2C2,PCA9685_adrr,I2C_Direction_Transmitter);
   I2C_write(I2C2,reg);
   I2C_stop(I2C2);
   I2C_start(I2C2,PCA9685_adrr|0X01,I2C_Direction_Receiver);
   res = I2C_read_nack(I2C2);       
   I2C_stop(I2C2);             
   return res;  
}

void SetPWMFreq(uint8_t freq)
{
   uint8_t prescale,oldmode,newmode;
   double prescaleval;
   //prescaleval = 25000000.0/(4096.0*freq*0.915);
   //prescale = (uint8_t)floor(prescaleval+0.5)-1;
   
   prescale = 110;	// (25000000.0/(4096.0*60*0.915) ) + 0.5 - 1

   oldmode = PCA9685_Read(PCA9685_MODE1);
   newmode = (oldmode&0x7F) | 0x10; // sleep , !PCA9685_MODE1_RESTART = !0x80 = 0x7F, PCA9685_MODE1_SLEEP = 0x10.
   PCA9685_Write(PCA9685_MODE1, newmode); // go to sleep
   PCA9685_Write(PCA9685_PRESCALE, prescale); // set the prescaler
   PCA9685_Write(PCA9685_MODE1, oldmode);
   Delay(5000);
   PCA9685_Write(PCA9685_MODE1, oldmode | 0xA1); // 0xA1 == PCA9685_MODE1_RESTART = 0x80 & PCA9685_MODE1_AI = 0x20 & PCA9685_MODE1_ALLCALL = 0x01.
}

void SetPWM(u8 num, u16 on, u16 off) 
{
   PCA9685_Write(LED0_ON_L+4*num,on);
   PCA9685_Write(LED0_ON_H+4*num,on>>8);
   PCA9685_Write(LED0_OFF_L+4*num,off);
   PCA9685_Write(LED0_OFF_H+4*num,off>>8);
}

uint16_t Calculate_PWM(u8 angle){
   //return (uint16_t)(204.8*(0.5+angle*1.0/(out_max / 2.0)));	//50Hz
   return (uint16_t)(245.85*(0.5+angle*1.0/(out_max / 2.0)));	//60Hz
}

void Init_PCA9685()
{
   PCA9685_Write(PCA9685_MODE1,0x00);
   Delay(5000);

   SetPWMFreq(60);	//60Hz
   Delay(5000);
}

int main(void)
{
   if(SysTick_Config(SystemCoreClock / 1000 / 1000)){
      while(1){}
   }

   USBVCPInit();
   Init_I2C2();

   Init_PCA9685();

   int id_state = 0;

   char str[255] = {""};  
   int index = 0;

   while(1){

      if(USB_VCP_GetStatus() == USB_VCP_CONNECTED) {

	 uint8_t c;
	 if (USB_VCP_Getc(&c) == USB_VCP_DATA_OK) {
	    USB_VCP_Putc(c);
	    if(c == '[' && id_state == 0){
	       ++id_state;
	    }
	    else if(c != ']' && id_state == 1){
	       str[index++] = c;
	    }	   
	    else if(c == ']' && id_state == 1){
	       ++id_state;
	    }

	    if(id_state == 2){
	       int servos = 0;
	       char *p = NULL;
	       p = strtok(str,",");

	       uint16_t pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
	       SetPWM(0x0,0,pwm);

	       ++servos;
	       while((p = strtok(NULL,","))){
		  USB_VCP_Puts(p);
		  int pulse = atoi(p);
		  switch(servos){
		     case 0x1:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0x1,0,pwm);	
			   break;
			}
		     case 0x2:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0x2,0,pwm);	
			   break;
			}
		     case 0x3:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0x3,0,pwm);	
			   break;
			}
		     case 0x4:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0x4,0,pwm);	
			   break;
			}
		     case 0x5:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0x5,0,pwm);	
			   break;
			}
		     case 0x6:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0x6,0,pwm);	
			   break;
			}
		     case 0x7:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0x7,0,pwm);	
			   break;
			}
		     case 0x8:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0x8,0,pwm);	
			   break;
			}
		     case 0x9:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0x9,0,pwm);	
			   break;
			}
		     case 0xA:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0xA,0,pwm);	
			   break;
			}
		     case 0xB:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0xB,0,pwm);	
			   break;
			}
		     case 0xC:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0xC,0,pwm);	
			   break;
			}
		     case 0xD:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0xD,0,pwm);	
			   break;
			}
		     case 0xE:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0xE,0,pwm);	
			   break;
			}
		     case 0xF:
			{
			   pwm = Calculate_PWM(map(atoi(p),in_min,in_max,out_min,out_max));
			   SetPWM(0xF,0,pwm);	
			   break;
			}
		  }
		  ++servos;
	       }
	       index = 0;
	       id_state = 0;
	       memset(str,0,sizeof(str));
	    }
	 }

      }
      Delay(1000);
   }

   return(0); // System will implode
}    
