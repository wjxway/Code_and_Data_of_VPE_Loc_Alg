#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "pwm.h"
#include "led.h"
#include "adc.h"
#include "dac.h"
#include "stdio.h"
#include "math.h"
#include "myiic.h"
#include "HMC5883L.h"
#include "stdbool.h"
#include "timer.h"


//constants and variables
#define pi 3.1415926535

extern bool trigger;

float ang[24]={258.212,259.536,263.454,269.81,278.323,288.576,300.,311.876,323.365,333.569,341.616,346.773,348.55,346.773,341.616,333.569,323.365,311.876,300.,288.576,278.323,269.81,263.454,259.536};
float anguni[24]={300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300,300};
	
//float angtest[24]={0,0,0,0,0,0,0,0,0,0,0,0,348.55,0,0,0,0,0,0,0,0,0,0,0};
//Note that the virtual particle number is stored in the way {x+,x-,y+,y-}
float VP[2]={1,1};
float angle0[2]={0,pi};

//x = scale*log(VP[0] / VP[1]);
float scale=3.5;


//note:
//background light, morning, indoor: 200~300
//300 emittion strength, self-emit: 65~90
//300 emittion strength, 1 emitting robot, center-center distance~20cm: 90~120

//self emit-receive strength in standard, uniaxial emittion situation.
int selfemit=70;

float chop(float s)
{
	if(s<0) return 0;
	else if(s>1) return 1;
	else return s;
}


//constant definition 2
bool flash_both[2] = { 1,1 };
bool flash_green[2] = { 1,0 };
bool flash_blue[2] = { 0,1 };

//flash light! half period in ms
void flash(unsigned short int period, unsigned short int times, bool* state)
{
	short int i=0;
	for (i = 0; i < times; i++)
	{
		//lit green LED
		if (state[0] == 1)  TIM_SetCompare1(TIM10,2000);
		//lit blue LED
		if (state[1] == 1)  TIM_SetCompare1(TIM11,2000);
		delay_ms(period / 2);
		//quench green LED
		if (state[0] == 1)  TIM_SetCompare1(TIM10,0);
		//quench blue LED
		if (state[1] == 1)  TIM_SetCompare1(TIM11,0);
		delay_ms(period / 2);
	}
}

void flash1(unsigned short int period,unsigned short int times)
{
	short int i=0;
	for(i=0;i<times;i++)
	{
		TIM_SetCompare1(TIM10,2000);
		TIM_SetCompare1(TIM11,0);
		delay_ms(period/2);
		TIM_SetCompare1(TIM10,0);
		TIM_SetCompare1(TIM11,2000);
		delay_ms(period/2);
	}
	TIM_SetCompare1(TIM11,0);
	delay_ms(period/2);
}

//this function is only used to emit a uni-axis strength simutaniously, only used in time synchronization
void uniemit(unsigned short int strength)
{
	TIM_SetCompare1(TIM2,2000-strength);
	TIM_SetCompare3(TIM3,2000-strength);
	TIM_SetCompare4(TIM3,2000-strength);
	TIM_SetCompare1(TIM1,2000-strength);
	TIM_SetCompare2(TIM1,2000-strength);
	TIM_SetCompare4(TIM1,2000-strength);
	TIM_SetCompare3(TIM1,2000-strength);
	TIM_SetCompare3(TIM2,2000-strength);
	TIM_SetCompare4(TIM2,2000-strength);
	TIM_SetCompare1(TIM4,2000-strength);
	TIM_SetCompare2(TIM4,2000-strength);
	TIM_SetCompare3(TIM4,2000-strength);
	TIM_SetCompare4(TIM4,2000-strength);
	TIM_SetCompare1(TIM8,2000-strength);
	TIM_SetCompare2(TIM8,2000-strength);
	TIM_SetCompare3(TIM8,2000-strength);
	TIM_SetCompare4(TIM8,2000-strength);
	TIM_SetCompare2(TIM2,2000-strength);
	TIM_SetCompare1(TIM3,2000-strength);
	TIM_SetCompare1(TIM9,2000-strength);
	TIM_SetCompare1(TIM5,2000-strength);
	TIM_SetCompare2(TIM5,2000-strength);
	TIM_SetCompare3(TIM5,2000-strength);
	TIM_SetCompare4(TIM5,2000-strength);
}

//light emittion
unsigned short emit(const float *ratios, const float strength,const float angle,const unsigned short int temit)
{
	float rot=angle/pi*12;
  float min,max;
	unsigned short int i, emits[24];

	max=(min=(short int)floor(rot))+1;
	
	for(i=0;i<24;i++) emits[i]=strength*((float)ratios[(i-(short int)min+96)%24]*(max-rot)+(float)ratios[(i-(short int)max+96)%24]*(rot-min));
	
	TIM_SetCompare1(TIM2,2000-emits[0]);
	TIM_SetCompare3(TIM3,2000-emits[1]);
	TIM_SetCompare4(TIM3,2000-emits[2]);
	TIM_SetCompare1(TIM1,2000-emits[3]);
	TIM_SetCompare2(TIM1,2000-emits[4]);
	TIM_SetCompare4(TIM1,2000-emits[5]);
	TIM_SetCompare3(TIM1,2000-emits[6]);
	TIM_SetCompare3(TIM2,2000-emits[7]);
	TIM_SetCompare4(TIM2,2000-emits[8]);
	TIM_SetCompare1(TIM4,2000-emits[9]);
	TIM_SetCompare2(TIM4,2000-emits[10]);
	TIM_SetCompare3(TIM4,2000-emits[11]);
	TIM_SetCompare4(TIM4,2000-emits[12]);
	TIM_SetCompare1(TIM8,2000-emits[13]);
	TIM_SetCompare2(TIM8,2000-emits[14]);
	TIM_SetCompare3(TIM8,2000-emits[15]);
	TIM_SetCompare4(TIM8,2000-emits[16]);
	TIM_SetCompare2(TIM2,2000-emits[17]);
	TIM_SetCompare1(TIM3,2000-emits[18]);
	TIM_SetCompare1(TIM9,2000-emits[19]);
	TIM_SetCompare1(TIM5,2000-emits[20]);
	TIM_SetCompare2(TIM5,2000-emits[21]);
	TIM_SetCompare3(TIM5,2000-emits[22]);
	TIM_SetCompare4(TIM5,2000-emits[23]);
	
	//measure
	delay_ms(temit/2);
	i=Get_Adc_Average(10,1000);
	delay_ms(temit/2);
	uniemit(0);
	return i;
}

float getposition(float k )
{
	return k*log(VP[0] / VP[1]);
}

//time synchronization 1: when the first is lit, all lit up!
//max error (ms) is the max time error allowed when using this function.
//activation theshold means how much more the receiver should get to activate!
//if working as desired, the time of exiting timesc should be the same.
void timesync1(unsigned short int maxerror, unsigned short int acttheshold)
{
	uniemit(0);
	delay_ms(maxerror);
	unsigned short int val=0,bg=Get_Adc_Average(10,1000)+acttheshold;
	MYTIM_Int_Init(10*maxerror,8400);
	trigger=1;
	while((val<bg)&&trigger) val=Get_Adc_Average(10,100);
	uniemit(1000);
	if(maxerror<50)	delay_ms(50);
	else delay_ms(maxerror);
	uniemit(0);
}

void timesyncext(unsigned short int n, unsigned short int maxerror, unsigned int time, unsigned short int indicate)
{
	//blue LED indicator
	if(indicate) TIM_SetCompare1(TIM11,1000);
	
	unsigned short int cc=0,cerr=0;
	while(true)
	{
		if(!TRIG)
		{
			cc++;
			if(cc>=n) break;
		}
		else
		{
			if(cc!=0)
			{
				cerr++;
				if(cerr>maxerror) cc=cerr=0;
			}
		}
		delay_us(time);
	}
	
	//sync successful!
	if(indicate) flash(100,1,flash_both);
}


//output @ A7
void outputsig(int n)
{
	if(n)
	{
		DAC_SetChannel1Data(DAC_Align_12b_R, 4095);
		TIM_SetCompare1(TIM11,1000);
	}
	else
	{
		DAC_SetChannel1Data(DAC_Align_12b_R, 0);
		TIM_SetCompare1(TIM11,0);
	}
}

void outputval(unsigned short int val,unsigned int t)
{
	for(unsigned short int i=0;i<val+2;i++)
	{
		outputsig(1);
		delay_us(t);
		outputsig(0);
		delay_us(t);
	}
	delay_us(2*t);
}

void outputinfo(float val, unsigned int t, unsigned short int digits)
{
	TIM_SetCompare1(TIM10,1000);
	
	outputsig(1);
	delay_us(t);
	outputsig(0);
	delay_us(3*t);
	
	float v=val;
	
	if(v<0)
	{
		outputval(0,t);
		v=-v;
	}
	else outputval(1,t);
	
	for(unsigned short int i=0;i<digits;i++)
	{
		outputval((int)floor(v),t);
		v=(v-floor(v))*10.0;
	}
	
	outputsig(1);
	delay_us(t);
	outputsig(0);
	delay_us(3*t);
	
	TIM_SetCompare1(TIM10,0);
}	

//temit is the time used in EACH emittion-detection event!
//loops1 indicates how many times should strength 1 emittion be sent and measured.
//loops2 indicates how many times should strength(VP) emittion be sent and measured per EACH loop1!!!
//evorate should be a number from 0~1, indicating the speed of changing the VP value! a typical value could be 0.04
void changeval(float *VPlist,float *ratios,const unsigned short int temit, const unsigned short int loops1, const unsigned short int loops2, const float evorate)
{
	unsigned short int bg=0,i=0,j=0,k=0;
	unsigned short int tmp[2]={0,0};
  unsigned int tot[2]={0,0};
	
	float pt=0;
	
	for(i=0;i<loops1;i++)
	{
		bg=emit(ratios,0,0,temit);//reset emittion strength to 0.
		
		for(k=0;k<2;k++) tot[k]+=tmp[k]=(emit(ratios,1,angle0[k]+pi,temit)-bg);
		
		for(j=0;j<loops2;j++) for(k=0;k<2;k++) VPlist[k]=VPlist[k]*chop(1-evorate*tmp[k]/100.0)+evorate*(emit(ratios,VPlist[k],angle0[k],temit)-bg)/100.0;
		//timesync1(10,40);
	}
	
	for(k=0;k<2;k++) tot[k]/=loops1;
	
	return;
}

//correction functions
//recommend settings:
//loops1:4
//loops2:10
//evorate: 0.04

void modifyVP(const unsigned short int temit,const unsigned short int loops1,const unsigned short int loops2,const float evorate)
{
	float VPavg[2]={0,0};
	unsigned short int bg=0,i=0,j=0,k=0;
	unsigned short int tmp[2]={0,0};
	
	for(i=0;i<2;i++) VPavg[i]=VP[i];
	
	for(i=0;i<loops1;i++)
	{
		timesyncext(10,2,200,1);
		bg=emit(anguni,0,0,temit);
		for(k=0;k<2;k++)
		{
			tmp[k]=(emit(anguni,1,0,temit)-bg);
			//if(tmp[k]>selfemit)	evoratio[k]*=chop(1-evorate*(tmp[k]-selfemit)/100.0);
		}
		
		for(j=0;j<loops2;j++) for(k=0;k<2;k++) VPavg[k]=VPavg[k]*chop(1-evorate*tmp[k]/100.0)+evorate*(emit(anguni,VPavg[k],angle0[k],temit)-bg)/100.0;
		//timesync1(10,40);
	}
	
	for(i=0;i<2;i++) VP[i]/=VPavg[i];
}


int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(115200);//初始化串口波特率为115200
	
	unsigned short int i=0;
	
	//initializaton
	Adc_Init();
	Dac1_Init();
	IIC_Init();
	hmc_init();
	LED_Init();
  TIM_PWM_Init(2000-1,3-1);	//84M/84=1Mhz?????,????500,??PWM??? 1M/500=2Khz.

	uniemit(0);
	
	//confirm power-on
	delay_ms(500);
	flash(500,3,flash_both);

//	while(true)//test emittion
//	{
//		emit(angtest,1,angle0[0],100);
//	}
	
	for (unsigned long i = 1; i <= 100000; i++)
	{
		timesyncext(10,2,200,1);
		
		//modify VP value
		if(i%20==0) modifyVP(10,20,40,0.05);
		else//change VP value
		changeval(VP,anguni,10,2,5,0.02);
		
		//send info back to arduino
		outputinfo(getposition(1.0),5000,6);
	}
	
//	for(int i=1;i<10000;i++)//data transmittion test
//	{
//		outputsig(0);
//		timesyncext(15,2,200,1);
//		delay_ms(1000);
//		float val=(1.0-2.0*(float)(i%2))*((float)(i%7))/7.0;
//		
//		flash(100,5,flash_green);
//		outputinfo(val,5000,6);
//		flash1(100,5);
//	}
	
	//program end indicator
	uniemit(0);
	flash(500,20,flash_green);
	TIM_SetCompare1(TIM10,400);
	
	return 0;
}