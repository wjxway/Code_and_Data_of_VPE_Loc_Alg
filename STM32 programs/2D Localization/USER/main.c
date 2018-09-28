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
//Note that the virtual particle number is stored in the way {x+,x-,y+,y-}
float VP[4]={1,1,1,1};
float angle0[4]={-3.0*pi/4,1.0*pi/4,3.0*pi/4,-pi/4};

//times(dir, regionf1*(1 + tanh(ldir*s / regionf2))*s / (ldir + 0.0001))
float regionf1=4,regionf2=0.4;

//repulse1*tanh(repulse2*(fmax(x1,x2)-repulse3))*((float)(x1-x2)/(x1+x2))
float repulse1=200.0,repulse2=0.01,repulse3=120.0;

//x = scale*log(VP[0] / VP[1]);
float scale=3.2;


//note:
//background light, morning, indoor: 200~300
//300 emittion strength, self-emit: 65~90
//300 emittion strength, 1 emitting robot, center-center distance~20cm: 90~120
//300 emittion strength, 3 emitting robot, center-center distance~20cm: 
//300 emittion strength, 6 emitting robot, center-center distance~20cm: 

//self emit-receive strength in standard, uniaxial emittion situation.
int selfemit=70;


//geometry definitions
typedef struct 
{
	float x;
	float y;
}point;

point plus(point pt1, point pt2)
{
	point pt = { pt1.x + pt2.x,pt1.y + pt2.y };
	return pt;
}
point subtract(point pt1, point pt2)
{
	point pt = { pt1.x - pt2.x,pt1.y - pt2.y };
	return pt;
}
point times(point pt1, float t)
{
	point pt = { pt1.x*t,pt1.y*t };
	return pt;
}
float dot(point pt1, point pt2)
{
	return pt1.x*pt2.x + pt1.y*pt2.y;
}
float cross(point pt1, point pt2)
{
	return pt1.x*pt2.y - pt2.x*pt1.y;
}
float norm(point pt1)
{
	return sqrt(dot(pt1, pt1));
}

typedef struct
{
	point pt1;
	point pt2;
}line;





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

bool motor_foward[2] = { 1,1 };
bool motor_backward[2] = { 0,0 };
bool motor_right[2] = { 1,0 };
bool motor_left[2] = { 0,1 };

short int magnetic_zero[2] = { 0,0 };


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


//magnetic return angle
float magnetic_getangle()
{
	short int data[3]={0,0,0};
	HMC_READ0(data);
	return atan2(data[1] - magnetic_zero[1],data[0]-magnetic_zero[0]);
}

//function used to calibrate magnetic zero
void magnetic_calibrate(unsigned short int period,unsigned short int times)
{
	int64_t x=0,y=0,xx=0,xy=0,yy=0,xxx=0,xxy=0,xyy=0,yyy=0,temp=0;

	int64_t tempx=0,tempy=0;
	short int i=0;
	
	short int data[3]={0,0,0};
	
	//turn on green LED as indicator
	TIM_SetCompare1(TIM10,1000);

	for (i=0;i<times;i++)
	{
		//read x & y and store
		//hmc_read_XYZ(data);
    HMC_READ0(data);
		tempx=data[0];
    tempy=data[1];
		
		//addition
		x+=tempx;
		y+=tempy;
		xx+=tempx*tempx;
		xy+=tempx*tempy;
		yy+=tempy*tempy;
		xxx+=tempx*tempx*tempx;
		xxy+=tempx*tempx*tempy;
		xyy+=tempx*tempy*tempy;
		yyy+=tempy*tempy*tempy;
		
		//blue LED as measuring indicator

		delay_ms(period/4);
		TIM_SetCompare1(TIM11,2000);
		delay_ms(3*period/4);
		TIM_SetCompare1(TIM11,0);
	}
  x/=times;y/=times;xx/=times;xy/=times;yy/=times;xxx/=times;xxy/=times;xyy/=times;yyy/=times;
	
	temp = 2*((y*y-yy)*(x*x-xx) - (x*y - xy)*(x*y - xy));
	magnetic_zero[0] = (((yyy + xxy) - y*xx - y*yy)*(x*y - xy) - ((xxx + xyy) - x*yy - x*xx)*(y*y - yy))/temp;
	magnetic_zero[1] = (((yyy + xxy) - y*xx - y*yy)*(xx - x*x) - ((xxx + xyy) - x*yy - x*xx)*(xy - x*y))/temp;
	
	//turn off green LED
	 TIM_SetCompare1(TIM10,0);
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
	



//motor start
void motor_start(bool* state)
{
	//left
	if (state[0] == 1)
	{
	GPIO_ResetBits(GPIOD,GPIO_Pin_2);  
	GPIO_SetBits(GPIOD,GPIO_Pin_4);   
	}
	else
	{
	GPIO_SetBits(GPIOD,GPIO_Pin_2);  
	GPIO_ResetBits(GPIOD,GPIO_Pin_4);   
	}

	//right
	if (state[1] == 1)
	{
	GPIO_ResetBits(GPIOD,GPIO_Pin_1);	   
	GPIO_SetBits(GPIOD,GPIO_Pin_3); 
	}
	else
	{
	GPIO_SetBits(GPIOD,GPIO_Pin_1);	   
	GPIO_ResetBits(GPIOD,GPIO_Pin_3); 
	}
}
//motor stop
void motor_stop()
{
	//set all to low, abrupt stop!
	GPIO_ResetBits(GPIOD,GPIO_Pin_1);	   
	GPIO_ResetBits(GPIOD,GPIO_Pin_3); 
	GPIO_ResetBits(GPIOD,GPIO_Pin_2);  
	GPIO_ResetBits(GPIOD,GPIO_Pin_4);   
}
//find out how much ms it needs to move s cm
int motor_calib(float s)
{
	if(s>0.5) return (int)(-39 + 83*s);
	else return 1;
}
//general purpose move, directly input the target movement pos and initial angle given by magnetic sensor, then move!

float angledif(float angle1,float angle2,float period)
{
	float tmp = (angle1 - angle2) / period+0.5;
	return (tmp - floor(tmp)-0.5) * period;
}

void motor_move(point pos)
{
	unsigned short int maxerror=2500,acttheshold=80;
	uniemit(0);
	delay_ms(maxerror);
	unsigned short int val=0;
	
	MYTIM_Int_Init(20*maxerror,16800);
	trigger=1;
	//preprocesssing
	float targetangle=atan2(pos.y, pos.x);
	float rotangle = angledif(magnetic_getangle(),targetangle,2*pi);
	float tmp1,tmp2;
	bool fowardq=0;
	if(fabs(rotangle)<pi/2) fowardq=1;
	rotangle=angledif(rotangle,0,pi);
	
	//rotation
	if (rotangle > 0) motor_start(motor_right);
	else motor_start(motor_left);
	tmp1=angledif(magnetic_getangle(),targetangle,2*pi);
	tmp2=tmp1;
	while(tmp1*tmp2>0)
  {  
		tmp2=tmp1;
		tmp1=angledif(magnetic_getangle(),targetangle,2*pi);
	}
	motor_stop();
	delay_ms(150);
	//movement
	if(fowardq) motor_start(motor_foward);
	else motor_start(motor_backward);
	delay_ms(motor_calib(norm(pos)));
	motor_stop();
	
	unsigned short int bg=Get_Adc_Average(10,1000)+acttheshold;
	while((val<bg)&&trigger) val=Get_Adc_Average(10,100);
	uniemit(1000);
	flash(500, 3, flash_blue);
	uniemit(0);
	
}


//wait till start signal is sent.
void excite(unsigned short int checktime, unsigned short int continuous_time, unsigned short int error_tolerance)
{
	unsigned short int continuous = 0;
	unsigned short int error = 0;

	flash(500, 3, flash_blue);

	//turn on blue LED as indicator
	TIM_SetCompare1(TIM11,1000);

	while (1)
	{
		//insert excite	signal here!
		if (!TRIG)
		{
			continuous++;
			if (continuous >= continuous_time) break;
		}
		else
		{
			if (continuous != 0) error++;
			if (error > error_tolerance) error = continuous = 0;
		}
		delay_ms(checktime);
	}
	flash(500,3,flash_both);
}




//force definition
short int insideq(line* l, unsigned short int length, point pt)
{
	short int i, count = 0;
	float xx = floor(10000 * pt.x) / 10000 + 0.00005, yy = floor(10000 * pt.y) / 10000 + 0.00005;
	point t1={0,0}, t2={0,0};

	//note that we need all input points with maximum precision of 1/10000!
	for (i = 0; i < length; i++)
	{
		t1 = l[i].pt1;
		t2 = l[i].pt2;
		if ((t1.x != t2.x) && ((t1.x<xx&&t2.x>xx) || (t1.x > xx&&t2.x < xx)) && (t1.y*(t2.x - xx) + t2.y*(xx - t1.x)) / (t2.x - t1.x)>yy) count++;
	}
	if (count % 2) return -1;
	else return 1;
}

point nearest(line l, point pt)
{
	point p1 = l.pt1, p2 = l.pt2;
	float dot1=0, dot2=0;
	if ((dot1 = dot(subtract(pt, p1), subtract(p2, p1))) <= 0) return subtract(p1, pt);
	else if ((dot2 = dot(subtract(pt, p2), subtract(p1, p2))) <= 0) return subtract(p2, pt);
	else return subtract(times(plus(times(p2, dot1), times(p1, dot2)), 1 / (dot1 + dot2)), pt);
}

point nearestall(line *l, unsigned short int length, point pt)
{
	short int i=0;
	point pout = nearest(*l, pt), ptemp={0,0};
	float shortest = norm(pout), stemp=0;

	for (i = 1; i < length; i++)
	{
		stemp = norm(ptemp = nearest(l[i], pt));
		if (stemp < shortest)
		{
			shortest = stemp;
			pout = ptemp;
		}
	}

	return pout;
}

point regionf(line *l, unsigned short int length, point pt)
{
	short int s = insideq(l, length, pt);
	point dir = nearestall(l, length, pt);
	float ldir = norm(dir);

	return times(dir, regionf1*(1 + tanh(ldir*s / regionf2))*s / (ldir + 0.0001));
}

//repulse force calculation

float repulse(const int x1,const int x2)
{
	return repulse1*tanh(repulse2*(fmax(x1,x2)-repulse3))*((float)(x1-x2)/(x1+x2));
}

point getposition(float k )
{
	point pt={0,0};
	pt.x = k*log(VP[0] / VP[1]);
	pt.y = k*log(VP[2] / VP[3]);
	return pt;
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
	return i;
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




//temit is the time used in EACH emittion-detection event!
//loops1 indicates how many times should strength 1 emittion be sent and measured.
//loops2 indicates how many times should strength(VP) emittion be sent and measured per EACH loop1!!!
//evorate should be a number from 0~1, indicating the speed of changing the VP value! a typical value could be 0.04
point changeval(float *VPlist,float *ratios,const unsigned short int temit, const unsigned short int loops1, const unsigned short int loops2, const float evorate)
{
	unsigned short int bg=0,i=0,j=0,k=0;
	unsigned short int tmp[4]={0,0,0,0};
  unsigned int tot[4]={0,0,0,0};
	float magangle=0;
	
	point pt={0,0};
	
	for(i=0;i<loops1;i++)
	{
		bg=emit(ratios,0,0,temit);
		magangle=magnetic_getangle();
		for(k=0;k<4;k++) tot[k]+=tmp[k]=(emit(ratios,1,magangle+angle0[k]+pi,temit)-bg);
		
		for(j=0;j<loops2;j++) for(k=0;k<4;k++) VPlist[k]=VPlist[k]*chop(1-evorate*tmp[k]/100.0)+evorate*(emit(ratios,VPlist[k],magangle+angle0[k],temit)-bg)/100.0;
		timesync1(10,40);
	}
	
	for(k=0;k<4;k++) tot[k]/=loops1;
	
	pt.x=repulse(tot[1],tot[0]);
	pt.y=repulse(tot[3],tot[2]);
	
	timesync1(10,40);
	
	return pt;
}

//correction functions
//recommend settings:
//loops1:4
//loops2:10
//evorate: 0.04

void modifyVP(const unsigned short int temit,const unsigned short int loops1,const unsigned short int loops2,const float evorate)
{
	unsigned short int i;
	float VPavg[4];
	unsigned short int bg=0,j=0,k=0;
	unsigned short int tmp[4]={0,0,0,0};
	float evoratio[4]={1,1,1,1};
	float magangle=0;
	
	for(i=0;i<4;i++) VPavg[i]=VP[i];
	
	for(i=0;i<loops1;i++)
	{
		bg=emit(anguni,0,0,temit);
		magangle=magnetic_getangle();
		for(k=0;k<4;k++)
		{
			tmp[k]=(emit(anguni,1,magangle+angle0[k]+pi,temit)-bg);
			if(tmp[k]>selfemit)	evoratio[k]*=chop(1-evorate*(tmp[k]-selfemit)/100.0);
		}
		
		for(j=0;j<loops2;j++) for(k=0;k<4;k++) VPavg[k]=VPavg[k]*chop(1-evorate*tmp[k]/100.0)+evorate*(emit(anguni,VPavg[k],magangle+angle0[k],temit)-bg)/100.0;
		timesync1(10,40);
	}
	
	for(i=0;i<4;i++) VP[i]/=(1+(VPavg[i]-1)*(1-evoratio[i])*(1-evoratio[i]));
}


int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(115200);//初始化串口波特率为115200
	
	//shape definition


//	//disk
//	point p1={1.8, 0.}, p2={1.5588, 0.8999}, p3={0.9, 1.5588}, p4={0., 1.8}, p5={-0.9, 1.5588}, p6={-1.5589, 0.9}, p7={-1.8, 0.}, p8={-1.5589, -0.9}, p9={-0.9001, -1.5589}, p10={-0.0001, -1.8}, p11={0.8999, -1.5589}, p12={1.5588, -0.9001};
//	line l[12];
//	l[0]=(line){p1, p2}; l[1]=(line){p2, p3}; l[2]=(line){p3, p4}; l[3]=(line){p4, p5}; l[4]=(line){p5, p6}; l[5]=(line){p6, p7}; l[6]=(line){p7, p8}; l[7]=(line){p8, p9}; l[8]=(line){p9, p10}; l[9]=(line){p10, p11}; l[10]=(line){p11, p12}; l[11]=(line){p12, p1};
//	int length=12;

//  //non-convex triangle
//	point p1={-3.0709, 1.3206}, p2={-2.8502, 1.9999}, p3={-2.6683, 1.9999}, p4={-1.5928, 1.6505}, p5={0.5337, 1.5167}, p6={2.6266, 1.916}, p7={2.8051, 1.9999}, p8={2.8501, 1.9999}, p9={3.0597, 1.355}, p10={1.7836, -0.0039}, p11={0.7571, -1.871}, p12={0.3828, -3.3287}, p13={-0.3714, -3.3287}, p14={-0.4273, -2.8862}, p15={-1.2117, -0.9051}, p16={-2.4641, 0.8186};
//	line l[16];
//	l[0]=(line){p1, p16}; l[1]=(line){p16, p15}; l[2]=(line){p15, p14}; l[3]=(line){p14, p13}; l[4]=(line){p13, p12}; l[5]=(line){p12, p11}; l[6]=(line){p11, p10}; l[7]=(line){p10, p9}; l[8]=(line){p9, p8}; l[9]=(line){p8, p7}; l[10]=(line){p7, p6}; l[11]=(line){p6, p5}; l[12]=(line){p5, p4}; l[13]=(line){p4, p3}; l[14]=(line){p3, p2}; l[15]=(line){p2, p1};
//	int length=16;

//	//triangle
//point p1={2.3999, -1.4}, p2={-2.4, -1.4}, p3={0., 2.7999};
//line l[3];
//l[0]=(line){p1, p3}; l[1]=(line){p3, p2}; l[2]=(line){p2, p1};
//int length=3;

//K
point p1={-0.4167, 2.0833}, p2={-1.4584, 2.0833}, p3={-1.4584, -2.0834}, p4={-0.4167, -2.0834}, p5={-0.4167, -0.8334}, p6={0.8333, -2.0834}, p7={1.9791, -2.0834}, p8={1.9791, -1.6667}, p9={0.4166, 0.}, p10={1.9791, 1.6666}, p11={1.9791, 2.0833}, p12={0.8333, 2.0833}, p13={-0.4167, 0.8333};
line l[13];
l[0]=(line){p1, p2}; l[1]=(line){p2, p3}; l[2]=(line){p3, p4}; l[3]=(line){p4, p5}; l[4]=(line){p5, p6}; l[5]=(line){p6, p7}; l[6]=(line){p7, p8}; l[7]=(line){p8, p9}; l[8]=(line){p9, p10}; l[9]=(line){p10, p11}; l[10]=(line){p11, p12}; l[11]=(line){p12, p13}; l[12]=(line){p13, p1};
int length=13;
	
//	//rectangle-disk
//	point p1={-2., -2.}, p2={-2., -1.}, p3={-2., 0.}, p4={-2., 1.}, p5={-2., 2.}, p6={-1., 2.}, p7={0., 2.}, p8={1., 2.}, p9={2., 2.}, p10={2., 1.}, p11={2., 0.}, p12={2., -1.}, p13={2., -2.}, p14={1., -2.}, p15={0., -2.}, p16={-1., -2.}, p17={1.1999, 0.}, p18={0.8485, 0.8485}, p19={0., 1.1999}, p20={-0.8486, 0.8485}, p21={-1.2, 0.}, p22={-0.8486, -0.8486}, p23={-0.0001, -1.2}, p24={0.8485, -0.8486};
//	line l[24];
//	l[0]=(line){p1, p16}; l[1]=(line){p16, p15}; l[2]=(line){p15, p14}; l[3]=(line){p14, p13}; l[4]=(line){p13, p12}; l[5]=(line){p12, p11}; l[6]=(line){p11, p10}; l[7]=(line){p10, p9}; l[8]=(line){p9, p8}; l[9]=(line){p8, p7}; l[10]=(line){p7, p6}; l[11]=(line){p6, p5}; l[12]=(line){p5, p4}; l[13]=(line){p4, p3}; l[14]=(line){p3, p2}; l[15]=(line){p2, p1}; l[16]=(line){p23, p24}; l[17]=(line){p24, p17}; l[18]=(line){p17, p18}; l[19]=(line){p18, p19}; l[20]=(line){p19, p20}; l[21]=(line){p20, p21}; l[22]=(line){p21, p22}; l[23]=(line){p22, p23};
//	int length=24;

//// Disk-disk
//point p1={1.892, 0.9111}, p2={2.0999, 0.}, p3={2.0999, -0.0001}, p4={1.892, -0.9112}, p5={1.3093, -1.6419}, p6={0.4672, -2.0474}, p7={-0.4673, -2.0474}, p8={-1.3094, -1.6419}, p9={-1.8921, -0.9112}, p10={-2.1, 0.}, p11={-1.8921, 0.9111}, p12={-1.3094, 1.6418}, p13={-0.4673, 2.0473}, p14={0.4672, 2.0473}, p15={1.3093, 1.6418}, p16={1., 0.}, p17={0.6234, 0.7818}, p18={-0.2226, 0.9749}, p19={-0.901, 0.4338}, p20={-0.901, -0.4339}, p21={-0.2226, -0.975}, p22={0.6234, -0.7819};
//line l[22];
//l[0]=(line){p1, p15}; l[1]=(line){p15, p14}; l[2]=(line){p14, p13}; l[3]=(line){p13, p12}; l[4]=(line){p12, p11}; l[5]=(line){p11, p10}; l[6]=(line){p10, p9}; l[7]=(line){p9, p8}; l[8]=(line){p8, p7}; l[9]=(line){p7, p6}; l[10]=(line){p6, p5}; l[11]=(line){p5, p4}; l[12]=(line){p4, p3}; l[13]=(line){p3, p2}; l[14]=(line){p2, p1}; l[15]=(line){p21, p22}; l[16]=(line){p22, p16}; l[17]=(line){p16, p17}; l[18]=(line){p17, p18}; l[19]=(line){p18, p19}; l[20]=(line){p19, p20}; l[21]=(line){p20, p21};
//int length=22;

	unsigned short int i=0;
	point forcerep,temp;

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
	
	//prepare to calibrate magnetic sensor
	delay_ms(500);
	
	excite(10,50,3);
	
	flash(500, 4, flash_green);
	
	timesync1(2500,40);
	
	flash(500, 4, flash_green);
	//calibrate magnetic sensor
	motor_start(motor_right);
	magnetic_calibrate(100,50);
	motor_stop();

	//waiting!
	delay_ms(1000);
    excite(10,50,3);
	delay_ms(500);
	timesync1(200,40);

	for (i = 1; i <= 1000; i++)
	{
		//start locating
		flash1(300,2);
		
		//divide VP with average value!
	
		//change VP value
		forcerep=changeval(VP,ang,8,1,5,0.01);
		
			if(i%2==0)
		{
			flash(300,2,flash_blue);
			modifyVP(8,20,20,0.02);
			DAC_SetChannel1Data(DAC_Align_12b_R, 4096/3.3*VP[0]);  //12位右对齐数据格式设置DAC值
		}
	
		//flash light
		if(i%2==0)
		{
			temp=getposition(700);

			flash(300,2,flash_both);
			if(temp.x<-1000) flash(500,4,flash_blue);
			else if(temp.x>1000) flash(500,4,flash_green);
			else{
				TIM_SetCompare1(TIM10,1000);
				delay_ms(10*(1000+temp.x));
				TIM_SetCompare1(TIM10,0);
				delay_ms(10*(1000-temp.x));
			}
			flash(300,2,flash_both);
			if(temp.y<-1000) flash(500,4,flash_blue);
			else if(temp.y>1000) flash(500,4,flash_green);
			else {
				TIM_SetCompare1(TIM11,1000);
				delay_ms(10*(1000+temp.y));
				TIM_SetCompare1(TIM11,0);
				delay_ms(10*(1000-temp.y));
			}
			
			flash(300,2,flash_both);
			if(temp.x<-1000) flash(500,4,flash_blue);
			else if(temp.x>1000) flash(500,4,flash_green);
			else{
				TIM_SetCompare1(TIM10,1000);
				delay_ms(5*(2000-2*sqrt(1000*fabs(temp.x))));
				TIM_SetCompare1(TIM10,0);
				delay_ms(5*2*sqrt(1000*fabs(temp.x)));
			}
			
			flash(300,2,flash_both);
			if(temp.y<-1000) flash(500,4,flash_blue);
			else if(temp.y>1000) flash(500,4,flash_green);
			else{
				TIM_SetCompare1(TIM11,1000);
				delay_ms(5*(2000-2*sqrt(1000*fabs(temp.y))));
				TIM_SetCompare1(TIM11,0);
				delay_ms(5*2*sqrt(1000*fabs(temp.y)));
			}
		}
		
		//motor move
//		if(i>3&&i%2==0)
//		{
//			temp=plus(regionf(l, length, getposition(scale)),forcerep);
//			temp=times(temp,7.5*tanh(norm(temp)/10.0)/norm(temp));
//		  motor_move(temp);
//			//motor_move(forcerep);
//		}
		 timesync1(50,40);
//		
		
  }
	
	//program end indicator
	uniemit(0);
	flash(500,20,flash_green);
	TIM_SetCompare1(TIM10,400);
	
	return 0;
}

//int main()
//{
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
//	delay_init(168);  //初始化延时函数
//	uart_init(115200);//初始化串口波特率为115200
//	
//	//shape definition


////	//disk
////	point p1={1.8, 0.}, p2={1.5588, 0.8999}, p3={0.9, 1.5588}, p4={0., 1.8}, p5={-0.9, 1.5588}, p6={-1.5589, 0.9}, p7={-1.8, 0.}, p8={-1.5589, -0.9}, p9={-0.9001, -1.5589}, p10={-0.0001, -1.8}, p11={0.8999, -1.5589}, p12={1.5588, -0.9001};
////	line l[12];
////	l[0]=(line){p1, p2}; l[1]=(line){p2, p3}; l[2]=(line){p3, p4}; l[3]=(line){p4, p5}; l[4]=(line){p5, p6}; l[5]=(line){p6, p7}; l[6]=(line){p7, p8}; l[7]=(line){p8, p9}; l[8]=(line){p9, p10}; l[9]=(line){p10, p11}; l[10]=(line){p11, p12}; l[11]=(line){p12, p1};
////	int length=12;

//  //non-convex triangle
//	point p1={-3.0709, 1.3206}, p2={-2.8502, 1.9999}, p3={-2.6683, 1.9999}, p4={-1.5928, 1.6505}, p5={0.5337, 1.5167}, p6={2.6266, 1.916}, p7={2.8051, 1.9999}, p8={2.8501, 1.9999}, p9={3.0597, 1.355}, p10={1.7836, -0.0039}, p11={0.7571, -1.871}, p12={0.3828, -3.3287}, p13={-0.3714, -3.3287}, p14={-0.4273, -2.8862}, p15={-1.2117, -0.9051}, p16={-2.4641, 0.8186};
//	line l[16];
//	l[0]=(line){p1, p16}; l[1]=(line){p16, p15}; l[2]=(line){p15, p14}; l[3]=(line){p14, p13}; l[4]=(line){p13, p12}; l[5]=(line){p12, p11}; l[6]=(line){p11, p10}; l[7]=(line){p10, p9}; l[8]=(line){p9, p8}; l[9]=(line){p8, p7}; l[10]=(line){p7, p6}; l[11]=(line){p6, p5}; l[12]=(line){p5, p4}; l[13]=(line){p4, p3}; l[14]=(line){p3, p2}; l[15]=(line){p2, p1};
//	int length=16;
//	
////	//rectangle-disk
////	point p1={-2., -2.}, p2={-2., -1.}, p3={-2., 0.}, p4={-2., 1.}, p5={-2., 2.}, p6={-1., 2.}, p7={0., 2.}, p8={1., 2.}, p9={2., 2.}, p10={2., 1.}, p11={2., 0.}, p12={2., -1.}, p13={2., -2.}, p14={1., -2.}, p15={0., -2.}, p16={-1., -2.}, p17={1.1999, 0.}, p18={0.8485, 0.8485}, p19={0., 1.1999}, p20={-0.8486, 0.8485}, p21={-1.2, 0.}, p22={-0.8486, -0.8486}, p23={-0.0001, -1.2}, p24={0.8485, -0.8486};
////	line l[24];
////	l[0]=(line){p1, p16}; l[1]=(line){p16, p15}; l[2]=(line){p15, p14}; l[3]=(line){p14, p13}; l[4]=(line){p13, p12}; l[5]=(line){p12, p11}; l[6]=(line){p11, p10}; l[7]=(line){p10, p9}; l[8]=(line){p9, p8}; l[9]=(line){p8, p7}; l[10]=(line){p7, p6}; l[11]=(line){p6, p5}; l[12]=(line){p5, p4}; l[13]=(line){p4, p3}; l[14]=(line){p3, p2}; l[15]=(line){p2, p1}; l[16]=(line){p23, p24}; l[17]=(line){p24, p17}; l[18]=(line){p17, p18}; l[19]=(line){p18, p19}; l[20]=(line){p19, p20}; l[21]=(line){p20, p21}; l[22]=(line){p21, p22}; l[23]=(line){p22, p23};
////	int length=24;

////// Disk-disk
////point p1={1.892, 0.9111}, p2={2.0999, 0.}, p3={2.0999, -0.0001}, p4={1.892, -0.9112}, p5={1.3093, -1.6419}, p6={0.4672, -2.0474}, p7={-0.4673, -2.0474}, p8={-1.3094, -1.6419}, p9={-1.8921, -0.9112}, p10={-2.1, 0.}, p11={-1.8921, 0.9111}, p12={-1.3094, 1.6418}, p13={-0.4673, 2.0473}, p14={0.4672, 2.0473}, p15={1.3093, 1.6418}, p16={1., 0.}, p17={0.6234, 0.7818}, p18={-0.2226, 0.9749}, p19={-0.901, 0.4338}, p20={-0.901, -0.4339}, p21={-0.2226, -0.975}, p22={0.6234, -0.7819};
////line l[22];
////l[0]=(line){p1, p15}; l[1]=(line){p15, p14}; l[2]=(line){p14, p13}; l[3]=(line){p13, p12}; l[4]=(line){p12, p11}; l[5]=(line){p11, p10}; l[6]=(line){p10, p9}; l[7]=(line){p9, p8}; l[8]=(line){p8, p7}; l[9]=(line){p7, p6}; l[10]=(line){p6, p5}; l[11]=(line){p5, p4}; l[12]=(line){p4, p3}; l[13]=(line){p3, p2}; l[14]=(line){p2, p1}; l[15]=(line){p21, p22}; l[16]=(line){p22, p16}; l[17]=(line){p16, p17}; l[18]=(line){p17, p18}; l[19]=(line){p18, p19}; l[20]=(line){p19, p20}; l[21]=(line){p20, p21};
////int length=22;

//	unsigned short int i=0,j=0,endeval=0;
//	point forcerep,temp;

//	//initializaton
//	Adc_Init();
//	Dac1_Init();
//	IIC_Init();
//	hmc_init();
//	LED_Init();
//  TIM_PWM_Init(2000-1,3-1);	//84M/84=1Mhz?????,????500,??PWM??? 1M/500=2Khz.

//	
//	//confirm power-on
//	delay_ms(500);
//	flash(500,3,flash_both);
//	
//	//prepare to calibrate magnetic sensor
//	delay_ms(500);
//	excite(10,50,3);
//	
//	flash(500, 4, flash_green);
//	delay_ms(500);
//	
//	//calibrate magnetic sensor
//	motor_start(motor_right);
//	magnetic_calibrate(100,50);
//	motor_stop();

//	//waiting!
//	delay_ms(1000);
//  excite(10,50,3);
//	delay_ms(500);
//	timesync1(200,40);

//	for (i = 1; i <= 1000; i++)
//	{
//		//start locating
//		flash1(300,2);
//		
//		//divide VP with average value!
//	
//		//change VP value
//		forcerep=changeval(VP,ang,8,5,10,0.01);
//		
//			if(i%1==0)
//		{
//			flash(300,2,flash_blue);
//			modifyVP(8,10,15,0.03);
//			DAC_SetChannel1Data(DAC_Align_12b_R, 4096/3.3*VP[0]);  //12位右对齐数据格式设置DAC值
//		}
//	
//		//flash light
//		if(i%1==0)
//		{
//			temp=getposition(470);

//			flash(300,2,flash_both);
//			if(temp.x<-1000) flash(500,4,flash_blue);
//			else if(temp.x>1000) flash(500,4,flash_green);
//			else{
//				TIM_SetCompare1(TIM10,1000);
//				delay_ms(1000+temp.x);
//				TIM_SetCompare1(TIM10,0);
//				delay_ms(1000-temp.x);
//			}
//			flash(300,2,flash_both);
//			if(temp.y<-1000) flash(500,4,flash_blue);
//			else if(temp.y>1000) flash(500,4,flash_green);
//			else {
//				TIM_SetCompare1(TIM11,1000);
//				delay_ms(1000+temp.y);
//				TIM_SetCompare1(TIM11,0);
//				delay_ms(1000-temp.y);
//			}
//			
//			flash(300,2,flash_both);
//			if(temp.x<-1000) flash(500,4,flash_blue);
//			else if(temp.x>1000) flash(500,4,flash_green);
//			else{
//				TIM_SetCompare1(TIM10,1000);
//				delay_ms(2000-2*sqrt(1000*fabs(temp.x)));
//				TIM_SetCompare1(TIM10,0);
//				delay_ms(2*sqrt(1000*fabs(temp.x)));
//			}
//			
//			flash(300,2,flash_both);
//			if(temp.y<-1000) flash(500,4,flash_blue);
//			else if(temp.y>1000) flash(500,4,flash_green);
//			else{
//				TIM_SetCompare1(TIM10,1000);
//				delay_ms(2000-2*sqrt(1000*fabs(temp.y)));
//				TIM_SetCompare1(TIM10,0);
//				delay_ms(2*sqrt(1000*fabs(temp.y)));
//			}
//		}
//		
//		//motor move
//		if(i>9)
//		{
//			motor_move(plus(regionf(l, length, getposition(scale)),forcerep));
//			//motor_move(forcerep);
//			timesync1(2500,40);
//		}
//		else timesync1(50,40);
//		
//		//end evaluation
//		endeval=0;
//		for(j=1;j<=20;j++)
//		{
//			endeval+=(!TRIG);
//			delay_ms(5);
//		}
//		if(endeval>=19) break;
//  }
//	
//	//program end indicator
//	uniemit(0);
//	flash(500,20,flash_green);
//	TIM_SetCompare1(TIM10,400);
//	
//	return 0;
//}
