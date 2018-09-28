#include "HMC5883L.h"
#include "myiic.h"
#include "delay.h"

u8 hmc_read_reg(u8 reg)
{
	u8 data;
	IIC_Start();
	IIC_Send_Byte(0x1a);
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0x1b);
	IIC_Wait_Ack();
	data=IIC_Read_Byte();
	IIC_NAck();
	IIC_Stop();
	return data;
}

/*void hmc_read_XYZ(u16* data)
{
	u8 temp;
	u8 i;
    IIC_Start();
	IIC_Send_Byte(WRITE_ADDRESS);
	IIC_Wait_Ack();
	IIC_Send_Byte(DATAX_M);
	IIC_Wait_Ack();
	IIC_Stop();
	IIC_Start();
	IIC_Send_Byte(READ_ADDRESS);
	IIC_Wait_Ack();
	for(i=0;i<3;i++)
	{
		temp=IIC_Read_Byte();
		IIC_Ack();
		*data++=temp*256+IIC_Read_Byte();
		IIC_Ack();
	}
	IIC_Stop();
}*/

void hmc_read_XYZ(short int *data)
{
	u16 temp;
	temp=hmc_read_reg(DATAX_M);
	*data++=(temp<<8)+hmc_read_reg(DATAX_L);
	temp=hmc_read_reg(DATAY_M);
	*data++=(temp<<8)+hmc_read_reg(DATAY_L);
	temp=hmc_read_reg(DATAZ_M);
	*data++=(temp<<8)+hmc_read_reg(DATAZ_L);
}

void hmc_init(void)
{
	IIC_Start();
	IIC_Send_Byte(0x1a);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x0b);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x01);
	IIC_Wait_Ack();
	IIC_Stop();
	
	IIC_Start();
	IIC_Send_Byte(0x1a);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x09);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x1d);
	IIC_Wait_Ack();
	IIC_Stop();
	
	
	delay_ms(10);
}

void HMC_READ0(short int *data)
{ 
	u8 i,temp;
	IIC_Start();
	IIC_Send_Byte(0x1a);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x00);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0x1b);
	IIC_Wait_Ack();
	for(i=0;i<3;i++)
	{
	temp=IIC_Read_Byte();
	IIC_Ack();
	*data++=(IIC_Read_Byte()<<8)+temp;
	IIC_Ack();
	}
	IIC_Read_Byte();
	IIC_Ack();
	IIC_Read_Byte();
	IIC_NAck();
	IIC_Stop();
}

