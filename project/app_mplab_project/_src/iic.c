#include	<p24FJ64GB002.h>
#include 	<i2c.h>

#define	idle_iic			IdleI2C2
#define	start_iic			StartI2C2
#define	master_write_iic	MasterWriteI2C2
#define	master_read_iic		MasterReadI2C2
#define	not_ack_iic			NotAckI2C2
#define	stop_iic			StopI2C2
#define	rstart_iic			RestartI2C2
#define	ack_iic				AckI2C2
#define	PEN_BIT				I2C2CONbits.PEN
#define	SEN_BIT				I2C2CONbits.SEN
#define	RSEN_BIT			I2C2CONbits.RSEN

unsigned char ee_read_b (unsigned long addr)
{
	unsigned char data;
 	idle_iic();
  	start_iic();
	while(SEN_BIT);
	if (addr&0x10000)
		master_write_iic(0xA8);
	else
		master_write_iic(0xA0);
 	idle_iic();
	master_write_iic(addr>>8);
 	idle_iic();
	master_write_iic(addr&0xFF);
 	idle_iic();
	
	rstart_iic();
	while(RSEN_BIT);
 	idle_iic();

	if (addr&0x10000)
		master_write_iic(0xA9);
	else
		master_write_iic(0xA1);
 	idle_iic();
 	
	data = master_read_iic();
	not_ack_iic();
 	idle_iic();
	stop_iic();
	  /* Wait till stop sequence is completed */
	while(PEN_BIT);
	return (data);
}

void ee_write_b (unsigned long addr, unsigned char data)
{
 	idle_iic();
  	start_iic();
	while(SEN_BIT);
	if (addr&0x10000)
		master_write_iic(0xA8);
	else
		master_write_iic(0xA0);
 	idle_iic();
	master_write_iic(addr>>8);
 	idle_iic();
	master_write_iic(addr&0xFF);
 	idle_iic();
	master_write_iic(data);
 	idle_iic();
	stop_iic();
	  /* Wait till stop sequence is completed */
	while(PEN_BIT);
}

unsigned char ee_poll_ack (void)
{
unsigned char state;
	idle_iic();
  	start_iic();
	while(SEN_BIT);
	master_write_iic(0xA0);
 	idle_iic();
 	state = I2C2STATbits.ACKSTAT;
	stop_iic();
	  /* Wait till stop sequence is completed */
	while(PEN_BIT);
	return state;
}

void ee_write_sec (unsigned long addr, unsigned char *data, unsigned int len)
{
unsigned int count;
unsigned char temp;
 	idle_iic();
  	start_iic();
	while(SEN_BIT);
	if (addr&0x10000)
		master_write_iic(0xA8);
	else
		master_write_iic(0xA0);

 	idle_iic();
	master_write_iic(addr>>8);
 	idle_iic();
	master_write_iic(addr&0xFF);
 	idle_iic();
  	for (count = 0; count < len; count++)
 		{
		temp = *data;
		*data++;
		master_write_iic(temp);
	 	idle_iic();
	 	}	
	stop_iic();
	  /* Wait till stop sequence is completed */
	while(PEN_BIT);
}

void ee_read_sec (unsigned long addr, unsigned char *data, unsigned int len)
{
	unsigned int count;
	unsigned char temp;
 	idle_iic();
  	start_iic();
	while(SEN_BIT);
	
	if (addr&0x10000)
		master_write_iic(0xA8);
	else
		master_write_iic(0xA0);

 	idle_iic();
	master_write_iic(addr>>8);
 	idle_iic();
	master_write_iic(addr&0xFF);
 	idle_iic();
	
	rstart_iic();
	while(RSEN_BIT);
 	idle_iic();

	if (addr&0x10000)
		master_write_iic(0xA9);
	else
		master_write_iic(0xA1);

 	idle_iic();
 	for (count = 0; count < (len-1); count++)
 		{
	 	temp = master_read_iic();
		ack_iic();
		idle_iic();
		*data = temp;
		*data++;
		}
	*data++ = master_read_iic();
	not_ack_iic();
 	idle_iic();
	stop_iic();
	  /* Wait till stop sequence is completed */
	while(PEN_BIT);
}


void ee_write_512 (unsigned int sector, unsigned char * data)
{
unsigned long addr;
addr = ((unsigned long)(sector))<<9;
ee_write_sec(addr,data,128);
while (ee_poll_ack()==1);
ee_write_sec(addr+128,data+128,128);
while (ee_poll_ack()==1);
ee_write_sec(addr+256,data+256,128);
while (ee_poll_ack()==1);
ee_write_sec(addr+384,data+384,128);
while (ee_poll_ack()==1);
}



void ee_read_512 (unsigned int sector, unsigned char * data)
{
unsigned long addr;
addr = ((unsigned long)(sector))<<9;
ee_read_sec(addr,data,128);
ee_read_sec(addr+128,data+128,128);
ee_read_sec(addr+256,data+256,128);
ee_read_sec(addr+384,data+384,128);
}



