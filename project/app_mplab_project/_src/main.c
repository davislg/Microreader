/** I N C L U D E S **********************************************************/

#include "USB/USB.h"
#include "USB/usb_function_msd.h"
#include "HardwareProfile.h"
#include "MDD File System/internal flash.h"
#include "iic.h"
#include "FSIO.h"
#include "disp.h"
#include <stdio.h>
#include <string.h>
/*

        _CONFIG1(WDTPS_PS1 & FWPSA_PR32 & WINDIS_OFF & FWDTEN_OFF & ICS_PGx3 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_HS & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV3 & IESO_OFF)
        _CONFIG3(WPFP_WPFP0 & SOSCSEL_IO & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM)
        _CONFIG4(DSWDTPS_DSWDTPS3 & DSWDTOSC_LPRC & RTCOSC_SOSC & DSBOREN_OFF & DSWDTEN_OFF)

*/
_CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & ICS_PGx1 & FWDTEN_OFF & WINDIS_OFF & FWPSA_PR32 & WDTPS_PS8192);
 _CONFIG2(IESO_OFF & FNOSC_FRCPLL & OSCIOFNC_ON & POSCMOD_NONE & PLL96MHZ_ON & PLLDIV_DIV2 & FCKSM_CSECMD & IOL1WAY_OFF);
 _CONFIG3(WPFP_WPFP0 & SOSCSEL_IO & WUTSEL_FST & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM)
 _CONFIG4(DSWDTPS_DSWDTPS3 & DSWDTOSC_LPRC & RTCOSC_LPRC & DSBOREN_OFF & DSWDTEN_OFF)


/** VARIABLES ******************************************************/

typedef union
{
struct
 {
    unsigned up:1;
    unsigned dn:1;
    unsigned ok:1;
    unsigned on:1;
    unsigned b4:1;
    unsigned b5:1;
    unsigned b6:1;
 	unsigned b7:1;
 };
unsigned char CHAR;
}keys_var;


LUN_FUNCTIONS LUN[MAX_LUN + 1] = 
{
    {
        &MDD_IntFlash_MediaInitialize,
        &MDD_IntFlash_ReadCapacity,
        &MDD_IntFlash_ReadSectorSize,
        &MDD_IntFlash_MediaDetect,
        &MDD_IntFlash_SectorRead,
        &MDD_IntFlash_WriteProtectState,
        &MDD_IntFlash_SectorWrite
    }
};

/* Standard Response to INQUIRY command stored in ROM 	*/
const ROM InquiryResponse inq_resp = {
	0x00,		// peripheral device is connected, direct access block device
	0x80,           // removable
	0x04,	 	// version = 00=> does not conform to any standard, 4=> SPC-2
	0x02,		// response is in format specified by SPC-2
	0x20,		// n-4 = 36-4=32= 0x20
	0x00,		// sccs etc.
	0x00,		// bque=1 and cmdque=0,indicates simple queueing 00 is obsolete,
			// but as in case of other device, we are just using 00
	0x00,		// 00 obsolete, 0x80 for basic task queueing
	{'M','i','c','r','o','c','h','p'
    },
	// this is the T10 assigned Vendor ID
	{'M','a','s','s',' ','S','t','o','r','a','g','e',' ',' ',' ',' '
    },
	{'0','0','0','1'
    }
};

unsigned char get_filelist (char * list, unsigned char ssize, unsigned char max_files);
void print_str_file_info (SearchRec searchRecord, unsigned char * dest, unsigned int index);
unsigned char print_filelist (unsigned char start, unsigned char max, unsigned char ssize, unsigned char * list);
void display_select_row (unsigned char *data, unsigned char row, unsigned char width, unsigned char disp_width);
unsigned char get_file_name (unsigned char pointer, char * fn, unsigned long * fsize);
void clear_lcd_buff (unsigned char val);
unsigned int get_adc (unsigned char chnl);
unsigned int averge (unsigned int * arr, unsigned char size, unsigned int val, unsigned char * index);
int comp(const void *e1, const void *e2);
void i2c_init (void);
void i2c_dly (void);
void shdn(void);

void USBDeviceTasks(void);
void ProcessIO(void);
void UserInit(void);
void USBCBSendResume(void);

unsigned char state,usb_mode;
unsigned char test,data;
unsigned long addr;
unsigned int count;
//unsigned char buff[50];
char filename[20];

#define	MAX_FILES	128

char filelist[MAX_FILES*DISP_COLS];
unsigned char files_total;

#define ADC_SIZE	25

unsigned char ls_first, ls_max, ls_ret, file_pointer, ls_invert;
unsigned long file_position, file_size;
unsigned int adc_res;
unsigned int adc_res_arr[ADC_SIZE],adc_res_temp[ADC_SIZE];
unsigned char adc_res_arr_p;

char str1[20];
unsigned char lcd_buff[(DISP_COLS*DISP_ROWS)+2];
FSFILE * pointer;
SearchRec           searchRecord;

volatile char i2c_count, main_count, on_count;

keys_var keys, keys_new, keys_old;

#define	key_up	keys.up
#define	key_dn	keys.dn
#define	key_ok	keys.ok
#define	key_on	keys.on



#define	BL	PORTBbits.RB9	
#define	BM	PORTBbits.RB13
#define	BU	PORTBbits.RB8

#define	USB_POW		PORTBbits.RB7

#define	DISP_ON		LATBbits.LATB5

unsigned int ms_count;
float volts;

/*
an1 - charger
an10 - bat
*/

int main(void)
{
//PMD4 = 0xFFFF;
DISP_ON = 0;
state = 255;
AD1PCFG = 0xFFFF;
I2C2BRG = 100;
I2C2CON = 0xD200; 
data = 0x55;
TRISB = 0x0000;
TRISA = 0;
TRISAbits.TRISA1 = 1;
TRISBbits.TRISB5 = 0;
TRISBbits.TRISB7 = 1;
TRISBbits.TRISB14 = 1;

CNEN1bits.CN13IE = 1;
CNEN2bits.CN21IE = 1;
CNEN2bits.CN22IE = 1;
CNPU1bits.CN13PUE = 1;
CNPU2bits.CN21PUE = 1;
CNPU2bits.CN22PUE = 1;
TRISBbits.TRISB8 = 1;
TRISBbits.TRISB9 = 1;
TRISBbits.TRISB13 = 1;


TMR3 = 0;
PR3 = 2000;
IFS0bits.T3IF = 0;
IEC0bits.T3IE = 1;
T3CONbits.TCKPS = 0b10;
T3CONbits.TON = 1;

AD1PCFGbits.PCFG1 = 0;
AD1PCFGbits.PCFG10 = 0;
AD1CON1 = 0x00E2;
AD1CHS = 0x0A;	//channel
AD1CSSL=0;
AD1CON3 = 0x9F02;
AD1CON2 = 0x0000;
AD1CON1bits.ADON = 1;
AD1CON1bits.ASAM=0;

main_count=0;
while (main_count<30);

iic_start();
if (I2C2STATbits.BCL==1)
	i2c_init();
else
	iic_stop();	
init_lcd();
DISP_ON = 1;
clear_lcd_buff(' ');
sprintf ((char *)lcd_buff+ROW0,"Filesystem OK");
refresh_lcd(lcd_buff);	
if ( FSInit() != TRUE )
	while (1);
sprintf ((char *)lcd_buff+ROW1,"Filelist");
refresh_lcd(lcd_buff);	
files_total = get_filelist(filelist, DISP_COLS, MAX_FILES);
ls_first = 0;
ls_max = 3;
file_pointer = 0;
usb_mode = 0;
keys.CHAR=0;

while (1)
	{
	
	if (usb_mode==0)
		{
		adc_res = get_adc(10);
		adc_res = averge(adc_res_arr,ADC_SIZE,adc_res,&adc_res_arr_p);
		
		if (I2C2STATbits.BCL==1)
			{
			i2c_init();
			}
		
		if (state==255)
			{
			CLKDIVbits.RCDIV0=0;
			CLKDIVbits.PLLEN = 0;
			__builtin_write_OSCCONH(0x07);  // Initiate Clock Switch to Primary
			             // Oscillator with PLL (NOSC=0b011)
			 __builtin_write_OSCCONL(0x01);  // Start clock switching
			 while (OSCCONbits.COSC != 0b111); // Wait for Clock switch to occur  
			I2C2BRG = 50;
			state = 0;
			}
			
		if (state==0)
			{
			count++;
			clear_lcd_buff(' ');
			ls_ret = print_filelist(ls_first,ls_max,DISP_COLS,filelist);
			if ((ls_ret>files_total)&(ls_first!=0)) 
				{
				file_pointer--;
				ls_first--;
				}	
			if (file_pointer>=ls_ret)
				ls_first++;
			if (file_pointer<ls_first)
				if (ls_first!=0) ls_first--;
			ls_ret = print_filelist(ls_first,ls_max,DISP_COLS,filelist);
			ls_invert = file_pointer - ls_first;
			display_select_row(lcd_buff,ls_invert,DISP_COLS,DISP_COLS);
			volts = (((float) (adc_res))/1023.0)*2.0*3.3;
/*			if (count&0x01)
				sprintf ((char *)lcd_buff+ROW3,"%F: %d/%d B:%2.2fV ",file_pointer+1,files_total,volts);
			else
				sprintf ((char *)lcd_buff+ROW3,"%F: %d/%d B:%2.2fV. ",file_pointer+1,files_total,volts);
*/
			sprintf ((char *)lcd_buff+ROW3,"%F: %d/%d B:%2.2fV ",file_pointer+1,files_total,volts);
			refresh_lcd(lcd_buff);
			main_count=0;
			state = 101;
			}
			
		if (state==101)
			{
			if (main_count>=10) state =0;
			if (key_up!=0)
				{
				key_up = 0;
				file_pointer++;
				if (file_pointer>=files_total)
					file_pointer--;
				state = 0;
				}
			if (key_dn!=0)
				{
				key_dn = 0;
				if (file_pointer>0) file_pointer--;
				state = 0;
				}
			if (key_ok!=0)
				{
				key_ok = 0;
				get_file_name(file_pointer,filename,&file_size);
				state = 1;
				file_position = 0;
				clear_lcd_buff(' ');
		   		pointer = FSfopen (filename, "r");
		   		if (pointer == NULL)
		      		state = 3;
				}
			if (key_on!=0)
				{
				shdn();	
				state = 255;
				}				
			}



		if (state==1)
			{
			clear_lcd_buff(' ');
			if( FSfseek( pointer, file_position, SEEK_SET ) != 0 )
				state = 3;
	      	if (FSfread (lcd_buff,DISP_COLS*DISP_ROWS, 1, pointer) != 1)
	      		state = 3;
	      	refresh_lcd(lcd_buff);		
			state = 2;
			
			}	
		if (state==2)
			{
			if (key_up!=0)
				{
				key_up = 0;
				if (file_position<(file_size-DISP_COLS))
					file_position = file_position + DISP_COLS;
				state = 1;
				}
			if (key_dn!=0)
				{
				key_dn = 0;
				if (file_position>=DISP_COLS)
					file_position = file_position - DISP_COLS;
				state = 1;
				}
			if (key_ok!=0)
				{
				key_ok = 0;
				state = 0;
			   	clear_lcd_buff(' ');
//			   	refresh_lcd(lcd_buff);
			   	if (FSfclose (pointer))
			      	state = 3;
			    
				}		
			}		
		if (state==3)
			{
			file_position++;
			}	
		if (USB_POW==1)
			{
			usb_mode = 1;
			state = 255;
			}
		}	
	if (usb_mode==1)
		{
		adc_res = get_adc(1);
		if (state==255)
			{
			I2C2BRG = 20;
			__builtin_write_OSCCONH(0x01);  // Initiate Clock Switch to Primary
			             // Oscillator with PLL (NOSC=0b011)
			 __builtin_write_OSCCONL(0x01);  // Start clock switching
			 while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur  
			CLKDIVbits.RCDIV0=0;
			CLKDIVbits.PLLEN       = 1;
			while(OSCCONbits.LOCK!=1) {};
			USBDeviceInit();
			state = 1;
			}
		if (state==1)
			{
			USBDeviceTasks(); 
			ProcessIO();        
			if (main_count>=10) state =2;
			}
		if (state==2)
			{
			main_count = 0;
			clear_lcd_buff(' ');
			sprintf ((char *)lcd_buff+ROW1," -- USB MODE --");
			if (adc_res<400)
				sprintf ((char *)lcd_buff+ROW2," charging...");
			else
				sprintf ((char *)lcd_buff+ROW2," charged   ");
//			sprintf ((char *)lcd_buff+ROW3," %d",adc_res);
			refresh_lcd(lcd_buff);	
			state = 1;	
			}

		if (USB_POW==0)
			{
			usb_mode = 0;
			state = 255;
			}

		}
	}

}//end main


void shdn(void)
{
while ((BL==0)|(BU==0)|(BM==0));
main_count=0;
while (main_count<30);
DISP_ON = 0;
shdn_disp();
DISP_ON = 0;
DISP_ON = 0;
TRISBbits.TRISB2 = 1;
TRISBbits.TRISB3 = 1;
PMD1 = 0xFFFF;
PMD2 = 0xFFFF;
PMD3 = 0xFFFF;
PMD4 = 0xFFFF;
IEC0 = 0;
IEC1 = 0;
IEC2 = 0;
IEC3 = 0;
IEC4 = 0;
IEC5 = 0;
IFS0 = 0;
IFS1 = 0;
IFS2 = 0;
IFS3 = 0;
IFS4 = 0;
IFS5 = 0;
CLKDIVbits.RCDIV = 0b111;
CNEN1bits.CN13IE = 0;
CNEN2bits.CN21IE = 1;
CNEN2bits.CN22IE = 0;
IFS1bits.CNIF = 0;
IEC1bits.CNIE = 1;
__asm__("PWRSAV #0");    //Sleep Mode
__asm__("NOP");
__asm__("NOP");
__asm__("NOP");
__asm__("RESET");    //Reset
}

void i2c_init (void)
{
unsigned char i;

I2C2CON = 0x0000;
LATBbits.LATB3 = 1;
TRISBbits.TRISB3 = 0;
TRISBbits.TRISB2 = 1;
i=0;
while ((i<9)&(PORTBbits.RB2==0))
	{
	LATBbits.LATB3 = 0;
	i2c_dly();
	LATBbits.LATB3 = 1;
	i2c_dly();
	}

TRISBbits.TRISB3 = 1;
I2C2CON = 0xD200; 
iic_start();
iic_stop();

}

void i2c_dly (void)
{
i2c_count=0;
while (i2c_count<2);
}

unsigned int averge (unsigned int * arr, unsigned char size, unsigned int val, unsigned char * index)
{
unsigned char ind,i;
unsigned int res;
ind = * index;
arr[ind++] = val;
if (ind == size)
	ind = 0;
for (i=0;i<size;i++) 
	adc_res_temp[i] = arr[i];
qsort(adc_res_temp, size, sizeof(unsigned int), comp);
res = adc_res_temp[size/2];
*index = ind;
return res;
}

int comp(const void *e1, const void *e2) 
	{ 
	unsigned int * a1 = e1; 
	unsigned int * a2 = e2; 
	if (*a1 < *a2) 
	return -1; 
	else if (*a1 == *a2) 
	return 0; 
	else 
	return 1; 
	} 

unsigned int get_adc (unsigned char chnl)
{
unsigned int temp;
AD1CHS = chnl;
AD1CON1bits.SAMP = 1;
while (AD1CON1bits.DONE == 0);
temp = ADC1BUF0;
return temp;
}

void __attribute__((interrupt, auto_psv)) _T3Interrupt(void)
{
IFS0bits.T3IF = 0;
keys_new.CHAR = 0;
i2c_count++;
main_count++;
if (BU==0) keys_new.up = 1;
if (BM==0) keys_new.ok = 1;
if (BL==0) keys_new.dn = 1;
if ((BL==0)&(BU==0)&(keys_new.on==0)) on_count++;
	else on_count==0;
if (on_count>20)
	keys_new.on = 1;

keys.CHAR = keys.CHAR | ((keys_old.CHAR ^ keys_new.CHAR) & keys_new.CHAR);
keys_old.CHAR = keys_new.CHAR;
}

void clear_lcd_buff (unsigned char val)
{
unsigned int i;
for (i=0;i<DISP_COLS*DISP_ROWS;i++) lcd_buff[i] = val;
}

unsigned char get_file_name (unsigned char pointer, char * fn, unsigned long * fsize)
{
unsigned char count;
count = 0;
if (!FindFirst( "*.*", ATTR_DIRECTORY | ATTR_ARCHIVE | ATTR_READ_ONLY | ATTR_HIDDEN, &searchRecord ))
	{
	if (count==pointer)
		{
		strcpy(fn,searchRecord.filename);
		*fsize = searchRecord.filesize;
		return 1;
		}
	count++;
	while ((!FindNext( &searchRecord ))&(count<(ls_first+ls_max)))
		{
		if (count==pointer)
			{
			strcpy(fn,searchRecord.filename);
			*fsize = searchRecord.filesize;
			return 1;
			}
		count++;
		}
	}
return 0;
}

void display_select_row (unsigned char *data, unsigned char row, unsigned char width, unsigned char disp_width)
{
unsigned int i;
//for (i=row*disp_width;i<(row*disp_width) + width;i++) if (data[i]!=0) data[i] = data[i]|0x80;
for (i=row*disp_width;i<(row*disp_width) + width;i++) data[i] = data[i]|0x80;
}

unsigned char get_filelist (char * list, unsigned char ssize, unsigned char max_files)
{
unsigned int count;
count = 0;
if (!FindFirst( "*.*", ATTR_DIRECTORY | ATTR_ARCHIVE | ATTR_READ_ONLY | ATTR_HIDDEN, &searchRecord ))
	{
	print_str_file_info(searchRecord, list, count * ssize);
	count++;
	while ((!FindNext( &searchRecord ))&(count<max_files))
		{
		print_str_file_info(searchRecord, list, count * ssize);
		count++;
		}
	}
return count;
}


unsigned char print_filelist (unsigned char start, unsigned char max, unsigned char ssize, unsigned char * list)
{
unsigned int i;
for (i=0;i<max;i++)
	strncpy(lcd_buff+(i*ssize),list+((start + i)*ssize), ssize);
return (start + max);
}

void print_str_file_info (SearchRec searchRecord, unsigned char * dest, unsigned int index)
{
if (searchRecord.attributes&ATTR_DIRECTORY)
	sprintf(str1, "%11s -DIR-", searchRecord.filename);
else
	{
	if (searchRecord.filesize<1000)
		sprintf(str1, "%11s %ldB", searchRecord.filename, searchRecord.filesize );
	else if (searchRecord.filesize<1000000)
		sprintf(str1, "%11s %ldK", searchRecord.filename, searchRecord.filesize/1024 );
	}
strcpy(dest+index, str1);	
}

void ProcessIO(void)
{   
    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

    MSDTasks();    
}//end ProcessIO

void USBCBSuspend(void)
{
}
void USBCBWakeFromSuspend(void)
{
}
void USBCB_SOF_Handler(void)
{
}
void USBCBErrorHandler(void)
{
}
void USBCBCheckOtherReq(void)
{
    USBCheckMSDRequest();
}//end
void USBCBStdSetDscHandler(void)
{
}//end
void USBCBInitEP(void)
{
    #if (MSD_DATA_IN_EP == MSD_DATA_OUT_EP)
        USBEnableEndpoint(MSD_DATA_IN_EP,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    #else
        USBEnableEndpoint(MSD_DATA_IN_EP,USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
        USBEnableEndpoint(MSD_DATA_OUT_EP,USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    #endif

    USBMSDInit();
}

void USBCBSendResume(void)
{
    static WORD delay_count;
    if(USBGetRemoteWakeupStatus() == TRUE) 
    {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();
            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE; 
            delay_count = 3600U;        
            do
            {
                delay_count--;
            }while(delay_count);
            
            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            }while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling
            USBUnmaskInterrupts();
        }
    }
}

BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch( (INT)event )
    {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            if(MSDWasLastCBWValid() == FALSE)
            {
                //Need to re-stall the endpoints, for persistent STALL behavior.
    			USBStallEndpoint(MSD_DATA_IN_EP, IN_TO_HOST);
      			USBStallEndpoint(MSD_DATA_OUT_EP, OUT_FROM_HOST);                 
            }
            else
            {   
                if((USB_HANDLE)pdata == USBGetNextHandle(MSD_DATA_OUT_EP, OUT_FROM_HOST))
                {
                    USBMSDOutHandle = USBRxOnePacket(MSD_DATA_OUT_EP, (BYTE*)&msd_cbw, MSD_OUT_EP_SIZE);
                }    
            }    
            break;
        default:
            break;
    }      
    return TRUE; 
}

           
/** EOF main.c ***************************************************************/

