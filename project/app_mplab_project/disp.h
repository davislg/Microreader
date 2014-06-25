#define	DISP_COLS	16
#define	DISP_ROWS	4

#define	ROW0	0
#define	ROW1	16
#define	ROW2	32
#define	ROW3	48

void refresh_lcd (unsigned char * buff);
void init_lcd (void);
void iic_start (void);
void iic_stop (void);
void iic_write (unsigned char data);
void lcd_cmd(unsigned char data);
void disp_set_xy (unsigned char x, unsigned char y);
void clrscr (unsigned char data);
void char_print (unsigned char data);
void putslcd(char *data);
void putslcd2(char *data);
void char_print2 (unsigned char data);
void shdn_disp (void);

