unsigned char ee_read_b (unsigned long addr);
void ee_write_b (unsigned long addr, unsigned char data);
unsigned char ee_poll_ack (void);

void ee_write_sec (unsigned long addr, unsigned char *data, unsigned int len);
void ee_read_sec (unsigned long addr, unsigned char *data, unsigned int len);

void ee_write_512 (unsigned int sector, unsigned char * data);
void ee_read_512 (unsigned int sector, unsigned char * data);


