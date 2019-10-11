#include "./nokialcd.h"
#include "./brucon.h"
#include "map.h"

#include <freertos/task.h>

TickType_t  last_click;

spi_device_handle_t spi;
uint8_t driver;
static char x_offset = 0;
static char y_offset = 0;
static char lastfill = 0;

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

uint16_t *frame = NULL;// [ROW_LENGTH*COL_HEIGHT];
uint8_t *framespi = NULL;

void do_spi_init();

void switchbacklight(int state){
    if(state)
        last_click=xTaskGetTickCount();
    if(esp_reset_reason() == ESP_RST_BROWNOUT){
      gpio_set_level(ENSCR_PIN,0);
      printf("has a brownout, not turning backlight on")	;
      return;
    }
    gpio_set_level(ENSCR_PIN,state);
}

portMUX_TYPE mmux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE * mux = &mmux;

void framespi_write9(int offset, uint16_t value)
{
  int index = offset/8; // offset in framespi
  int shift = offset%8; // bitshift to apply
  uint8_t mask0 = 0xff<<(8-shift);
  uint8_t mask1 = 0xff>>(7-shift);

  framespi[index] = (framespi[index]&mask0)|(value>>1)>>shift;
  framespi[index+1] = (value&mask1) << (7-shift);
}


void send_9(uint16_t data){
    esp_err_t ret;
    spi_transaction_t t;
    uint16_t workaround = SPI_SWAP_DATA_TX(data,9);
    memset(&t, 0, sizeof(t));        //Zero out the transaction
    t.length=9;                      //Command is 9 bits
    t.tx_buffer=&workaround;               //The data is the cmd itself
    //  portENTER_CRITICAL(mux);

//    __disable_interrupt();
// taskENTER_CRITICAL();

    ret=
        spi_device_transmit(spi, &t);//Transmit!
    assert(ret==ESP_OK);
//    portEXIT_CRITICAL(mux);
//    __enable_interrupt();
//    taskEXIT_CRITICAL();
}


void lcd_send(char t,uint8_t d)
{
    uint16_t c = ((t<<8) | d);
    send_9(c);
}

void lcd_send_framespi(int nbits)
{
  esp_err_t ret;
  spi_transaction_t t;

  memset(&t, 0, sizeof(t));        //Zero out the transaction
  t.length=(nbits/64)*64;                      //Command is 9 bits
  t.tx_buffer=framespi;               //The data is the cmd itself
  ret = spi_device_transmit(spi, &t);//Transmit!
  assert(ret==ESP_OK);
}

/**
 * send_frame()
 *
 * Sends a 132*132 pixels frame to the screen, the high-speed way.
 * Takes a framebuffer (132*132 uint16_t values) as 12-bit pixels.
 *
 * This function refreshes the whole screen.
 **/

void send_frame(uint16_t *framebuffer)
{
  int k,i;

  /* Init column and raw */
  lcd_send(LCD_COMMAND,PASETP);
  lcd_send(LCD_DATA,0);
  lcd_send(LCD_DATA,131);

  lcd_send(LCD_COMMAND,CASETP);
  lcd_send(LCD_DATA,0);
  lcd_send(LCD_DATA,131);

  lcd_send(LCD_COMMAND,RAMWRP);

  /* Send line by line 8 pixels -> 9 bytes*/
  memset(framespi, 0x0, 792*sizeof(uint8_t));

  i=0;
  k=0;
  while (i<ROW_LENGTH*COL_HEIGHT)
  {
    framespi_write9(k, ((framebuffer[i]>>4)&0xff) | (LCD_DATA << 8));
    k+=9;
    framespi_write9(k, (framebuffer[i]&0x0F)<<4 | (framebuffer[i+1]>>8) | (LCD_DATA << 8));
    k+=9;
    framespi_write9(k, (framebuffer[i+1]&0xff) | (LCD_DATA << 8));
    k+=9;

    i+=2;
    if (i%128==0)
    {
      lcd_send_framespi(k);
      k=0;
    }
  }
}

/**
 * Loads a 132*132 bitmap into the framebuffer, and updates the screen.
 **/

void load_bitmap(uint16_t *bitmap)
{
  bitblt(0,0,132,132,bitmap);
}

/**
 * bitblt()
 *
 * Bitblt a region into our framebuffer.
 * This routine DOES NOT refresh screen !
 *
 **/
void bitblt(int x, int y, int width, int height, uint16_t *region)
{
  int w,z;
  for (w=0;w<height;w++)
    for (z=0;z<width;z++)
      frame[(y+w)*ROW_LENGTH + (132 - x+z)] = region[w*ROW_LENGTH + z];
}

void lcd_cmd(uint8_t cmd)
{
    uint16_t c = (0x000 | cmd);
    send_9(c);
}

void lcd_data(uint8_t data)
{
    uint16_t c = (0x100 | data);
    send_9(c);
}


void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    gpio_set_level(LCD_CS, 0);
}
void lcd_spi_post_transfer_callback(spi_transaction_t *t)
{
    gpio_set_level(LCD_CS, 1);
}




void init_lcd(int type)
{
    esp_err_t ret;
    gpio_config_t io_conf;

    /* Allocate framebuffer */
    frame = (uint16_t*) malloc(ROW_LENGTH*COL_HEIGHT*sizeof(uint16_t));
    /* Allocate frame SPI transactions buffer. */
    framespi = (uint8_t*) heap_caps_malloc(792*sizeof(uint8_t), MALLOC_CAP_DMA);

    vPortCPUInitializeMutex(mux);
    driver = type;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =  (1ULL<<LCD_SCK) |  (1ULL<<LCD_DIO) | (1ULL<<LCD_RST) |  (1ULL<<LCD_CS)|  (1ULL<<TRIGLA);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    TRIG;

    LCD_SCK_L;
    LCD_DIO_L;
    dl_us(10);
    LCD_CS_H;
    dl_us(10);
    LCD_RST_L;
    dl_us(200);
    LCD_SCK_H;
    LCD_DIO_H;
    LCD_RST_H;


    spi_bus_config_t buscfg={
        .mosi_io_num=LCD_DIO,
        .sclk_io_num=LCD_SCK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=30000000,               //Clock out at 1 MHz
        .mode=3,                                //SPI mode 3

        .queue_size=50,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=NULL,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .spics_io_num=LCD_CS,               //CS pin
        /* .pre_cb=lcd_spi_pre_transfer_callback, //Specify pre-transfer callback to handle D/C line
           .post_cb=lcd_spi_post_transfer_callback //Specify pre-transfer callback to handle D/C line*/
    };
    gpio_config_t io_conf2={
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<LCD_SCK) |  (1ULL<<LCD_DIO) |  (1ULL<<LCD_RST) |  (1ULL<<LCD_CS)|  (1ULL<<TRIGLA),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf2);

    driver = type;



    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
    do_spi_init();


    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =  (1ULL<<ENSCR_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 0;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);


}

void do_spi_init()
{
    TRIG;
    LCD_CS_H;
    dl_us(10);
    LCD_RST_L;
    dl_us(200);
    LCD_RST_H;

    lcd_send(LCD_COMMAND,SLEEPOUT);	// Sleep Out (0x11)
    lcd_send(LCD_COMMAND,BSTRON);   	// Booster voltage on (0x03)
    lcd_send(LCD_COMMAND,DISPON);		// Display on (0x29)
    // lcd_send(LCD_COMMAND,INVON);		// Inversion on (0x20)
    // 12-bit color pixel format:
    lcd_send(LCD_COMMAND,PCOLMOD);		// Color interface format (0x3A)
    lcd_send(LCD_DATA,0x03);			// 0b011 is 12-bit/pixel mode
    lcd_send(LCD_COMMAND,MADCTL);		// Memory Access Control(PHILLIPS)

    lcd_send(LCD_DATA,0x00);
    lcd_send(LCD_COMMAND,SETCON);		// Set Contrast(PHILLIPS)
    lcd_send(LCD_DATA,0x3f);

    lcd_send(LCD_COMMAND,NOPP); // nop(PHILLIPS)
}


void fillframe12B(uint16_t color_12b)
{
    uint32_t i =0;
    while(i<(ROW_LENGTH*COL_HEIGHT))
            frame[i++]=color_12b;
}

int lcd_commit(void)
{
  if (frame != NULL)
  {
    send_frame(frame);
    return 1;
  }
  else
    return 0;
}

void go_framep(uint16_t *p)
{

  lcd_send(LCD_COMMAND,PASETP);
  lcd_send(LCD_DATA,0);
  lcd_send(LCD_DATA,131);

  lcd_send(LCD_COMMAND,CASETP);
  lcd_send(LCD_DATA,0);
  lcd_send(LCD_DATA,131);

  lcd_send(LCD_COMMAND,RAMWRP);

	for(unsigned int i=0; i < (ROW_LENGTH*COL_HEIGHT); i+=2)
	{
		lcd_send(LCD_DATA,(  (*(p+i))  >>4)&0xFF);
    lcd_send(LCD_DATA,( ((*(p+i))  &0x0F)<<4)|( (*(p+i+1))>>8));
		lcd_send(LCD_DATA,( (*(p+i+1)) &0xFF));
	}

	x_offset = 0;
	y_offset = 0;
}

void lcd_clearB12(int color){
    lastfill=color;

		lcd_send(LCD_COMMAND,PASETP);
		lcd_send(LCD_DATA,0);
		lcd_send(LCD_DATA,131);

		lcd_send(LCD_COMMAND,CASETP);
		lcd_send(LCD_DATA,0);
		lcd_send(LCD_DATA,131);

    lcd_send(LCD_COMMAND,RAMWRP);
};

void lcd_contrast(char setting){
		setting &= 0x7F;	// msb is not used, mask it out
    lcd_send(LCD_COMMAND,SETCON);	// contrast command (PHILLIPS)
    lcd_send(LCD_DATA,setting);	// volume (contrast) setting - course adjustment,  -- original was 24
};

void lcd_setChar(char c, int x, int y, int fColor, int bColor, char transp)
{
	y	=	(COL_HEIGHT - 1) - y; // make display "right" side up
	x	=	(ROW_LENGTH - 2) - x;

	int             i,j;
	unsigned int    nCols;
	unsigned int    nRows;
	unsigned int    nBytes;
	unsigned char   PixelRow;
	unsigned char   Mask;
	unsigned int    Word0;
	unsigned int    Word1;
	const unsigned char   *pFont;
	const unsigned char   *pChar;


	// get pointer to the beginning of the selected font table
	pFont = (const unsigned char *)FONT8x16;
	// get the nColumns, nRows and nBytes
	nCols = *(pFont);
	nRows = *(pFont + 1);
	nBytes = *(pFont + 2);
	// get pointer to the last byte of the desired character
	pChar = pFont + (nBytes * (c - 0x1F)) + nBytes - 1;

	if (driver)	// if it's an epson
	{
		// Row address set (command 0x2B)
		lcd_send(LCD_COMMAND,PASET);
		lcd_send(LCD_DATA,x);
		lcd_send(LCD_DATA,x + nRows - 1);
		// Column address set (command 0x2A)
		lcd_send(LCD_COMMAND,CASET);
		lcd_send(LCD_DATA,y);
		lcd_send(LCD_DATA,y + nCols - 1);

		// WRITE MEMORY
		lcd_send(LCD_COMMAND,RAMWR);
		// loop on each row, working backwards from the bottom to the top
		for (i = nRows - 1; i >= 0; i--) {
			// copy pixel row from font table and then decrement row
			PixelRow = *(pChar++);
			// loop on each pixel in the row (left to right)
			// Note: we do two pixels each loop
			Mask = 0x80;
			for (j = 0; j < nCols; j += 2)
			{
				// if pixel bit set, use foreground color; else use the background color
				// now get the pixel color for two successive pixels
				if ((PixelRow & Mask) == 0)
					Word0 = frame[i*ROW_LENGTH+j];
				else
					Word0 = fColor;
				Mask = Mask >> 1;
				if ((PixelRow & Mask) == 0)
					Word1 = bColor;
				else
					Word1 = fColor;
				Mask = Mask >> 1;
				// use this information to output three data bytes
				lcd_send(LCD_DATA,(Word0 >> 4) & 0xFF);
				lcd_send(LCD_DATA,((Word0 & 0xF) << 4) | ((Word1 >> 8) & 0xF));
				lcd_send(LCD_DATA,Word1 & 0xFF);
			}
		}
	}
	else
	{
        // fColor = swapColors(fColor);
        // bColor = swapColors(bColor);

		// Row address set (command 0x2B)
		lcd_send(LCD_COMMAND,PASETP);
		lcd_send(LCD_DATA,x);
		lcd_send(LCD_DATA,x + nRows - 1);
		// Column address set (command 0x2A)
		lcd_send(LCD_COMMAND,CASETP);
		lcd_send(LCD_DATA,y);
		lcd_send(LCD_DATA,y + nCols - 1);

		// WRITE MEMORY
		lcd_send(LCD_COMMAND,RAMWRP);
		// loop on each row, working backwards from the bottom to the top
		pChar+=nBytes-1;  // stick pChar at the end of the row - gonna reverse print on phillips
		for (i = nRows - 1; i >= 0; i--) {
			// copy pixel row from font table and then decrement row
			PixelRow = *(pChar--);
			// loop on each pixel in the row (left to right)
			// Note: we do two pixels each loop
			Mask = 0x01;  // <- opposite of epson
			for (j = 0; j < nCols; j += 2)
			{
				// if pixel bit set, use foreground color; else use the background color
				// now get the pixel color for two successive pixels
				if ((PixelRow & Mask) == 0)
					Word0 = bColor;//frame[i*ROW_LENGTH+j];
				else
					Word0 = fColor;
				Mask = Mask << 1; // <- opposite of epson
				if ((PixelRow & Mask) == 0)
					Word1 = bColor;//frame[i*ROW_LENGTH+j];
				else
					Word1 = fColor;
				Mask = Mask << 1; // <- opposite of epson
				// use this information to output three data bytes
				lcd_send(LCD_DATA,(Word0 >> 4) & 0xFF);
				lcd_send(LCD_DATA,((Word0 & 0xF) << 4) | ((Word1 >> 8) & 0xF));
				lcd_send(LCD_DATA,Word1 & 0xFF);
			}
		}
	}
}


void lcd_setStr(char *pString, int x, int y, int fColor, int bColor, char uselastfill, char newline)
{
	x = x + 12;
	y = y + 7;
    int originalY = y;

	// loop until null-terminator is seen
	while (*pString != 0x00) {
		// draw the character
		lcd_setChar(*pString++, x, y, fColor, bColor,uselastfill);
		// advance the y position
		y = y + 8;
		// bail out if y exceeds 131

		if ((y > 131) ) {
            if(newline){

            x = x + 16;
            y = originalY;
            } else {
                break;

            }

        }
        if (x > 131) break;
	}
}

void setPixel(int color, unsigned char x, unsigned char y)
{
  y = (COL_HEIGHT - 1) - y;
  x = (ROW_LENGTH - 1) - x;

  lcd_send(LCD_COMMAND,PASETP); // page start/end ram
  lcd_send(LCD_DATA,x);
  lcd_send(LCD_DATA,x);

  lcd_send(LCD_COMMAND,CASETP); // column start/end ram
  lcd_send(LCD_DATA,y);
  lcd_send(LCD_DATA,y);

  lcd_send(LCD_COMMAND,RAMWRP); // write

  lcd_send(LCD_DATA,(unsigned char)((color>>4)&0x00FF));
  lcd_send(LCD_DATA,(unsigned char)(((color&0x0F)<<4)|0x00));
}

void lcd_setLine(int x0, int y0, int x1, int y1, int color)
{
        int dy = y1 - y0; // Difference between y0 and y1
        int dx = x1 - x0; // Difference between x0 and x1
        int stepx, stepy;

        if (dy < 0)
        {
                dy = -dy;
                stepy = -1;
        }
        else
                stepy = 1;

        if (dx < 0)
        {
                dx = -dx;
                stepx = -1;
        }
        else
                stepx = 1;

        dy <<= 1; // dy is now 2*dy
        dx <<= 1; // dx is now 2*dx
        setPixel(color, x0, y0);

        if (dx > dy)
        {
                int fraction = dy - (dx >> 1);
                while (x0 != x1)
                {
                        if (fraction >= 0)
                        {
                                y0 += stepy;
                                fraction -= dx;
                        }
                        x0 += stepx;
                        fraction += dy;
                        setPixel(color, x0, y0);
                }
        }
        else
        {
                int fraction = dx - (dy >> 1);
                while (y0 != y1)
                {
                        if (fraction >= 0)
                        {
                                x0 += stepx;
                                fraction -= dy;
                        }
                        y0 += stepy;
                        fraction += dx;
                        setPixel(color, x0, y0);
                }
        }
}

void lcd_setRect(int x0, int y0, int x1, int y1, unsigned char fill, int color)
{
    // check if the rectangle is to be filled
    unsigned int xstart,xend,ystart,yend,width=0,height=0;
    int j=0,i=0;
    // int tx0,tx1,ty0,ty1;

//    uint8_t cb0,cb1,cb2;

    if (fill == 1)
    {

        y0 = (COL_HEIGHT - 1) - y0;
        x0 = (ROW_LENGTH - 1) - x0;
        y1 = (COL_HEIGHT - 1) - y1;
        x1 = (ROW_LENGTH - 1) - x1;

        if(x0>x1){
            xstart=x1;
            xend=x0;
        } else {
            xstart=x0;
            xend=x1;
        }
        if(y0>y1){
            ystart=y1;
            yend=y0;
        } else {
            ystart=y0;
            yend=y1;
        }

        width  = xend - xstart;
        height = yend - ystart;


        if (driver == EPSON) // if it's an epson
        {
            i=0;

            lcd_send(LCD_COMMAND,PASET);  // page start/end ram
            lcd_send(LCD_DATA,xstart);
            lcd_send(LCD_DATA,xend);


            lcd_send(LCD_COMMAND,CASET);  // column start/end ram
            lcd_send(LCD_DATA,ystart);
            lcd_send(LCD_DATA,yend);
            j++;

            lcd_send(LCD_COMMAND,RAMWR);  // write

            while( i < (width*height)/2 ){
                lcd_send(LCD_DATA,(color>>4)&0x00FF);
                lcd_send(LCD_DATA,((color&0x0F)<<4)|(color>>8));
                lcd_send(LCD_DATA,color&0x0FF);
                i++;

            }
            if((width*height) & 1){
                lcd_send(LCD_DATA,((color>>4)&0x00FF));
                lcd_send(LCD_DATA,(((color&0x0F)<<4)|0x00));
            }
        }
        else if (driver == PHILLIPS)  // otherwise it's a phillips
        {
            i=0;
            lcd_send(LCD_COMMAND,PASETP); // page start/end ram
            lcd_send(LCD_DATA,xstart);
            lcd_send(LCD_DATA,xend);
            ;

            lcd_send(LCD_COMMAND,CASETP); // column start/end ram
            lcd_send(LCD_DATA,ystart);
            lcd_send(LCD_DATA,yend);
            j++;


            lcd_send(LCD_COMMAND,RAMWRP); // write

            while( i < (((width*height)/2)+ (height>width?height:width))  ){
                lcd_send(LCD_DATA,(color>>4)&0x00FF);
                lcd_send(LCD_DATA,((color&0x0F)<<4)|(color>>8));
                lcd_send(LCD_DATA,color&0x0FF);
                i++;

            }
            if((width*height) & 1){
                lcd_send(LCD_DATA,((color>>4)&0x00FF));
                lcd_send(LCD_DATA,(((color&0x0F)<<4)|0x00));
            }
        }
    }


    else
    {
        // best way to draw an unfilled rectangle is to draw four lines
        lcd_setLine(x0, y0, x1, y0, color);
        lcd_setLine(x0, y1, x1, y1, color);
        lcd_setLine(x0, y0, x0, y1, color);
        lcd_setLine(x1, y0, x1, y1, color);
    }

}
