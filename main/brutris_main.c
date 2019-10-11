/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nokialcd.h"
#include "test.h"
#include "troll1.h"
#include "troll2.h"
#include "troll3.h"
#include "troll4.h"
//#include "troll5.h"

#define DELAY 200000

uint16_t region[16]={0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
void app_main(void)
{
  //int i;
    /* Start LCD and enable backlight */
    init_lcd(PHILLIPS);
    switchbacklight(1);

    while(1){
      //fillframe12B(BLACK);
      //send_frame();
      //commit();
      //ets_delay_us(1000000);
      //fillframe12B(B12_NAVY);
      //send_frame();
      //commit();
      //ets_delay_us(1000000);
      //fillframe12B(B12_GREEN);
      //send_frame();
      //commit();
      //ets_delay_us(1000000);
    //}
    load_bitmap(troll0);
    /* Try bitblt */
    bitblt(10,10,4,4,region);
    lcd_commit();
    ets_delay_us(DELAY);
    load_bitmap(troll1);
    bitblt(10,10,4,4,region);
    lcd_commit();
    ets_delay_us(DELAY);
    load_bitmap(troll2);
    bitblt(10,10,4,4,region);
    lcd_commit();
    ets_delay_us(DELAY);
    load_bitmap(troll3);
    bitblt(10,10,4,4,region);
    lcd_commit();
    ets_delay_us(DELAY);
    load_bitmap(troll2);
    bitblt(10,10,4,4,region);
    lcd_commit();
    ets_delay_us(DELAY);
    load_bitmap(troll1);
    bitblt(10,10,4,4,region);
    lcd_commit();
    ets_delay_us(DELAY);
    /*
    load_bitmap(troll4);
    send_frame();
    ets_delay_us(70000);
    */
    /*
    fillframe12B(B12_WHITE);
    fillframe12B(BLACK);
    */
  }

}
