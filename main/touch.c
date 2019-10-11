#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_pad.h"

#define TOUCH_PAD_NO_CHANGE   (-1)
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_FILTER_MODE_EN  (1)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

#define TOUCH_LEFT    4
#define TOUCH_RIGHT   6
#define TOUCH_UP      5
#define TOUCH_DOWN    7
#define TOUCH_B       8
#define TOUCH_A       9

void init_touchpad(void)
{
    int i;
    touch_pad_init();
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    for (i = 0;i< TOUCH_PAD_MAX;i++) {
        touch_pad_config(i, TOUCH_THRESH_NO_USE);
    }
}

bool is_key_pressed(int key)
{
  uint16_t touch_filter_value;
  touch_pad_read_filtered(key, &touch_filter_value);
  //printf("v%d: %d\n", key, touch_filter_value);
  return ((touch_filter_value < 800) && touch_filter_value!=0);
}

bool touch_left_pressed(void)
{
  return is_key_pressed(TOUCH_LEFT);
}

bool touch_right_pressed(void)
{
  return is_key_pressed(TOUCH_RIGHT);
}

bool touch_up_pressed(void)
{
  return is_key_pressed(TOUCH_UP);
}

bool touch_down_pressed(void)
{
  return is_key_pressed(TOUCH_DOWN);
}

bool touch_A_pressed(void)
{
  return is_key_pressed(TOUCH_A);
}

bool touch_B_pressed(void)
{
  return is_key_pressed(TOUCH_B);
}
