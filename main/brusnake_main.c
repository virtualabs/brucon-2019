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
#include "touch.h"
#include "gameover.h"

#define DELAY 200000

typedef enum {
  NONE,
  UP,
  DOWN,
  LEFT,
  RIGHT
} t_direction;

/* Game state. */
typedef struct {
  uint8_t map[33][33];
  int head_x;
  int head_y;
  int tail_x;
  int tail_y;
  int apple_x;
  int apple_y;
  int speed;
  int add_tail;
  int steps;
  uint8_t direction;
  int size;
} t_gamestate, *p_gamestate;

t_gamestate game;

void place_apple(p_gamestate state)
{
  /* Place randomly our apple. */
  do
  {
    state->apple_x = esp_random()%33;
    state->apple_y = esp_random()%33;
  } while(state->map[state->apple_x][state->apple_y]!=NONE);

}

/**
 * Initialize our game.
 **/
void init_game(p_gamestate state)
{
  /* Clear map. */
  memset(state->map, NONE, 33*33);

  /* Place our snake (from [14,16] to [18,16], size:5) */
  state->head_x = 18;
  state->head_y = 16;
  state->tail_x = 14;
  state->tail_y = 16;
  state->size = 5;
  state->speed = 30;
  state->steps = 0;
  state->add_tail = 0;

  /* Mark our snake on the map. */
  state->map[14][16] = RIGHT;
  state->map[15][16] = RIGHT;
  state->map[16][16] = RIGHT;
  state->map[17][16] = RIGHT;
  state->map[18][16] = RIGHT;
  state->direction = RIGHT;

  /* Initialize screen. */
  lcd_region_fill(0,0,132,132,B12_WHITE);

  /* Place randomly our apple. */
  place_apple(state);
  lcd_region_fill(state->apple_x*4, state->apple_y*4, 4,4,B12_RED);

  /* Place snake. */
  lcd_region_fill(state->head_x*4, state->head_y*4, state->size*4, 4, BLACK);

}

void change_direction(p_gamestate state, t_direction dir)
{
  if ((dir==state->direction) ||
      ((dir==RIGHT) && (state->direction==LEFT)) ||
      ((dir==LEFT) && (state->direction==RIGHT)) ||
      ((dir==UP) && (state->direction==DOWN)) ||
      ((dir==DOWN) && (state->direction==UP))
  )
    return;

  /* Keep going in this direction. */
  state->direction = dir;
}

int render_game(p_gamestate state)
{
  int prev_tail_x;
  int prev_tail_y;

  /* Shall we update the screen ? (the lower the speed, the quicker we refresh it) */
  if ((++state->steps)<state->speed)
    return;
  else
    state->steps=0;

  /* We do not allow these moves: */
  if (((state->direction == UP) && (state->map[state->head_x][state->head_y] == DOWN)) ||
      ((state->direction == DOWN) && (state->map[state->head_x][state->head_y] == UP)) ||
      ((state->direction == RIGHT) && (state->map[state->head_x][state->head_y] == LEFT)) ||
      ((state->direction == LEFT) && (state->map[state->head_x][state->head_y] == RIGHT))
    )
    /* Keep current direction. */
    state->direction = state->map[state->head_x][state->head_y];

  /* Note: Other direction changes are allowed. */

  /* Mark the direction change in the map. */
  state->map[state->head_x][state->head_y] = state->direction;

  /* Move the snake's head according to map. */
  switch(state->map[state->head_x][state->head_y])
  {
    /* Going up. */
    case UP:
      printf("going up\n");
      state->head_y++;
      if (state->head_y>32)
        state->head_y=0;
      break;

    case DOWN:
      printf("going down\n");
      state->head_y--;
      if (state->head_y<0)
        state->head_y=32;
      break;

    case LEFT:
      printf("going left\n");
      state->head_x--;
      if (state->head_x<0)
        state->head_x=32;
      break;

    case RIGHT:
      printf("going right\n");
      state->head_x++;
      if (state->head_x>32)
        state->head_x=0;
      break;
  }

  /* Have we met an obstacle ? */
  if (state->map[state->head_x][state->head_y] != NONE)
  {
    /* Game over */
    for (int w=0;w<132;w+=2) {
      lcd_line(0,w,132,w,BLACK);
      lcd_line(0,w+1,132,w+1,BLACK);
      lcd_commit();
    }

    for (int w=0;w<132;w+=2) {
      lcd_line(0,w,132,w,B12_WHITE);
      lcd_line(0,w+1,132,w+1,B12_WHITE);
      lcd_commit();
    }

    return 0;
  }

  /* Mark the direction in the map. */
  state->map[state->head_x][state->head_y] = state->direction;

  /* Draw the apple if not eaten. */
  if ((state->head_x!=state->apple_x)||(state->head_y!=state->apple_y))
  {
    lcd_region_fill(state->apple_x*4, state->apple_y*4, 4,4,B12_RED);

    if (state->add_tail == 0)
    {
      /* Follow tail. */
      prev_tail_x = state->tail_x;
      prev_tail_y = state->tail_y;

      lcd_region_fill(state->tail_x*4, state->tail_y*4, 4,4,B12_WHITE);
      switch(state->map[state->tail_x][state->tail_y])
      {
        /* Going up. */
        case UP:
          printf("tail going up\n");
          state->tail_y++;
          if (state->tail_y>32)
            state->tail_y=0;
          break;

        case DOWN:
          printf("tail going down\n");
          state->tail_y--;
          if (state->tail_y<0)
            state->tail_y=32;
          break;

        case LEFT:
          printf("tail going left\n");
          state->tail_x--;
          if (state->tail_x<0)
            state->tail_x=32;
          break;

        case RIGHT:
          printf("tail going right\n");
          state->tail_x++;
          if (state->tail_x>32)
            state->tail_x=0;
          break;
      }

      /* Remove tail block. */
      state->map[prev_tail_x][prev_tail_y] = NONE;
    }
    else
      state->add_tail--;

    /* Draw the new head position */
    lcd_region_fill(state->head_x*4, state->head_y*4, 4,4,BLACK);

  }
  else
  {
    /* Draw the new head position */
    lcd_region_fill(state->head_x*4, state->head_y*4, 4,4,BLACK);

    /* Generate new apple position */
    place_apple(state);

    if (state->speed > 0)
      state->speed--;

    state->add_tail=3;
  }

  lcd_commit();

  return 1;
}

void game_task(void *parameter)
{
  init_game(&game);


  while(1){

    /* Change direction if corresponding touch is pressed. */
    if (touch_up_pressed())
      change_direction(&game, UP);
    if (touch_right_pressed())
      change_direction(&game, RIGHT);
    if (touch_left_pressed())
      change_direction(&game, LEFT);
    if (touch_down_pressed())
      change_direction(&game, DOWN);

    /* Move head and tail. */
    if (!render_game(&game))
    {

      init_game(&game);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void app_main(void)
{
    int i;
    int x,y;

    /* Start LCD and enable backlight */
    init_lcd(PHILLIPS);
    init_touchpad();
    switchbacklight(1);

    /* Create game task. */
    xTaskCreate(game_task, "gameTask", 10000, NULL, 1, NULL);
    //lcd_load_bitmap(gameover);
    //lcd_commit();
}
