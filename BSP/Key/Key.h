#ifndef __KEY_H__
#define __KEY_H__

#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"
#include "main.h"
#include "tim.h"


uint8_t Key_GetNum(void);

uint8_t Key_Scan(void);
void Joystick_Init(void);
void joystick_Process(void);
#endif // __KEY_H__
