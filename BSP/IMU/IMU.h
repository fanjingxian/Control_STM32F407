#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"
#include "main.h"
#include "usart.h"

void jy901p_ReceiveData(uint8_t RxData);

extern float Roll,Pitch,Yaw;
void IMU_Init(void);

#endif 
