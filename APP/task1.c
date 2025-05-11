#include "task1.h"

void Usart1Printf(const char *format, ...);
void Uart_Transmit_Task(void);
void OLED_Task(void);
void Key_Task(void);
void Motor_Task(void);

uint8_t UartTxBuf[100];
int num = 0;
int8_t Key_ID = 0;
float speed =10.0f;

float pid_num[3] = {5.0f,0.4f,0.0f};
uint8_t pid_index=0,Set_mode=0;


void OLED_Task(void){
	static uint32_t OLED_Task_Delay=0;
	if(HAL_GetTick() - OLED_Task_Delay >= 2){
		OLED_Task_Delay = HAL_GetTick();
		OLED_PrintString(0,  0, "KP", &font16x16, OLED_COLOR_NORMAL);
		OLED_PrintString(0, 16, "KI", &font16x16, OLED_COLOR_NORMAL);
		OLED_PrintString(0, 32, "SP", &font16x16, OLED_COLOR_NORMAL);
		OLED_PrintString(0, 48, "IN", &font16x16, OLED_COLOR_NORMAL);
		OLED_PrintString(0, 64, "MO", &font16x16, OLED_COLOR_NORMAL);
		
		OLED_PrintFloatNum(30,  0, pid_num[0], &font16x16, OLED_COLOR_NORMAL);
		OLED_PrintFloatNum(30, 16, pid_num[1], &font16x16, OLED_COLOR_NORMAL);
		OLED_PrintFloatNum(30, 32, MOTOR_D->Target_speed, &font16x16, OLED_COLOR_NORMAL);
		OLED_PrintNum(30, 48, pid_index, &font16x16, OLED_COLOR_NORMAL);
		OLED_PrintNum(30, 64, Set_mode, &font16x16, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
		//num++;
	}
}

void Key_Task(void){
	static uint32_t Key_Task_Delay=0;
	if(HAL_GetTick() - Key_Task_Delay >= 20){
		Key_Task_Delay = HAL_GetTick();
		uint8_t Key_Num = Key_Scan();
			switch (Key_Num) {
				case 1:
					//Key_ID =1;
				if(Set_mode){
					pid_num[pid_index] += 0.1f;
				}else{
					speed+=10.0f;
				}
					break;
				case 2:
					if(Set_mode){
					pid_num[pid_index] -=  0.1f;
				}else{
					speed-=10.0f;
				}
					break;
				case 3:
					if(Set_mode){
						if(++pid_index==2)pid_index=0;
					}
					break;
				case 4:
					if(++Set_mode==2)Set_mode=0;
					break;
				default:break;
				}
			//Motor_SetPID(MOTOR_D,pid_num);
				//__HAL_TIM_SET_COMPARE(MOTOR_D->PWM_htim, MOTOR_D->PWM_channelA, 500);
		}
}

void Motor_Task(void){
    Motor_SetSpeed(MOTOR_D, speed); // 设置电机速度
		Motor_SetSpeed(MOTOR_B, -100.0f); // 设置电机速度
}

void Uart_Transmit_Task(void){
	static uint32_t Uart_Transmit_Task_Delay=0;
	if(HAL_GetTick() - Uart_Transmit_Task_Delay >= 80){
		Uart_Transmit_Task_Delay = HAL_GetTick();
		
		Usart1Printf("speed:%f,OUT:%f,%f,\r\n",MOTOR_D->Speed,MOTOR_D->Target_speed,MOTOR_D->PID_Output);
	}
}

void Task_Run(void){
		HAL_TIM_Base_Start_IT(&htim13); // 启动定时器中断
		Motor_Init();
		OLED_Init();
		IMU_Init();
		OLED_NewFrame();
	for(;;){
		OLED_Task();
		Key_Task();
		Motor_Task();
		Uart_Transmit_Task();
	}
}

/*串口打印重定向*/
void Usart1Printf(const char *format, ...)
{
	uint16_t len;
	va_list args;
	va_start(args, format);
	len = vsnprintf((char *)UartTxBuf, sizeof(UartTxBuf), (char *)format, args);
	va_end(args);
	HAL_UART_Transmit(&huart1, UartTxBuf, len, HAL_MAX_DELAY);
}
