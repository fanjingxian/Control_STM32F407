#include "Key.h"

#define Key_Numbers 5 
uint8_t Get_Key_Num = 0;
static bool Key_Pressed[Key_Numbers] = {false};

#define RX_BUFFER_SIZE 64 				// 接收缓冲区大小

char rx3_buffer[RX_BUFFER_SIZE]; 	// 接收缓冲区
char received_char3;            	// 临时存储接收的字符
//uint8_t rx3_index = 0;          // 接收缓冲区索引
uint8_t data3_ready = 0;        	// 数据接收标志

float x0, y0;       	//左摇杆
float x1, y1;       	//右摇杆
int button;     			//按键
int button_value[10]; //按键值
int hat_x, hat_y;   	//十字按键

uint8_t Key_GetNum(void) {
    uint8_t Key_value = 0;
    if (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2))  
        Key_value = 1;
    else if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) 
        Key_value = 2;
    else if (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)) 
        Key_value = 3;
    else if (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1)) 
        Key_value = 4;

    return Key_value;
}

uint8_t Key_Scan(void) {
    uint8_t Key_Flag = Key_GetNum();
    for (uint8_t i = 0; i < Key_Numbers; i++) {
        if (Key_Flag == (i + 1)) { 
            if (!Key_Pressed[i]) {  
                Key_Pressed[i] = true;  
                Get_Key_Num = i + 1;  
            }
        } else {
            if (Key_Pressed[i]) {  
                Key_Pressed[i] = false;  
            }
        }
    }
    uint8_t Key_value = Get_Key_Num;
    Get_Key_Num = 0;
    return Key_value;
}


void Joystick_Init(void){
    //HAL_UART_Receive_IT(&huart3, (uint8_t *)&received_char3, 1);
}
// 解析接收到的手柄数据
void parse_joystick_data(const char *data) {
//    if (strncmp(data, "LEFT_STICK", 10) == 0) {
//        
//        if (sscanf(data, "LEFT_STICK (%f, %f)", &x0, &y0) == 2) {
//            //printf("左摇杆数据: x=%.1f, y=%.1f\n", x, y);
//            // 根据左摇杆数据执行操作，例如控制电机或舵机
//        }
//    } else if (strncmp(data, "RIGHT_STICK", 11) == 0) {
//        
//        if (sscanf(data, "RIGHT_STICK (%f, %f)", &x1, &y1) == 2) {
//            //printf("右摇杆数据: x=%.1f, y=%.1f\n", x, y);
//            // 根据右摇杆数据执行操作，例如控制电机或舵机
//        }
//    } else if (strncmp(data, "DOWN", 4) == 0) {
//        
//        if (sscanf(data, "DOWN %d", &button) == 1) {
//            button_value[button] = 1;
//            if(button_value[0] == 1){
//                Servo_tasks[0] = 1;
//                //QR_Proc();
//            } 
//            //printf("按键 %d 按下\n", button);
//            // 根据按键状态执行操作
//        }
//    } else if (strncmp(data, "UP", 2) == 0) {
//        if (sscanf(data, "UP %d", &button) == 1) {
//            button_value[button] = 0;
//            //printf("按键 %d 松开\n", button);
//            // 根据按键状态执行操作
//        }
//    } else if (strncmp(data, "HAT", 3) == 0) {
//        
//        if (sscanf(data, "HAT 0 (%d, %d)", &hat_x, &hat_y) == 2) {
//            //printf("十字键: x=%d, y=%d\n", hat_x, hat_y);
//            // 根据十字键数据执行操作
//        }
//    } 
}

void joystick_Process(void){
//    if (data3_ready) {
//            parse_joystick_data(rx3_buffer); // 解析接收到的数据
//            data3_ready = 0;                // 重置标志位
//        }
}
/*--------------------------------------------------------结束-------------------------------------------------------------*/

