#include "Key.h"

#define Key_Numbers 5 
uint8_t Get_Key_Num = 0;
static bool Key_Pressed[Key_Numbers] = {false};

#define RX_BUFFER_SIZE 64 				// ���ջ�������С

char rx3_buffer[RX_BUFFER_SIZE]; 	// ���ջ�����
char received_char3;            	// ��ʱ�洢���յ��ַ�
//uint8_t rx3_index = 0;          // ���ջ���������
uint8_t data3_ready = 0;        	// ���ݽ��ձ�־

float x0, y0;       	//��ҡ��
float x1, y1;       	//��ҡ��
int button;     			//����
int button_value[10]; //����ֵ
int hat_x, hat_y;   	//ʮ�ְ���

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
// �������յ����ֱ�����
void parse_joystick_data(const char *data) {
//    if (strncmp(data, "LEFT_STICK", 10) == 0) {
//        
//        if (sscanf(data, "LEFT_STICK (%f, %f)", &x0, &y0) == 2) {
//            //printf("��ҡ������: x=%.1f, y=%.1f\n", x, y);
//            // ������ҡ������ִ�в�����������Ƶ������
//        }
//    } else if (strncmp(data, "RIGHT_STICK", 11) == 0) {
//        
//        if (sscanf(data, "RIGHT_STICK (%f, %f)", &x1, &y1) == 2) {
//            //printf("��ҡ������: x=%.1f, y=%.1f\n", x, y);
//            // ������ҡ������ִ�в�����������Ƶ������
//        }
//    } else if (strncmp(data, "DOWN", 4) == 0) {
//        
//        if (sscanf(data, "DOWN %d", &button) == 1) {
//            button_value[button] = 1;
//            if(button_value[0] == 1){
//                Servo_tasks[0] = 1;
//                //QR_Proc();
//            } 
//            //printf("���� %d ����\n", button);
//            // ���ݰ���״ִ̬�в���
//        }
//    } else if (strncmp(data, "UP", 2) == 0) {
//        if (sscanf(data, "UP %d", &button) == 1) {
//            button_value[button] = 0;
//            //printf("���� %d �ɿ�\n", button);
//            // ���ݰ���״ִ̬�в���
//        }
//    } else if (strncmp(data, "HAT", 3) == 0) {
//        
//        if (sscanf(data, "HAT 0 (%d, %d)", &hat_x, &hat_y) == 2) {
//            //printf("ʮ�ּ�: x=%d, y=%d\n", hat_x, hat_y);
//            // ����ʮ�ּ�����ִ�в���
//        }
//    } 
}

void joystick_Process(void){
//    if (data3_ready) {
//            parse_joystick_data(rx3_buffer); // �������յ�������
//            data3_ready = 0;                // ���ñ�־λ
//        }
}
/*--------------------------------------------------------����-------------------------------------------------------------*/

