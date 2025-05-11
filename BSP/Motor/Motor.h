#ifndef __MOTOR_H
#define __MOTOR_H

//#include "headfile.h"

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "PID.h"
#include "math.h"

typedef struct Encoder
{
    TIM_HandleTypeDef *Encode_htim; //编码器定时器句柄
    int32_t Nowcount;     //当前计数值
    int32_t lastCount;   //上一次计数值
    int32_t totalCount;  //总计数值
    float speed;         //电机转速
}Encoder_t;

typedef struct Motor {
    PID_Controller *pid;
    TIM_HandleTypeDef *PWM_htim;
    uint32_t PWM_channelA;
    uint32_t PWM_channelB;

    int pwm_freq;
    float max_speed;
    float min_speed;

    float Target_speed;
    float Speed;
    Encoder_t *Encoder;
    // uint32_t Encode_channelA;
    // uint32_t Encode_channelB;
    float PID_Output;
    // 使用结构体标签作为参数类型
    float (*GetSpeed)(struct Motor*);
    void (*UpdataSpeed)(struct Motor*, float);
} Motor_t;

Encoder_t Encoder[4] = {
    {
        .Encode_htim = &htim3,
        .Nowcount = 0,   // 当前计数值
        .lastCount = 0,   // 上一次计数值
        .totalCount = 0,  // 总计数值
        .speed = 0,       // 电机转速
        
    },
    {
        .Encode_htim = &htim1,
        .Nowcount = 0,   // 当前计数值
        .lastCount = 0,   // 上一次计数值
        .totalCount = 0,  // 总计数值
        .speed = 0,       // 电机转速
        
    },
    {
        .Encode_htim = &htim4,
        .Nowcount = 0,   // 当前计数值
        .lastCount = 0,   // 上一次计数值
        .totalCount = 0,  // 总计数值
        .speed = 0,       // 电机转速
        
    },
    {
        .Encode_htim = &htim2,
        .Nowcount = 0,   // 当前计数值
        .lastCount = 0,   // 上一次计数值
        .totalCount = 0,  // 总计数值
        .speed = 0,       // 电机转速
        
    }
};


extern Motor_t Motors[4]; // 电机数组

#define MOTOR_A  (&Motors[0])
#define MOTOR_B  (&Motors[1])
#define MOTOR_C  (&Motors[2])
#define MOTOR_D  (&Motors[3])
void Motor_Init(void); //电机初始化函数
void Motor_SetSpeed(Motor_t *motor, float speed);//设置电机速度函数
int  Motor_GetSpeed(Motor_t *motor); //获取电机速度函数
void Motor_SetPID(Motor_t *motor, float *PID_Set);
#endif // !__MOTOR_H


