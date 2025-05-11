#ifndef __PID_H
#define __PID_H__

#include "main.h"
#include "math.h"
#include "tim.h"
#include "gpio.h"
//#include "Motor.h"

typedef struct {
    float Kp;   
    float Ki;
    float Kd;
    float integral;         // 积分项   
    float integral_max;     // 积分项限幅
    float output_max ;       // 输出限幅
		float output_min ;
    float DEAD_ZONE;    // 调节死区
    float filtering;    // 滤波系数
    float last_error;     // 上一次误差
    float previous_D;       // 滤波后的微分项历史值
} PID_Controller;

// PID 算法
float PID_Calculate(PID_Controller* pid, float target, float actual);
// /*-------------------------------PID双环控制-----------------------------------------*/
// // 双环控制器结构体
// typedef struct {
//     PID_Controller pos_pid;  // 位置环
//     PID_Controller vel_pid;  // 速度环
//     float target_pos;        // 目标位置
//     float target_vel;        // 速度环设定值
//     float vel_feedforward;   // 速度前馈增益
//     float dt;                // 控制周期(秒)
// } DualLoop_Controller;

// // 双环控制器初始化
// void DualLoop_Init(DualLoop_Controller* ctrl, float dt);

// // 双环控制更新
// float DualLoop_Update(DualLoop_Controller* ctrl, float actual_pos, float actual_vel);

#endif // !__PID_H
/*********************** 使用示例 ***************************
DualLoop_Controller ctrl;
DualLoop_Init(&ctrl, 0.01); // 10ms控制周期

while(1) {
    float actual_pos = GetPosition();  // 获取实际位置
    float actual_vel = GetVelocity();  // 获取实际速度
    
    // 设置目标位置
    ctrl.target_pos = DesiredPosition(); 
    
    // 计算控制量
    float output = DualLoop_Update(&ctrl, actual_pos, actual_vel);
    
    // 输出到执行机构
    SetMotorOutput(output);
    
    Delay(10); // 10ms延时
}
************************************************************/
/*------------------------------------结束-----------------------------------------*/


/*------------------------------------S型曲线-----------------------------------------*/
// float fhan(float x1, float x2, float r, float h0) {
//     float d = r * h0 * h0;
//     float a0 = h0 * x2;
//     float y = x1 + a0;
//     float a1 = sqrtf(d*(d + 8*fabsf(y)));
//     float a2 = a0 + copysignf(0.5f*(a1-d), y);
//     float sy = (y > 0 ? 1 : -1) * (fabsf(y) > d ? 1 : 0);
//     float a = (a0 + y - a2)*sy + a2;
//     float sa = (a > 0 ? 1 : -1) * (fabsf(a) > d ? 1 : 0);
//     return -r*(a/d - sa)*r - r*sa;
// }

// typedef struct {
//     float target_pos;    // 目标位置
//     float current_pos;   // 当前位置
//     float current_vel;   // 当前速度
//     float max_vel;       // 最大速度
//     float max_acc;       // 最大加速度
//     float max_jerk;      // 最大加加速度
//     float dt;            // 控制周期
//     float h0;            // 滤波因子
//     float r;             // 速度因子
// } SCurve_Controller;

// void SCurve_Update(SCurve_Controller* ctrl) {
//     // 第一阶段：位置跟踪
//     float dx1 = ctrl->current_vel;
//     float ddx1 = fhan(ctrl->target_pos - ctrl->current_pos, dx1, ctrl->r, ctrl->h0);
    
//     // 第二阶段：速度限幅
//     float temp_vel = ctrl->current_vel + ddx1 * ctrl->dt;
//     if(fabsf(temp_vel) > ctrl->max_vel) {
//         temp_vel = copysignf(ctrl->max_vel, temp_vel);
//     }
    
//     // 第三阶段：加速度限幅
//     float acc = (temp_vel - ctrl->current_vel) / ctrl->dt;
//     if(fabsf(acc) > ctrl->max_acc) {
//         acc = copysignf(ctrl->max_acc, acc);
//         temp_vel = ctrl->current_vel + acc * ctrl->dt;
//     }
    
//     // 第四阶段：加加速度限幅
//     float jerk = (acc - (ctrl->current_vel - ctrl->current_vel_prev)/ctrl->dt) / ctrl->dt;
//     if(fabsf(jerk) > ctrl->max_jerk) {
//         jerk = copysignf(ctrl->max_jerk, jerk);
//         acc = (ctrl->current_vel - ctrl->current_vel_prev)/ctrl->dt + jerk * ctrl->dt;
//         temp_vel = ctrl->current_vel + acc * ctrl->dt;
//     }
    
//     // 更新状态
//     ctrl->current_vel_prev = ctrl->current_vel;
//     ctrl->current_vel = temp_vel;
//     ctrl->current_pos += ctrl->current_vel * ctrl->dt;
// }
// /*-------------------应用示例--------------------------------------------*/
// // 系统初始化
// SCurve_Controller ctrl = {
//     .target_pos = 0,
//     .current_pos = 0,
//     .current_vel = 0,
//     .max_vel = 1000,    // 单位：脉冲/秒
//     .max_acc = 5000,    // 单位：脉冲/秒²
//     .max_jerk = 20000,  // 单位：脉冲/秒³
//     .dt = 0.001,        // 1ms控制周期
//     .h0 = 0.0015,       // 1.5*dt
//     .r = 3.0*1000/(2*5000*0.001) // 计算得到300
// };
// void Motor_Control_Loop() {
//     // 获取实际位置（编码器反馈）
//     float actual_pos = Encoder_GetPosition();
    
//     // 更新控制器状态
//     ctrl.current_pos = actual_pos;
    
//     // 设置新目标位置
//     if(Need_New_Target()){
//         ctrl.target_pos = Get_New_Target();
//     }
    
//     // 执行S曲线计算
//     SCurve_Update(&ctrl);
    
//     // 将速度指令发送给驱动器
//     //Motor_SetSpeed(ctrl.current_vel);
    
//     // 1ms延时
//     Delay(1);
// }

// /*------------------------------------结束-----------------------------------------*/

// // 扩展结构体
// typedef struct {
//     SCurve_Controller s_curve;
//     PID_Controller vel_pid;
//     float vel_feedforward;
// } Motion_Controller;

// // 控制函数
// float Motion_Update(Motion_Controller* ctrl, float actual_pos) {
//     // S曲线生成目标速度
//     SCurve_Update(&ctrl->s_curve);
    
//     // 获取实际速度
//     float actual_vel = (actual_pos - ctrl->s_curve.prev_pos)/ctrl->s_curve.dt;
    
//     // PID计算
//     float pid_out = PID_Update(&ctrl->vel_pid, 
//                             ctrl->s_curve.current_vel - actual_vel);
    
//     // 前馈补偿
//     float total_out = pid_out + ctrl->vel_feedforward * ctrl->s_curve.current_vel;
    
//     ctrl->s_curve.prev_pos = actual_pos;
//     return total_out;
// }







