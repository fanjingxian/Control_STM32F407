#include "Motor.h"

// 电机参数（根据实际值修改）
#define POLE_PAIRS 11         // 编码器极对数
#define GEAR_RATIO 21.3f      // 减速比
#define SAMPLE_INTERVAL 0.02f // 采样间隔10ms（单位：秒）

float Encoder_GetSpeed(Motor_t *motor);
void Motor_UpdataSpeed(Motor_t *motor, float duty_percent);

PID_Controller PID = {
  .Kp = 1.7f,
  .Ki = 0.1f, // 0.2
  .Kd = 0.00f,
  .integral = 0.0f,
  .integral_max = 100.0f,
  .output_max = 100.0f,
  .output_min = 53.5f,
  .last_error = 0,
  .previous_D = 0.0f,
  .filtering = 9.0f,
  .DEAD_ZONE = 40.0f, // 调节死区
};
Motor_t Motors[4] = {
    {
        .pid = &PID,
        .PWM_htim = &htim8,
        .PWM_channelA = TIM_CHANNEL_1,
        .PWM_channelB = TIM_CHANNEL_2,
        .pwm_freq = 0,           // PWM频率
        .max_speed = 100.0f,     // 最大速度值(对应PWM占空比最大值)
        .min_speed = 53.5f,      // 最小启动速度(防止电机卡顿)
        .Speed = 0.0f,           // 当前速度值
        .Encoder = &Encoder[0],
        .GetSpeed = Encoder_GetSpeed,
        .UpdataSpeed = Motor_UpdataSpeed,
        //.PID = PID_Motor
    },
    {
        .pid = &PID,
        .PWM_htim = &htim5,
        .PWM_channelA = TIM_CHANNEL_1,
        .PWM_channelB = TIM_CHANNEL_2,
        .pwm_freq = 0,           // PWM频率
        .max_speed = 100.0f,     // 最大速度值(对应PWM占空比最大值)
        .min_speed = 53.5f,      // 最小启动速度(防止电机卡顿)
        .Speed = 0.0f,           // 当前速度值
        .Encoder = &Encoder[1],
        .GetSpeed = Encoder_GetSpeed,
        .UpdataSpeed = Motor_UpdataSpeed,
        //.PID = PID_Motor
    },
    {
        .pid = &PID,
        .PWM_htim = &htim8,
        .PWM_channelA = TIM_CHANNEL_3,
        .PWM_channelB = TIM_CHANNEL_4,
        .pwm_freq = 0,           // PWM频率
        .max_speed = 100.0f,     // 最大速度值(对应PWM占空比最大值)
        .min_speed = 53.5f,      // 最小启动速度(防止电机卡顿)
        .Speed = 0.0f,           // 当前速度值
        .Encoder = &Encoder[2],
        .GetSpeed = Encoder_GetSpeed,
        .UpdataSpeed = Motor_UpdataSpeed,
        //.PID = PID_Motor
    },
    {
        .pid = &PID,
        .PWM_htim = &htim5,
        .PWM_channelA = TIM_CHANNEL_3,
        .PWM_channelB = TIM_CHANNEL_4,
        .pwm_freq = 0,           // PWM频率
        .max_speed = 100.0f,     // 最大速度值(对应PWM占空比最大值)
        .min_speed = 53.5f,      // 最小启动速度(防止电机卡顿)
        .Speed = 0.0f,           // 当前速度值
        .Encoder = &Encoder[3],
        .GetSpeed = Encoder_GetSpeed,
        .UpdataSpeed = Motor_UpdataSpeed,
        //.PID = PID_Motor
    }
  };

void Motor_Init(void)
{
  for (int i = 0; i < 4; i++)
  {
    // 初始化编码器
    HAL_TIM_Encoder_Start(Motors[i].Encoder->Encode_htim, TIM_CHANNEL_ALL);
    // 初始化PWM
    HAL_TIM_PWM_Start(Motors[i].PWM_htim, Motors[i].PWM_channelA);
    HAL_TIM_PWM_Start(Motors[i].PWM_htim, Motors[i].PWM_channelB);
  }
}

float Encoder_GetSpeed(Motor_t *motor)
{
  static float rpm_lest = 0;
  motor->Encoder->Nowcount = motor->Encoder->Encode_htim->Instance->CNT - 0x7FFF;                 // 获取编码器值
  motor->Encoder->Encode_htim->Instance->CNT = 0x7FFF;                                      // 清零编码器计数器
  motor->Encoder->totalCount += motor->Encoder->Nowcount;                                 // 计算增量值
  float rpm = (motor->Encoder->Nowcount / 44.0f) / SAMPLE_INTERVAL * 60.0f / GEAR_RATIO; // 转数 // 转/秒// 转/分钟（电机轴）
  rpm = 0.8 * rpm_lest + 0.2 * rpm;
  motor->Speed = -rpm; // 输出轴转速
  return motor->Speed; // 返回速度值
}

void Motor_UpdataSpeed(Motor_t *motor, float duty_percent)
{
  uint8_t dir = (duty_percent > 0.0f) ? 1 : 0; // 判断方向
                                               //	if(duty_percent>0.0f) duty_percent+=53.5f;
                                               //	if(duty_percent<0.0f) duty_percent-=53.5f;
  duty_percent = fabs(duty_percent);
  __HAL_TIM_SetAutoreload(motor->PWM_htim, 1000000 / 20000 - 1);
  duty_percent = (duty_percent < 0.0f) ? 0.0f : (duty_percent > 100.0f) ? 100.0f
                                                                        : duty_percent;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->PWM_htim);
  uint32_t new_ccr = (uint32_t)((duty_percent / 100.0f) * (arr + 1));
  if (dir)
  {
    __HAL_TIM_SET_COMPARE(motor->PWM_htim, motor->PWM_channelA, 0);
    __HAL_TIM_SET_COMPARE(motor->PWM_htim, motor->PWM_channelB, new_ccr);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(motor->PWM_htim, motor->PWM_channelA, new_ccr);
    __HAL_TIM_SET_COMPARE(motor->PWM_htim, motor->PWM_channelB, 0);
  }
  HAL_TIM_GenerateEvent(motor->PWM_htim, TIM_EVENTSOURCE_UPDATE);
}

// 设置电机速度函数
void Motor_SetSpeed(Motor_t *motor, float speed)
{
  motor->Target_speed = speed; // 设置目标速度
}

// 设置电机PID函数
void Motor_SetPID(Motor_t *motor, float *PID_Set)
{
  motor->pid->Kp = PID_Set[0];
  motor->pid->Ki = PID_Set[1];
  motor->pid->Kd = PID_Set[2];
}

// 获取电机速度函数
int Motor_GetSpeed(Motor_t *motor)
{
  return motor->Speed; // 获取当前速度
}

int cnt = 0, cnt_20ms = 0;
uint8_t i = 0;
uint32_t Pin[3] = {GPIO_PIN_3, GPIO_PIN_1, GPIO_PIN_0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM13)
  { // TIM13中断处理
    cnt++;
    cnt_20ms++;
    if (cnt >= 100)
    { // 每100ms执行一次
      cnt = 0;
      HAL_GPIO_TogglePin(GPIOD, Pin[i]);
      i++;
      if (i == 3)
        i = 0;
    }
    if (cnt_20ms >= 20)
    {
      cnt_20ms = 0;
      for (int i = 0; i < 4; i++)
      {
        Motors[i].GetSpeed(&Motors[i]);                                                               // 获取速度值
        Motors[i].PID_Output = PID_Calculate(Motors[i].pid, Motors[i].Target_speed, Motors[i].Speed); // PID计算
        Motors[i].UpdataSpeed(&Motors[i], (int)Motors[i].PID_Output);                                 // 设置速度值
        // Motors[3].UpdataSpeed(&Motors[3], -53.9f); // 设置速度值
      }
    }
  }
}
