//舵机与电机结合到一起

#include "driver/mcpwm.h"

void setup() {
  Serial.begin(115200);

  //初始化gpio口
  mcpwm_gpio_init(MCPWM_UNIT_1,MCPWM1A,13);//设置舵机脚
  mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM0A,26);//设置电机
  mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM0B,27);//设置电机

  //通过mcpwm_config_t结构体为定时器设置频率和初始值  舵机
  mcpwm_config_t servo_pwm_config;
  servo_pwm_config.frequency = 50;
  servo_pwm_config.cmpr_a = 0;
  servo_pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  servo_pwm_config.counter_mode = MCPWM_UP_COUNTER;

  //通过mcpwm_config_t结构体为定时器设置频率和初始值  电机
  mcpwm_config_t motor_pwm_config = {
    .frequency = 1000,
    .cmpr_a = 0,
    .cmpr_b = 0,
    .duty_mode = MCPWM_DUTY_MODE_0,
    .counter_mode = MCPWM_UP_COUNTER,
    };
  
  //使用以上设置配置PWM1A
  mcpwm_init(MCPWM_UNIT_1,MCPWM_TIMER_1,&servo_pwm_config);
  //使用以上设置配置PWM0A和PWM0B
  mcpwm_init(MCPWM_UNIT_0,MCPWM_TIMER_0,&servo_pwm_config);
}

void loop() {
  Serial.println("Setting motor pwm success!");
  mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_A,100);//全速前进
  mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_B,0);

  mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_A,7.5);//直行
  delay(5000);
  mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_A,2.5);//左转
  delay(5000);
  mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_A,7.5);
  delay(5000);
  mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_A,12.5);//右转
  delay(5000);
}
