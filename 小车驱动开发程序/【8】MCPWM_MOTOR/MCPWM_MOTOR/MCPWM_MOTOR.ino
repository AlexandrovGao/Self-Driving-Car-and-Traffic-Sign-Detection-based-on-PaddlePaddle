//mcpwm单元的介绍 控制电机舵机用的(电机驱控：motorcontrol) 前面介绍的ledc主要是控制小灯的(虽然也可控制电机)
//目标:前进后退来回切换

#include "driver/mcpwm.h" //导入模块



void setup() {
  //用选定的MCPWM_UNIT_0来初始化gpio口         ps：在芯片中共有两个电机驱动单元，标号分别为0和1，在每一个单元中又有3个定时器(用来计算占空比)，每个定时器又有两组信号，分别发出信号A和信号B
  mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM0A,26); //0号单元 定时器0 信号A 26号管脚
  mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM0B,27); //0号单元 定时器0 信号B 27号管脚

  //通过mcpwm_config_t结构体为定时器设置频率和初始值
  mcpwm_config_t motor_pwm_config = {
    .frequency = 1000, //频率
    .cmpr_a = 0, //A的占空比(%)
    .cmpr_b = 0, //B的占空比(%)
    .duty_mode = MCPWM_DUTY_MODE_0, //占空比模式(高电平) --一般不用改
    .counter_mode = MCPWM_UP_COUNTER, //计数器模式(上位计数)  --一般不用改 
    };

  //使用以上设置配置PWM0A和PWM0B
  {
    mcpwm_init(MCPWM_UNIT_0,MCPWM_TIMER_0,&motor_pwm_config);
    };
}

void loop() {
  //PWM
  //如果希望禁止可以都设成一样的数,如都设为30
  //全速后退
  mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_A,0);//设置占空比
  mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_B,100);//这里设100和255无区别 由于是百分比
  mcpwm_start(MCPWM_UNIT_0,MCPWM_TIMER_0);//开始输出信号(mcpwm单元,定时器)
  delay(5000); //持续5s
  mcpwm_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);//停止输出
  //全速前进
  mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_A,100);//设置占空比
  mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_B,0);
  mcpwm_start(MCPWM_UNIT_0,MCPWM_TIMER_0);//开始输出信号(mcpwm单元,定时器)
  delay(5000);
  mcpwm_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
 
}
