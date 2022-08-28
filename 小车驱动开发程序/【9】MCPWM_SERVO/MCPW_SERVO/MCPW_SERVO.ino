//舵机的调试程序

#include "driver/mcpwm.h"

void setup() {
  Serial.begin(115200); //这是查看窗口的方法，让窗口输出一个115200的波特率

  //用选定的MCPWM_UNIT_1来初始化gpio口
  mcpwm_gpio_init(MCPWM_UNIT_1,MCPWM1A,13);  //用的1号PWM单元,该单元中的第一组定时器，配置到!13!号管脚
  
  //通过mcpwm_config_t结构体为定时器设置频率和初始值   结构体部分的另一种写法
  mcpwm_config_t servo_pwm_config;
  servo_pwm_config.frequency = 50;
  servo_pwm_config.cmpr_a = 0;//初始占空比 0%
  servo_pwm_config.duty_mode = MCPWM_DUTY_MODE_0;//占空比的类型
  servo_pwm_config.counter_mode = MCPWM_UP_COUNTER;//计数器的类型

  //使用以上设置配置PWM1A
  mcpwm_init(MCPWM_UNIT_1,MCPWM_TIMER_1,&servo_pwm_config);
  
}

void loop() {
  Serial.println("Setting motor pwm success!");//在窗口中输出这样的一段话
  //mcpwm_stop(MCPWM_UNIT_1,MCPWM_TIMER_1);
  mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_A,7.5); //占空比取值2.5%-12.5% 7.5%是90度
  delay(1000);
  mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_A,2.5); //0度
  delay(1000);
  mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_A,7.5);
  delay(1000);
  mcpwm_set_duty(MCPWM_UNIT_1,MCPWM_TIMER_1,MCPWM_OPR_A,12.5); //180度
  delay(1000);

}
