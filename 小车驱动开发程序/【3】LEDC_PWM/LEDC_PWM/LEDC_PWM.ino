//模拟呼吸灯 利用pwm
const int led = 33;

//pwm信号  四部分组成 ①频率 即1s内生成几个周期 如50HZ表示1s内生成50个周期 每一个周期20ms ②信号通道 16个 用来计数 ③分辨率 8 精度 ④占空比 高电平所占比重/总
const int freq = 2000; //频率
const int resolution = 8; //分辨率
const int channel = 0; //通道
const int duty_cycle = 0; //占空比


void setup() {
  //配置ledc通道
  ledcSetup(channel,freq,resolution);
  //将Pin脚放置通道中
  ledcAttachPin(led,channel);
}

void loop() {
  //示例程序，用来控制小灯的亮度，利用它来实现呼吸灯(从最亮到最暗再到最亮)
  //ledcWrite(channel,0);  //占空比为0时，应该是最亮的 相当于GPIO脚是低电平 占空比为255时是很暗很暗的,再大就不亮了

  for (int a = 0;a<=255;a++){
    delay(10);
    ledcWrite(channel,a);
    }

  for (int a = 255;a>=0;a--){
    delay(10);
    ledcWrite(channel,a);
    }

}
