//让电机转动,先加速在减速再加速...
const int a = 26; //a由26号脚控制
const int b = 27; //b由27号脚控制

//pwm 给电机的pwm配置一些准备工作
const int freq = 2000; //频率  1s内2000次
const int resolution = 8;//分辨率  占空比位数精度 8的话就是256位 占空比取值就是0-255
const int channel_A = 0;//通道0  设置信号通道 信号通道一共有16个
const int channel_B = 1;//通道1
const int duty_cycle = 255; //占空比 高电平占周期的比重 可以通过调整占空比生成不同的pwm信号(模拟电压信号)



void setup() {
  //配置ledc通道
  ledcSetup(channel_A,freq,resolution);
  ledcSetup(channel_B,freq,resolution);
  //通过pwm设置管脚的转速值
  ledcAttachPin(a,channel_A);
  ledcAttachPin(b,channel_B);

}

void loop() {
  //这段是A控制B，即正转，且缓慢加速
  for(int i = 0;i<=255;i = i+5){
    ledcWrite(channel_A,i);
    ledcWrite(channel_B,0); 
    delay(50);
    }
   //这段是保持最大的速度匀速转
   ledcWrite(channel_A,duty_cycle);
   delay(5000);

  //这段是A控制B，即正转，且缓慢减速
  for(int i = 255;i>=0;i = i-10){
    ledcWrite(channel_A,i);
    ledcWrite(channel_B,0); 
    delay(50);
  }
}
