//舵机 通过pwm信号来确定角度,其频率为固定值(50HZ)....一个周期为20ms,高电平固定在0.5-2.5ms,由于0.5ms代表0度,2.5ms代表180度,故1ms代表45度
//0.5ms的占空比为 0.5/20 = 0.025 = 2.5%  即0度对应的占空比为2.5 同理180度对应的占空比为12.5 90度对应的占空比为7.5
//90度是舵机初始化的程序,比较重要
const int a = 15; //舵机A 15号管脚

//pwm
const int f = 50; //频率
const int r = 8; //分辨率
const int c = 0; //通道 用0号
const int d = 20; //占空比 实际取值在7-32之间
 
void setup() {
  ledcSetup(c,f,r);
  ledcAttachPin(a,c);
}

void loop() {
  ledcWrite(c,d);

}
