//舵机实现不同角度的旋转
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
  //ledcWrite(c,7);
  //delay(1000);
  //ledcWrite(c,20);
  //delay(1000);
  //ledcWrite(c,32);
  //delay(1000);
  
  //若想实现连续转动则设置for循环
  for (int i=7;i<=32;i++){
  ledcWrite(c,i);
  delay(500);
    }
}
