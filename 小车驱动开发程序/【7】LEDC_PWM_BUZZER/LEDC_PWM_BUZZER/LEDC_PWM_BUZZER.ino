//蜂鸣器 25号管脚
const int buzzer = 25;

//pwm
const int f = 1000;
const int c = 0;
const int r = 8;
const int d = 128 ;//占空比 取值在0-255 控制响度

void setup() {
  ledcSetup(c,f,r);
  ledcAttachPin(buzzer,c);
}

void loop() {
  //固定频率修改占空比查看响度变化
  ledcWriteTone(c,f);
  for(int d = 0;d<=255;d = d+10){
      ledcWrite(c,d);
      delay(1000);
  }  
    
  //固定占空比修改频率查看音调的变化  
  ledcWrite(c,d);
  for(int f = 200;f<=2000;f = f+10){
      ledcWriteTone(c,f);
      delay(1000);
  }
}
