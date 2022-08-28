//设置LED与按钮变量，目标是按一下亮再按一下灭
const int led32 = 32;
const int button35 = 35; //按钮一端是0V，另一端是按下那端，按下时接收到0V(低电平)    35号管脚对应的是最左边的按钮KEY1

//设置按钮初始状态
int buttonstate = 1;

//交替的亮灭状态
boolean change = true; //储存亮灭的情况   !change 的值是 false

void setup() {
  Serial.begin(115200); //将程序的波特率与串口设为一致的
  pinMode(led32,OUTPUT);
  pinMode(button35,INPUT);
}

void loop() {
  delay(500); //延时监测，由于按钮灵敏度太高
  
  while(digitalRead(button35) == HIGH){} //不按按钮时，空的死循环，也就是什么都不发生

  if (change == true){  //假设现在是暗的，需要让他亮
  change = !change;
  digitalWrite(led32,LOW);  
    }
    else{
    change = !change;
    digitalWrite(led32,HIGH);}
}
