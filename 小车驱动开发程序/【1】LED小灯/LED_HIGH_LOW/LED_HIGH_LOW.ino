//设置变量
const int led32 = 32; //32号GPIO脚(绿灯)
const int led33 = 33; //33号GPIO脚(红灯)
//基本设置程序(只运行一次)
void setup() {
  pinMode(led32,OUTPUT); //将GPIO脚lede32设置为输出脚
  pinMode(led33,OUTPUT); //将GPIO脚lede33设置为输出脚

}

//主程序(循环运行)
void loop() {
  digitalWrite(led32,LOW); //将led32设置为低电平  由于小灯是一端接电一端接GPIO角，故此时绿灯是亮的(由于绿灯连的是32脚)  (LOW--0 HIGH--1)
  digitalWrite(led33,LOW); //33号管脚控制的红灯此时也亮
}
