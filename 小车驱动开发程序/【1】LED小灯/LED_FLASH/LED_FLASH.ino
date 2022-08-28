//目标：使得32号小灯和33号小灯交替闪烁
const int led32 = 32; //32号GPIO脚(绿灯)
const int led33 = 33; //33号GPIO脚(红灯)

void setup() {
  pinMode(led32,OUTPUT);
  pinMode(led33,OUTPUT);

}

void loop() {
  digitalWrite(led32,LOW);
  digitalWrite(led33,HIGH);
  delay(500); //延时500ms
  digitalWrite(led32,HIGH);
  digitalWrite(led33,LOW);
  delay(500);

}
