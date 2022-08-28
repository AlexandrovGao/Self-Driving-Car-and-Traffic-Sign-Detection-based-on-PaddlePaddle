//目标：呼吸灯

const int led32 = 32; //32号GPIO脚(绿灯)
const int led33 = 33; //33号GPIO脚(红灯)

void setup() {
    pinMode(led32,OUTPUT);
    //pinMode(led33,OUTPUT);

}

void loop() {                     //在一个周期内使得高电平和低电平分别占有不同的比率，它们结合后会产生不同的电压，本例1份LOW,9份HIGH，故电压为0.5V
  digitalWrite(led32,LOW);
  digitalWrite(led32,HIGH);
  digitalWrite(led32,HIGH);
  digitalWrite(led32,HIGH);
  digitalWrite(led32,HIGH);
  digitalWrite(led32,HIGH);
  digitalWrite(led32,HIGH);
  digitalWrite(led32,HIGH);
  digitalWrite(led32,HIGH);
  digitalWrite(led32,HIGH);




  
}
