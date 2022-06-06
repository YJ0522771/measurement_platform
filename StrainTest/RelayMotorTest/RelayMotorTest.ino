#define RELAY1 3
#define RELAY2 2
//릴레이 모듈 > LOW : NC에 연결, HIGH : NO에 연결

void setup() {
  Serial.begin(9600);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  delay(5*1000);
  
  digitalWrite(RELAY1, HIGH);    //NC 연결
  digitalWrite(RELAY2, HIGH);

}

void loop() {
  
  delay(5*1000);
  
  digitalWrite(RELAY2, LOW);
  digitalWrite(RELAY1, LOW);   //NO 연결

  delay(5*1000);

  digitalWrite(RELAY1, HIGH);    //NC 연결
  digitalWrite(RELAY2, HIGH);    

}
