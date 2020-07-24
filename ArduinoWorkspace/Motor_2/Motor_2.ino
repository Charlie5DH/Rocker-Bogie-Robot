const int PWMA_Pin = 3;
const int ain2_Pin = 32; //entradas H-bridge
const int ain1_Pin = 33;
const int bin1_Pin = 34; //entradas H-bridge
const int bin2_Pin = 35;
const int PWMB_Pin = 4;

const int PWMA_2_Pin = 5;
const int ain2_2_Pin = 36; //entradas H-bridge
const int ain1_2_Pin = 37;
const int bin1_2_Pin = 38; //entradas H-bridge
const int bin2_2_Pin = 39;
const int PWMB_2_Pin = 6;

const int PWMA_3_Pin = 7;
const int ain2_3_Pin = 22; //entradas H-bridge
const int ain1_3_Pin = 23;
const int bin1_3_Pin = 24; //entradas H-bridge
const int bin2_3_Pin = 25;
const int PWMB_3_Pin = 8;

void setup() {
  Serial.begin(9600);
  pinMode(bin1_Pin, OUTPUT);
  pinMode(bin2_Pin, OUTPUT);
  pinMode(PWMB_Pin, OUTPUT);
  pinMode(PWMA_Pin, OUTPUT);
  pinMode(ain1_Pin, OUTPUT);
  pinMode(ain2_Pin, OUTPUT);

  pinMode(PWMA_2_Pin, OUTPUT);
  pinMode(ain1_2_Pin, OUTPUT);
  pinMode(ain2_2_Pin, OUTPUT);
  pinMode(PWMB_2_Pin, OUTPUT);
  pinMode(bin1_2_Pin, OUTPUT);
  pinMode(bin2_2_Pin, OUTPUT);

  pinMode(PWMA_3_Pin, OUTPUT);
  pinMode(ain1_3_Pin, OUTPUT);
  pinMode(ain2_3_Pin, OUTPUT);
  pinMode(PWMB_3_Pin, OUTPUT);
  pinMode(bin1_3_Pin, OUTPUT);
  pinMode(bin2_3_Pin, OUTPUT);

  Serial.println("Speed (0-9), (+,-) to ser direction, (s)to stop");
}

void loop() {
  if(Serial.available()){
    char ch = Serial.read();
    if(isDigit(ch)){
      int speed = map(ch,'0','9',0,255);
      analogWrite(PWMB_Pin, speed);
      analogWrite(PWMA_Pin, speed);
      analogWrite(PWMA_2_Pin, speed);
      analogWrite(PWMB_2_Pin, speed);
      analogWrite(PWMA_3_Pin, speed);
      analogWrite(PWMB_2_Pin, speed);
    }
    else if(ch=='+'){
      Serial.println("CW");
      
      digitalWrite(bin1_Pin,LOW);
      digitalWrite(bin2_Pin,HIGH);
      digitalWrite(ain1_Pin,LOW);
      digitalWrite(ain2_Pin,HIGH);

      digitalWrite(ain1_2_Pin,LOW);
      digitalWrite(ain2_2_Pin,HIGH);
      digitalWrite(bin1_2_Pin,LOW);
      digitalWrite(bin2_2_Pin,HIGH);

      digitalWrite(ain1_3_Pin,LOW);
      digitalWrite(ain2_3_Pin,HIGH);
      digitalWrite(bin1_3_Pin,LOW);
      digitalWrite(bin2_3_Pin,HIGH);            
    }
    else if(ch=='-'){
      Serial.println("CCW");
      digitalWrite(bin1_Pin,HIGH);
      digitalWrite(bin2_Pin,LOW);
      digitalWrite(ain1_Pin,HIGH);
      digitalWrite(ain2_Pin,LOW);

      digitalWrite(ain1_2_Pin,HIGH);
      digitalWrite(ain2_2_Pin,LOW);
      digitalWrite(bin1_2_Pin,HIGH);
      digitalWrite(bin2_2_Pin,LOW);

      digitalWrite(ain1_3_Pin,HIGH);
      digitalWrite(ain2_3_Pin,LOW);
      digitalWrite(bin1_3_Pin,HIGH);
      digitalWrite(bin2_3_Pin,LOW);
    }
    else if(ch=='s'){
      Serial.println("Stop");
      digitalWrite(bin1_Pin,LOW);
      digitalWrite(bin2_Pin,LOW);
      
      digitalWrite(ain1_Pin,LOW);
      digitalWrite(ain2_Pin,LOW);

      digitalWrite(ain1_2_Pin,LOW);
      digitalWrite(ain2_2_Pin,LOW);
    }
    else{
      Serial.println("Unexpected character");
      Serial.println(ch);
    }
  }
}
