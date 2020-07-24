const int PWMA_Pin = 13;
const int ain2_Pin = 12; //entradas H-bridge
const int ain1_Pin = 11;
const int bin1_Pin = 10; //entradas H-bridge
const int bin2_Pin = 9;
const int PWMB_Pin = 8;
const int PWMA_2_Pin = 7;
const int ain2_2_Pin = 6; //entradas H-bridge
const int ain1_2_Pin = 5;



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
    }
    else if(ch=='+'){
      Serial.println("CW");
      
      digitalWrite(bin1_Pin,LOW);
      digitalWrite(bin2_Pin,HIGH);
      
      digitalWrite(ain1_Pin,LOW);
      digitalWrite(ain2_Pin,HIGH);

      digitalWrite(ain1_2_Pin,LOW);
      digitalWrite(ain2_2_Pin,HIGH);
    }
    else if(ch=='-'){
      Serial.println("CCW");
      digitalWrite(bin1_Pin,HIGH);
      digitalWrite(bin2_Pin,LOW);
      
      digitalWrite(ain1_Pin,HIGH);
      digitalWrite(ain2_Pin,LOW);

      digitalWrite(ain1_2_Pin,HIGH);
      digitalWrite(ain2_2_Pin,LOW);
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
