int led = 13;

void setup() { 
  pinMode(led, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if(Serial.available()){
    char c = Serial.read();
    if(c == 'H'){
      digitalWrite(led, HIGH);
    }
    else if(c == 'L'){
      digitalWrite(led, LOW);
    }
  }
}
