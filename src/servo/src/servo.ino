byte clockwise[4] = {0x03, 0x06, 0x0c, 0x09};
byte anti_clockwise[4] = {0x09, 0x0c, 0x06, 0x03};

int turn_delay = 2;

void turnClockwise() {
  for(int i = 0; i < 512; i++) {
    for(int j = 0; j < 4; j++) {
      PORTB = clockwise[j];
      delay(turn_delay);
    }
  }
}

void setup() {
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  
  PORTB = 0x00;
}

void loop() {
  turnClockwise();
  delay(1000);
}