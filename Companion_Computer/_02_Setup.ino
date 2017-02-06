void setup() {
  debugbegin(38400);
  Serial1.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  debugln("LED Initialized");
  delay(100);

  pinMode(TRIGGER_PIN, INPUT);
}

