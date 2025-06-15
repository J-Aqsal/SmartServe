#define PB1 22
#define PB2 24
#define PB3 26
#define PB4 28
#define LED_PIN 2

int currentTable = 0;

void setup() {
  pinMode(PB1, INPUT_PULLUP);
  pinMode(PB2, INPUT_PULLUP);
  pinMode(PB3, INPUT_PULLUP);
  pinMode(PB4, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200); // komunikasi ke ESP
}

void loop() {
  if (digitalRead(PB1) == LOW) {
    currentTable++;
    Serial.println("currentTable:" + String(currentTable));
    delay(100);
  }

  if (digitalRead(PB2) == LOW) {
    Serial.println("mode:delivery");
    delay(100);
  }

  if (digitalRead(PB3) == LOW) {
    Serial.println("mode:return");
    delay(100);
  }

  if (digitalRead(PB4) == LOW) {
    Serial.println("mode:idle");
    delay(100);
  }

  // Terima feedback dari ESP
  if (Serial.available()) {
    String feedback = Serial.readStringUntil('\n');
    feedback.trim();
    if (feedback.startsWith("mode:")) { // mode dari firebase
      String mode = feedback.substring(feedback.indexOf(":") + 1);
      mode.trim();
      if (mode == "on"){

        // delivery

        digitalWrite(LED_PIN, HIGH);
      }else {
        digitalWrite(LED_PIN, LOW);
      }
    }
  }
}
