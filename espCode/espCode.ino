#include <ESP8266WiFi.h>
#include <Firebase.h>

// Ganti dengan SSID WiFi kamu
#define WIFI_SSID "Hall Of Mechatronic Project"
#define WIFI_PASSWORD "mechproject2805"

// Ganti dengan URL dan Secret dari Firebase kamu
#define FIREBASE_HOST "https://smartserve-96848-default-rtdb.firebaseio.com/"  // jangan lupa "/"
#define FIREBASE_AUTH "mtCJ2E06VMT8QeYh2QMgZ4npt60nfaWBju1dtZrW"  // bisa API key (jika pakai Web API) atau legacy key

Firebase fb(FIREBASE_HOST, FIREBASE_AUTH);


void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // Set LED sebagai output
  digitalWrite(LED_BUILTIN, LOW);  // LED menyala (aktif LOW)
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Serial.print("Menghubungkan ke WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    // Serial.print(".");
  }
  digitalWrite(LED_BUILTIN, HIGH);
  // Serial.println("\nWiFi terhubung!");

}

unsigned long lastFirebaseCheck = 0;
const long firebaseCheckInterval = 500; // Cek Firebase setiap 500 ms

void loop() {
  // Ambil perintah dari Mega
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("currentTable:")) {
      int val = input.substring(input.indexOf(":") + 1).toInt();
      fb.setInt("SmartServe/currentTable", val);
    } else if (input.startsWith("mode:")) {
      String setMode = input.substring(input.indexOf(":") + 1);
      setMode.trim();
      fb.setString("SmartServe/mode", setMode);
    }
  }

  // Logika respon balik (non-blocking)
  if (millis() - lastFirebaseCheck >= firebaseCheckInterval) {
    lastFirebaseCheck = millis();
    int targetTable = fb.getInt("SmartServe/targetTable");
    String currentMode = fb.getString("SmartServe/mode"); // Hanya ambil mode
  
    currentMode.replace("\"","");
    currentMode.replace("\\","");
    Serial.print("mode:");
    Serial.println(currentMode);
    // Kirim feedback ke Arduino hanya jika mode "process"
    if (currentMode == "process") {
      Serial.print("targetTable:"); // Kirim juga targetTable jika diperlukan
      Serial.println(targetTable);
    }
  }
}