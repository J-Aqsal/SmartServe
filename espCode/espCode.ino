#include <ESP8266WiFi.h>
#include <Firebase.h>

// Ganti dengan SSID WiFi kamu
#define WIFI_SSID "Hall Of Mechatronic Project"
#define WIFI_PASSWORD "mechproject2805"

// Ganti dengan URL dan Secret dari Firebase kamu
#define FIREBASE_HOST "https://smartserve-kdt2-k2-default-rtdb.asia-southeast1.firebasedatabase.app/"  // jangan lupa "/"
#define FIREBASE_AUTH "yAu9r1jKxln5YTeXm9xd7gv7caPZ9CexwCe5TiAT"  // bisa API key (jika pakai Web API) atau legacy key

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

  // Logika respon balik
  int targetTable = fb.getInt("SmartServe/targetTable");
  int currentTable = fb.getInt("SmartServe/currentTable");
  String currentmode = fb.getString("SmartServe/mode");

  if (currentmode == "process") {
    Serial.println("mode:on"); // kirim ke Mega untuk nyalakan LED
  }

  delay(2000); // waktu antar polling
}