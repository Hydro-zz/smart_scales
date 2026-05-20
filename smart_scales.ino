#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <WiFiClient.h>
#include <ESP8266HTTPUpdateServer.h>
#include "HX711.h"

// --- НАЗНАЧЕНИЕ ПИНОВ ---
#define PIN_BUTTON D3
#define PIN_GREEN  D7
#define PIN_YELLOW D6
#define PIN_RED    D5
#define PIN_DT     D1
#define PIN_SCK    D2

HX711 scale;
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;

// Структура настроек в EEPROM с массивами Wi-Fi и OTA
struct Settings {
  char ssid[32]; 
  char pass[32]; 
  float calibration;    // Коэффициент калибровки
  long raw_platform;    // Сырое значение Raw пустой платформы (0 г)
  float threshold_high; // Порог "Норма" (для светодиодов)
  float threshold_low;  // Порог "Мало" (для светодиодов)
  float tare_weight;    // Вес пустого соляного бака
  float max_salt;       // Максимальный вес соли в баке (г) для расчета 100%
  char otaUser[32];     // Динамический логин для страницы /update
  char otaPass[32];     // Динамический пароль для страницы /update
  uint32_t magic;       // Метка валидности памяти
} user_data;

// Глобальные переменные данных
long global_raw = 0;       
float global_gross = 0;    
float global_net = 0;      
int currentPercent = 0;

bool sensorWasError = false;
unsigned long lastReadyTime = 0; 
unsigned long buttonTimer = 0;
bool buttonActive = false;
unsigned long lastSerialPrint = 0;
unsigned long lastScaleUpdate = 0;

bool isConfigMode = false;
bool needRestart = false;

int wifiSignalPct = 0;            
long wifiSignalDbm = 0;           

// Переменная для хранения промежуточного шага калибровки
long raw_step1 = 0;
float step1_weight = 0;

// Безопасное ручное чтение датчика с отключением прерываний
long readSafeRaw() {
  if (digitalRead(PIN_DT) == HIGH) {
    return 0; 
  }
  
  long count = 0;
  noInterrupts(); 
  for (int i = 0; i < 24; i++) {
    digitalWrite(PIN_SCK, HIGH);
    delayMicroseconds(1);
    count = count << 1;
    digitalWrite(PIN_SCK, LOW);
    delayMicroseconds(1);
    if (digitalRead(PIN_DT)) count++;
  }
  digitalWrite(PIN_SCK, HIGH); 
  delayMicroseconds(1);
  digitalWrite(PIN_SCK, LOW);
  delayMicroseconds(1);
  interrupts(); 
  
  long signed_data = count;
  if (count & 0x800000) signed_data |= 0xFF000000;
  return signed_data;
}

bool isScaleReady() {
  return (digitalRead(PIN_DT) == LOW);
}

int calculatePercent(float current_net, float max_salt_weight) {
  if (max_salt_weight <= 0) return 0;
  float pct = (current_net / max_salt_weight) * 100.0;
  int result = (int)pct;
  if (result < 0) result = 0;
  if (result > 100) result = 100;
  return result;
}

int rssiToPercentage(long rssi) {
  if (rssi >= -50) return 100;
  if (rssi <= -100) return 0;
  return 2 * (rssi + 100);
}

String getWifiQualityText(int pct) {
  if (pct > 80) return "<span style='color:green; font-weight:bold;'>Отличный</span>";
  if (pct > 55) return "<span style='color:#28a745; font-weight:bold;'>Хороший</span>";
  if (pct > 30) return "<span style='color:orange; font-weight:bold;'>Слабый</span>";
  return "<span style='color:red; font-weight:bold;'>Плохой (Помехи!)</span>";
}

void startConfigMode();
void startNormalMode();
void connectWiFiAnimation();
void showSensorError();
void updateLEDs();
void handleButton();
void setupWebServerHandlers();

String getCalibrationHtml() {
  String block = "";
  block += "Текущий живой Raw: <b id='cal_raw_val' style='color:blue; font-size:1.1em;'>" + String(global_raw) + "</b><br><br>";
  if (raw_step1 == 0) {
    block += "<form action='/cal_step1' method='POST'>";
    block += "<b>ШАГ 1:</b> Какой ПОЛНЫЙ физический вес сейчас давит на весы (включая фанеру, бак и соль, в граммах)?<br>";
    block += "<input name='current_total_w' placeholder='Пример: 99300' style='width:100%; padding:8px; margin:5px 0 10px; box-sizing:border-box;'>";
    block += "<input type='submit' value='Зафиксировать начальную точку' style='padding:10px; background:#2196F3; color:white; border:0; cursor:pointer; width:100%; border-radius:4px; font-weight:bold;'>";
    block += "</form>";
  } else {
    block += "<div style='background:#e2f0d9; padding:8px; margin-bottom:10px; border-radius:4px; font-size:14px; border:1px solid #ccd7c5;'>Шаг 1 пройден. Начальный Raw (" + String(raw_step1) + ") сохранен.</div>";
    block += "<form action='/cal_step2' method='POST'>";
    block += "<b>ШАГ 2:</b> Положите контрольный грузд сверху. Какой чистый вес ГРУЗА (в граммах)?<br>";
    block += "<input name='cal_grunt_w' placeholder='Пример: 10000' style='width:100%; padding:8px; margin:5px 0 10px; box-sizing:border-box;'>";
    block += "<input type='submit' value='Кладем груз и калибруем' style='padding:10px; background:#4CAF50; color:white; border:0; cursor:pointer; width:100%; border-radius:4px; font-weight:bold;'>";
    block += "</form>";
    block += "<br><a href='/cal_reset' style='color:red; text-decoration:none; font-size:14px;'>❌ Сбросить и начать заново</a>";
  }
  return block;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n--- Scaler Boot V2 (Safe OTA) ---");

  EEPROM.begin(512);
  EEPROM.get(0, user_data);

  // Ваша текущая калибровка БЕЗ учета константы фанеры (все калибруется с нуля)
  if (user_data.magic != 0xABCD1255) { 
    Serial.println("Чистый EEPROM! Записываем стартовые дефолты...");
    memset(user_data.ssid, 0, 32);
    memset(user_data.pass, 0, 32);
    memset(user_data.otaUser, 0, 32); // Строго пусто до ручной настройки в AP!
    memset(user_data.otaPass, 0, 32); 
    user_data.calibration = 25.3788; 
    user_data.raw_platform = -251644; 
    user_data.threshold_high = 15000.0; 
    user_data.threshold_low = 3000.0;   
    user_data.tare_weight = 97000; 
    user_data.max_salt = 45000.0; 
    user_data.magic = 0xABCD1255;
    EEPROM.put(0, user_data);
    EEPROM.commit();
  } else {
    Serial.println("Калибровка успешно загружена из EEPROM!");
  }

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_RED, OUTPUT);
  
  pinMode(PIN_DT, INPUT);
  pinMode(PIN_SCK, OUTPUT);
  digitalWrite(PIN_SCK, LOW);

  setupWebServerHandlers();
  
  Serial.println("Ожидание 5 секунд для входа в настройки...");
  unsigned long startWait = millis();
  bool configMode = false;
  while (millis() - startWait < 5000) {
    digitalWrite(PIN_GREEN, (millis() / 200) % 2); 
    if (digitalRead(PIN_BUTTON) == LOW) { configMode = true; break; }
    delay(10);
    yield();
  }
  digitalWrite(PIN_GREEN, LOW);

  if (configMode) {
    Serial.println("Запуск в РЕЖИМЕ НАСТРОЕК (Точка доступа Scales_Setup)...");
    startConfigMode();
  } else {
    Serial.println("Запуск в РЕЖИМЕ РАБОТЫ (Клиент вашей WiFi сети)...");
    startNormalMode();
  }
  
  server.begin();
  Serial.println("HTTP Веб-сервер запущен!");
}

void loop() {
  server.handleClient();
  if (needRestart) { delay(500); ESP.restart(); }

  if (isScaleReady()) {
    lastReadyTime = millis(); 
    if (millis() - lastScaleUpdate > 300) {
      lastScaleUpdate = millis();
      long buffer[9];
      int validSamples = 0;
      
      for (int i = 0; i < 9; i++) {
        long val = readSafeRaw();
        if (val != 0) { buffer[validSamples] = val; validSamples++; }
        delay(15);
      }
      
      if (validSamples >= 5) {
        for (int i = 0; i < validSamples - 1; i++) {
          for (int j = i + 1; j < validSamples; j++) {
            if (buffer[i] > buffer[j]) { long temp = buffer[i]; buffer[i] = buffer[j]; buffer[j] = temp; }
          }
        }
        global_raw = buffer[validSamples / 2];
        sensorWasError = false;
      }
      
      // ИСПРАВЛЕННАЯ МАТЕМАТИКА БЕЗ СМЕЩЕНИЯ НА ВЕС ФАНЕРЫ
      global_gross = (float)(global_raw - user_data.raw_platform) / user_data.calibration;
      global_net = global_gross - user_data.tare_weight;
      if (global_net < 0) global_net = 0; 
      currentPercent = calculatePercent(global_net, user_data.max_salt);
    }
  }
  
  if (millis() - lastReadyTime > 2500) {
    sensorWasError = true;
    showSensorError(); 
    return;            
  } else {
    if (sensorWasError) {
      digitalWrite(PIN_GREEN, LOW); digitalWrite(PIN_YELLOW, LOW); digitalWrite(PIN_RED, LOW);
      sensorWasError = false;
    }
  }

  if (!isConfigMode && WiFi.status() == WL_CONNECTED) {
    wifiSignalDbm = WiFi.RSSI(); wifiSignalPct = rssiToPercentage(wifiSignalDbm);
  }

  handleButton(); 
  updateLEDs();   
  yield();
}

void startConfigMode() {
  isConfigMode = true;
  WiFi.softAPdisconnect(true); WiFi.disconnect(); delay(100);
  WiFi.mode(WIFI_AP); 
  WiFi.softAP("Scales_Setup");
}

void startNormalMode() {
  isConfigMode = false;
  WiFi.mode(WIFI_STA); WiFi.begin(user_data.ssid, user_data.pass);
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 30) { connectWiFiAnimation(); delay(500); attempt++; }
}

void setupWebServerHandlers() {
  
  server.on("/", []() {
    if (isConfigMode) {
      String safeSSID = String(user_data.ssid); String safePass = String(user_data.pass);
      if (user_data.magic != 0xABCD1255) { safeSSID = ""; safePass = ""; }

      String h = "<html><head><meta charset='UTF-8'><title>Scales Config</title></head><body style='font-family:sans-serif; padding:20px; max-width:500px; margin:auto;'>";
      h += "<h2>Абсолютные настройки весов (Режим AP)</h2>";
      
      h += "<div style='background:#f0f0f0; padding:15px; margin-bottom:20px; border-radius:5px; border:1px solid #ccc;'>";
      h += "<h3>1. Экспресс-калибровка под баком:</h3>";
      h += getCalibrationHtml(); 
      h += "</div>";

      h += "<form action='/save' method='POST'>";
      h += "<h3>2. Настройки Wi-Fi и Порогов</h3>";
      h += "SSID сети:<br><input type='text' name='ssid' value='" + safeSSID + "' style='width:100%; padding:8px; margin-bottom:15px;'>";
      h += "WiFi Pass:<br><input type='password' name='pass' value='" + safePass + "' style='width:100%; padding:8px; margin-bottom:15px;'>";
      
      h += "<h3>3. Безопасность обновления (/update)</h3>";
      h += "Логин администратора:<br><input type='text' name='ota_u' value='" + String(user_data.otaUser) + "' style='width:100%; padding:8px; margin-bottom:10px;'>";
      h += "Пароль администратора:<br><input type='password' name='ota_p' value='" + String(user_data.otaPass) + "' style='width:100%; padding:8px; margin-bottom:15px;'>";

      h += "<h3>4. Веса и пороги</h3>";
      h += "Макс. вес соли в баке (100%), г:<br><input type='text' name='max_salt' value='" + String(user_data.max_salt, 0) + "' style='width:100%; padding:8px; margin-bottom:15px;'>";
      h += "Вес бака (тара, г):<br><input type='text' name='tare' value='" + String(user_data.tare_weight, 0) + "' style='width:100%; padding:8px; margin-bottom:15px;'>";
      h += "Порог НОРМА (г):<br><input type='text' name='high' value='" + String(user_data.threshold_high, 0) + "' style='width:100%; padding:8px; margin-bottom:15px;'>";
      h += "Порог МАЛО (г):<br><input type='text' name='low' value='" + String(user_data.threshold_low, 0) + "' style='width:100%; padding:8px; margin-bottom:20px;'>";
      h += "<input type='submit' value='Сохранить все настройки' style='background:green; color:white; padding:12px; width:100%; border-radius:4px; cursor:pointer; font-size:1.1em; border:0;'>";
      h += "</form></body></html>";
      server.send(200, "text/html", h);
    } 
    else {
      String status_text = ""; String status_color = "gray";
      if (global_net > user_data.threshold_high) { status_text = "НОРМА"; status_color = "green"; }
      else if (global_net <= user_data.threshold_high && global_net > user_data.threshold_low) { status_text = "СРЕДНИЙ УРОВЕНЬ"; status_color = "orange"; }
      else { status_text = "МАЛО СОЛИ!"; status_color = "red"; }

      String html = "<html><head><meta charset='UTF-8'></head><body style='font-family:Arial; text-align:center; padding-top:20px; max-width:500px; margin:auto;'>";
      html += "<h2>Мониторинг соли в баке</h2><br>";
      html += "<div style='font-size:48px; margin:20px 0;'>Чистый вес: <b id='salt_w'>" + String(global_net, 0) + " г</b></div>";
      
      html += "<div style='width:80%; background:#ddd; margin:auto; border-radius:15px; overflow:hidden; border:1px solid #999;'>";
      html += "<div id='pbar' style='width:" + String(currentPercent) + "%; background:#3498db; height:30px; line-height:30px; color:white; font-weight:bold; transition: width 0.5s;'>" + String(currentPercent) + "%</div></div><br>";
      
      html += "<p>Статус: <span id='status_text' style='color:" + status_color + "; font-weight:bold; font-size:1.3em;'> " + status_text + "</span></p>";
      html += "<p style='color:#555;'><small>📶 Wi-Fi: " + getWifiQualityText(wifiSignalPct) + " (<span id='w_pct'>" + String(wifiSignalPct) + "</span>% / <span id='w_dbm'>" + String(wifiSignalDbm) + "</span> dBm)</small></p>";
      
      html += "<div style='background:#fdfdfd; padding:15px; border-radius:8px; text-align:left; margin-top:25px; border:1px solid #ddd;'>";
      html += "<details " + String(raw_step1 > 0 ? "open" : "") + "><summary style='cursor:pointer; color:#007bff; font-weight:bold; font-size:1.1em;'>🔧 Сервисная калибровка под баком</summary><br>";
      html += getCalibrationHtml(); 
      html += "<br><form action='/save' method='POST'>";
      html += "Макс. вес соли в баке (100%), г:<br><input type='text' name='max_salt' value='" + String(user_data.max_salt, 0) + "' style='width:100%; padding:6px; margin:5px 0 10px;'>";
      html += "Вес бака (тара, г):<br><input type='text' name='tare' value='" + String(user_data.tare_weight, 0) + "' style='width:100%; padding:6px; margin:5px 0 10px;'>";
      html += "Порог НОРМА (г):<br><input type='text' name='high' value='" + String(user_data.threshold_high, 0) + "' style='width:100%; padding:6px; margin:5px 0 10px;'>";
      html += "Порог МАЛО (г):<br><input type='text' name='low' value='" + String(user_data.threshold_low, 0) + "' style='width:100%; padding:6px; margin:5px 0 15px;'>";
      html += "<input type='submit' value='Применить изменения параметров' style='background:#6c757d; color:white; padding:8px; width:100%; border:0; border-radius:4px; font-weight:bold; cursor:pointer;'>";
      html += "</form></details></div>";
      
      html += "<script>";
      html += "setInterval(function() {";
      html += "  fetch('/api/status').then(response => response.json()).then(data => {";
      html += "    document.getElementById('salt_w').innerText = data.net + ' г';";
      html += "    if(document.getElementById('cal_raw_val')) document.getElementById('cal_raw_val').innerText = data.raw;";
      html += "    document.getElementById('w_pct').innerText = data.wifi_pct;";
      html += "    document.getElementById('w_dbm').innerText = data.wifi_rssi;";
      html += "    var pbar = document.getElementById('pbar'); pbar.style.width = data.percent + '%'; pbar.innerText = data.percent + '%';";
      html += "    var st = document.getElementById('status_text');";
      html += "    if (data.net > " + String(user_data.threshold_high) + ") { st.innerText = 'НОРМА'; st.style.color = 'green'; }";
      html += "    else if (data.net <= " + String(user_data.threshold_high) + " && data.net > " + String(user_data.threshold_low) + ") { st.innerText = 'СРЕДНИЙ УРОВЕНЬ'; st.style.color = 'orange'; }";
      html += "    else { st.innerText = 'МАЛО СОЛИ!'; st.style.color = 'red'; }";
      html += "  });";
      html += "}, 2000);"; 
      html += "</script>";

      html += "</body></html>";
      server.send(200, "text/html", html);
    }
  });

  server.on("/cal_step1", []() {
    raw_step1 = global_raw; step1_weight = server.arg("current_total_w").toFloat();
    server.sendHeader("Location", "/"); server.send(303);
  });

  server.on("/cal_step2", []() {
    float pure_grunt_weight = server.arg("cal_grunt_w").toFloat(); long raw_step2 = global_raw;
    if (pure_grunt_weight > 0 && (raw_step2 - raw_step1) != 0) {
      user_data.calibration = (float)(raw_step2 - raw_step1) / pure_grunt_weight;
      // ВЫЧИСЛЕНИЕ АБСОЛЮТНОГО НУЛЯ (0г) БЕЗ СДВИГА НА ВЕС ФАНЕРЫ
      user_data.raw_platform = raw_step1 - (step1_weight * user_data.calibration);
      user_data.magic = 0xABCD1255; EEPROM.put(0, user_data); EEPROM.commit(); raw_step1 = 0;
      server.send(200, "text/html", "<h3>Калибровка завершена успешно! Пересчитано. Возврат...</h3><script>setTimeout(function(){ window.location.href = '/'; }, 3000);</script>");
      return;
    }
    server.sendHeader("Location", "/"); server.send(303);
  });

  server.on("/cal_reset", []() { raw_step1 = 0; server.sendHeader("Location", "/"); server.send(303); });

  server.on("/api/status", []() {
    String json = "{";
    json += "\"raw\":" + String(global_raw) + ",";
    json += "\"gross\":" + String(global_gross, 1) + ",";
    json += "\"net\":" + String(global_net, 1) + ",";
    json += "\"percent\":" + String(currentPercent) + ",";
    json += "\"threshold_low\":" + String(user_data.threshold_low, 1) + ",";
    json += "\"threshold_high\":" + String(user_data.threshold_high, 1) + ",";
    json += "\"tare_weight\":" + String(user_data.tare_weight, 1) + ",";
    json += "\"max_salt\":" + String(user_data.max_salt, 1) + ",";
    json += "\"calibration\":" + String(user_data.calibration, 4) + ",";
    json += "\"raw_platform\":" + String(user_data.raw_platform) + ",";
    json += "\"wifi_rssi\":" + String(wifiSignalDbm) + ",";
    json += "\"wifi_pct\":" + String(wifiSignalPct) + "";
    json += "}";
    server.send(200, "application/json", json);
  });

  // 4. БЕЗОПАСНОЕ СОХРАНЕНИЕ НАСТРОЕК СЕТИ И ПОРОГОВ
  server.on("/save", []() {
    String req_ssid = server.arg("ssid"); String req_pass = server.arg("pass");
    float new_low = server.arg("low").toFloat(); float new_high = server.arg("high").toFloat();
    float new_tare = server.arg("tare").toFloat(); float new_max = server.arg("max_salt").toFloat();

    bool wifiChanged = false;
    bool securityChanged = false;
    
    if (req_ssid.length() > 0 && req_ssid != "null" && req_ssid != "") {
      wifiChanged = (req_ssid != String(user_data.ssid)) || (req_pass != String(user_data.pass));
      req_ssid.toCharArray(user_data.ssid, 32); req_pass.toCharArray(user_data.pass, 32);
    }
    
    if (isConfigMode) {
      String req_ota_u = server.arg("ota_u"); String req_ota_p = server.arg("ota_p");
      if (req_ota_u.length() > 0 && req_ota_u != "None" && req_ota_u != "") {
        if (req_ota_u != String(user_data.otaUser)) { req_ota_u.toCharArray(user_data.otaUser, 32); securityChanged = true; }
      }
      if (req_ota_p.length() > 0 && req_ota_p != "None" && req_ota_p != "") {
        if (req_ota_p != String(user_data.otaPass)) { req_ota_p.toCharArray(user_data.otaPass, 32); securityChanged = true; }
      }
    }

    user_data.threshold_high = new_high; user_data.threshold_low = new_low; 
    user_data.tare_weight = new_tare; user_data.max_salt = new_max;
    user_data.magic = 0xABCD1255; 
    EEPROM.put(0, user_data); EEPROM.commit();

    if (isConfigMode || wifiChanged || securityChanged) {
      server.send(200, "text/html", "<html><meta charset='UTF-8'><body><h3>Данные сохранены. Перезагрузка системы...</h3></body></html>");
      needRestart = true; 
    } else {
      String html = "<html><meta charset='UTF-8'><body><h3>Настройки весов успешно применены на лету!</h3>";
      html += "<script>setTimeout(function(){ window.location.href = '/'; }, 1500);</script></body></html>";
      server.send(200, "text/html", html);
    }
  });

  // 5. ИСПРАВЛЕННЫЙ ПЕРЕХВАТ СТРАНИЦЫ ОБНОВЛЕНИЯ (Блокирует пустые поля в памяти)
  server.on("/update", HTTP_GET, []() {
    if (strlen(user_data.otaUser) == 0 || strlen(user_data.otaPass) == 0) {
      server.send(403, "text/html", "<html><meta charset='UTF-8'><body><h2>Ошибка 403: Доступ заблокирован. Установите логин и пароль в режиме конфигурации устройства!</h2></body></html>");
      return;
    }
    if (!server.authenticate(user_data.otaUser, user_data.otaPass)) {
      return server.requestAuthentication();
    }
    server.send(200, "text/html", "<html><head><meta charset='UTF-8'></head><body><h2>Обновление прошивки Весов</h2><form method='POST' action='/update' enctype='multipart/form-data'>Выбор бинарного файла (.bin):<br><br><input type='file' name='firmware'><br><br><input type='submit' value='Запустить обновление' style='padding:8px 20px; background:#007bff; color:white; border:0; border-radius:4px; cursor:pointer;'></form></body></html>");
  });

  httpUpdater.setup(&server, "/update", user_data.otaUser, user_data.otaPass);
}

// Пересчет светодиодов (остается без изменений)
void connectWiFiAnimation() {
  static int attemptStep = 0; attemptStep++; if (attemptStep > 6) attemptStep = 1;
  digitalWrite(PIN_GREEN,  (attemptStep == 1 || attemptStep == 2 || attemptStep == 3));
  digitalWrite(PIN_YELLOW, (attemptStep == 2 || attemptStep == 3 || attemptStep == 4));
  digitalWrite(PIN_RED,    (attemptStep == 3 || attemptStep == 4 || attemptStep == 5));
}

void showSensorError() {
  static unsigned long lastF = 0;
  if (millis() - lastF > 300) { bool s = !digitalRead(PIN_RED); digitalWrite(PIN_GREEN, s); digitalWrite(PIN_YELLOW, s); digitalWrite(PIN_RED, s); lastF = millis(); }
}

void updateLEDs() {
  static unsigned long lastU = 0; if (millis() - lastU < 1000) return; lastU = millis();
  float w = global_net;
  digitalWrite(PIN_GREEN,  (w > user_data.threshold_high));
  digitalWrite(PIN_YELLOW, (w <= user_data.threshold_high && w > user_data.threshold_low));
  digitalWrite(PIN_RED,    (w <= user_data.threshold_low));
}

void handleButton() {
  if (digitalRead(PIN_BUTTON) == LOW) {
    if (!buttonActive) { buttonActive = true; buttonTimer = millis(); }
    if (buttonActive && (millis() - buttonTimer > 5000)) {
      // ИСПРАВЛЕННО: Ручное тарирование бака кнопкой теперь также учитывает абсолютный ноль платформы
      user_data.tare_weight = global_gross; 
      if (user_data.tare_weight < 0) user_data.tare_weight = 0;
      EEPROM.put(0, user_data); EEPROM.commit(); buttonActive = false;
      digitalWrite(PIN_RED, HIGH); delay(500); digitalWrite(PIN_RED, LOW);
    }
  } else { buttonActive = false; }
}
