#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>  // <-- Biblioteca Wi-Fi

// Wi-Fi
const char* ssid = "Pedro2 2.4G";
const char* password = "23022001";
WiFiServer tcpServer(12345);  // Porta do servidor TCP
WiFiClient client;

TaskHandle_t TaskRainbow;

// Pinos dos motores
const int motorA1 = 18;
const int motorA2 = 19;
const int motorB1 = 21;
const int motorB2 = 4;
const int motorC1 = 25;
const int motorC2 = 13;
const int motorD1 = 26;
const int motorD2 = 33;
String ultimoLado = "frente";

// Sensores
const int sensorEsquerdo = 34;
const int sensorDireito = 35;

// Velocidades
const int vSpeedSegueLinha = 46;
const int vSpeedSegueLinhavira = 100;
const int vSpeed = 100;
const int boostedSpeed = 190;

// NeoPixel
#define PIN 5
#define NUMPIXELS 16
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
int modoAnimacao = 0;
const int totalAnimacoes = 3;
int rainbowOffset = 0;
bool btnYAnterior = false;
String comando = "";

// Controle Xbox
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
bool modoSegueLinha = false;
bool btnBAnterior = false;

// Estado não bloqueante
bool aguardandoParado = false;
unsigned long tempoInicioParada = 0;
const unsigned long tempoEspera = 3000;

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi conectado!");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());

  tcpServer.begin();
  Serial.println("Servidor TCP aguardando conexões...");

  xboxController.begin();
  pixels.begin();
  pixels.show();
  pixels.setBrightness(150);

  pinMode(motorA1, OUTPUT); pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT); pinMode(motorB2, OUTPUT);
  pinMode(motorC1, OUTPUT); pinMode(motorC2, OUTPUT);
  pinMode(motorD1, OUTPUT); pinMode(motorD2, OUTPUT);

  pinMode(sensorEsquerdo, INPUT);
  pinMode(sensorDireito, INPUT);

  xTaskCreatePinnedToCore(
    rainbowCycleTask, "RainbowTask", 2000, NULL, 1, &TaskRainbow, 0
  );
}

void loop() {
  if (!client || !client.connected()) {
    client = tcpServer.available();
  }

  if (client && client.connected() && client.available()) {
    comando = client.readStringUntil('\n');
    comando.trim();
    Serial.print("🔁 Comando recebido via Wi-Fi: ");
    Serial.println(comando);
  }

  xboxController.onLoop();

  if (xboxController.isConnected() && !xboxController.isWaitingForFirstNotification()) {
    uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
    bool btnB = xboxController.xboxNotif.btnB;
    if (btnB && !btnBAnterior) {
      modoSegueLinha = !modoSegueLinha;
      Serial.println(modoSegueLinha ? "Modo Segue Linha ATIVADO" : "Modo Segue Linha DESATIVADO");
    }
    btnBAnterior = btnB;

    bool btnY = xboxController.xboxNotif.btnY;
    if (btnY && !btnYAnterior) {
      modoAnimacao = (modoAnimacao + 1) % totalAnimacoes;
      Serial.print("Mudou para animacao: ");
      Serial.println(modoAnimacao);
    }
    btnYAnterior = btnY;

    if (modoSegueLinha) {
      int estadoEsquerdo = digitalRead(sensorEsquerdo);
      int estadoDireito = digitalRead(sensorDireito);

      if (aguardandoParado) {
        if (millis() - tempoInicioParada >= tempoEspera) {
          if (comando == "Vire à esquerda") {
            esquerda(140);
            delay(800);
          } else if (comando == "Vire à direita") {
            direita(140);
            delay(200);
          } else if (comando == "Siga em frente") {
            frente(1.0, vSpeed);
            delay(200);
          }
          para();
          tras(1.0, vSpeed);
          delay(50);
          aguardandoParado = false;
        } else {
          para();  // mantém parado durante a espera
        }
      } else if (estadoEsquerdo == HIGH && estadoDireito == HIGH) {
        aguardandoParado = true;
        tempoInicioParada = millis();
        para();
      } else if (estadoEsquerdo == LOW && estadoDireito == LOW) {
        frente(1.0, vSpeedSegueLinha);
      } else if (estadoEsquerdo == LOW && estadoDireito == HIGH) {
        esquerdaTras(vSpeedSegueLinhavira);
      } else if (estadoEsquerdo == HIGH && estadoDireito == LOW) {
        direitaTras(vSpeedSegueLinhavira);
      } else {
        para();
      }

    } else {
      float joyLHori = (float)xboxController.xboxNotif.joyLHori / joystickMax;
      float joyLVert = (float)xboxController.xboxNotif.joyLVert / joystickMax;
      bool btnA = xboxController.xboxNotif.btnA;

      float absJoyLHori = abs(joyLHori - 0.5);
      float absJoyLVert = abs(joyLVert - 0.5);
      int currentSpeed = btnA ? boostedSpeed : vSpeed;

      if (absJoyLVert > absJoyLHori) {
        if (joyLVert < 0.46) frente((0.5 - joyLVert) * 2, currentSpeed);
        else if (joyLVert > 0.54) tras((joyLVert - 0.5) * 2, currentSpeed);
        else para();
      } else {
        if (joyLHori > 0.55) direita(currentSpeed);
        else if (joyLHori < 0.45) esquerda(currentSpeed);
        else para();
      }
    }
  }
}

// === MOTORES ===
void frente(float velocidade, int speed) {
  int motorSpeed = (int)(velocidade * speed);
  analogWrite(motorA1, motorSpeed); analogWrite(motorA2, 0);
  analogWrite(motorB1, motorSpeed); analogWrite(motorB2, 0);
  analogWrite(motorC1, motorSpeed); analogWrite(motorC2, 0);
  analogWrite(motorD1, motorSpeed); analogWrite(motorD2, 0);
}
void tras(float velocidade, int speed) {
  int motorSpeed = (int)(velocidade * speed);
  analogWrite(motorA1, 0); analogWrite(motorA2, motorSpeed);
  analogWrite(motorB1, 0); analogWrite(motorB2, motorSpeed);
  analogWrite(motorC1, 0); analogWrite(motorC2, motorSpeed);
  analogWrite(motorD1, 0); analogWrite(motorD2, motorSpeed);
}
void esquerdaTras(int speed) {
  analogWrite(motorA1, speed); analogWrite(motorA2, 0);
  analogWrite(motorB1, 0); analogWrite(motorB2, speed);
  analogWrite(motorC1, 0); analogWrite(motorC2, 0);
  analogWrite(motorD1, speed); analogWrite(motorD2, speed);
}
void esquerda(int speed) {
  analogWrite(motorA1, speed); analogWrite(motorA2, 0);
  analogWrite(motorB1, 0); analogWrite(motorB2, 0);
  analogWrite(motorC1, 0); analogWrite(motorC2, 0);
  analogWrite(motorD1, speed); analogWrite(motorD2, 0);
}
void direitaTras(int speed) {
  analogWrite(motorA1, 0); analogWrite(motorA2, speed);
  analogWrite(motorB1, speed); analogWrite(motorB2, 0);
  analogWrite(motorC1, speed); analogWrite(motorC2, speed);
  analogWrite(motorD1, 0); analogWrite(motorD2, 0);
}
void direita(int speed) {
  analogWrite(motorA1, 0); analogWrite(motorA2, 0);
  analogWrite(motorB1, speed); analogWrite(motorB2, 0);
  analogWrite(motorC1, speed); analogWrite(motorC2, 0);
  analogWrite(motorD1, 0); analogWrite(motorD2, 0);
}
void para() {
  analogWrite(motorA1, 0); analogWrite(motorA2, 0);
  analogWrite(motorB1, 0); analogWrite(motorB2, 0);
  analogWrite(motorC1, 0); analogWrite(motorC2, 0);
  analogWrite(motorD1, 0); analogWrite(motorD2, 0);
}

// === LEDS ===
void rainbowCycleTask(void *pvParameters) {
  int j = 0;
  while (true) {
    switch (modoAnimacao) {
      case 0:
        for (int i = 0; i < pixels.numPixels(); i++) {
          pixels.setPixelColor(i, Wheel(((i * 256 / pixels.numPixels()) + j) & 255));
        }
        break;
      case 1:
        for (int i = 0; i < pixels.numPixels(); i++) {
          pixels.setPixelColor(i, i % 2 == 0 ? pixels.Color(255, 0, 0) : pixels.Color(0, 255, 0));
        }
        break;
      case 2:
        for (int i = 0; i < pixels.numPixels(); i++) {
          pixels.setPixelColor(i, ((i + j / 10) % pixels.numPixels() == 0) ? pixels.Color(0, 0, 255) : pixels.Color(0, 0, 0));
        }
        break;
    }
    pixels.show();
    j = (j + 1) % 256;
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  if (WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
