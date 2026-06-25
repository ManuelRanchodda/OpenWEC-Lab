#include <Arduino.h>

/* ================= CONFIGURACIÓN GENERAL ================= */
#define NUM_SENSORS 9
#define SOUND_SPEED 0.0343        // cm/us
#define TIMEOUT_US 25000          // ~4.3 m
#define INTER_SENSOR_DELAY 200    // us
#define SAMPLE_PERIOD_MS 10       // 100 Hz

/* ================= PINES ================= */
const uint8_t TRIG_PINS[NUM_SENSORS] = {
  2, 16, 18, 21, 12, 27, 25, 32, 23
};

const uint8_t ECHO_PINS[NUM_SENSORS] = {
  15, 4, 5, 19, 13, 14, 26, 33, 22
};

/* ================= ESTRUCTURA KALMAN ================= */
typedef struct {
  float x;   // Estado estimado (altura)
  float P;   // Incertidumbre
  float Q;   // Ruido del modelo
  float R;   // Ruido de medición
} Kalman1D;

/* ================= VARIABLES GLOBALES ================= */
float distances[NUM_SENSORS];
unsigned long timeStamp = 0;

Kalman1D kalman[NUM_SENSORS];
SemaphoreHandle_t dataMutex;

/* ================= FUNCIONES ================= */
float readUltrasonic(uint8_t trigPin, uint8_t echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, TIMEOUT_US);
  if (duration == 0) return -1.0;

  return (duration * 0.5) * SOUND_SPEED;
}

float kalmanUpdate(Kalman1D *k, float z) {

  // Predicción
  k->P += k->Q;

  // Ganancia de Kalman
  float K = k->P / (k->P + k->R);

  // Corrección
  k->x = k->x + K * (z - k->x);
  k->P = (1.0 - K) * k->P;

  return k->x;
}

void initKalman() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    kalman[i].x = 0.0;
    kalman[i].P = 1.0;
    kalman[i].Q = 0.05;   // Dinámica de la ola
    kalman[i].R = 3.0;    // Ruido ultrasónico (cm²)
  }
}

/* ================= TASK 1: ADQUISICIÓN ================= */
void TaskUltrasonic(void *pvParameters) {

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);

  while (true) {

    float localDistances[NUM_SENSORS];

    for (int i = 0; i < NUM_SENSORS; i++) {

      float raw = readUltrasonic(TRIG_PINS[i], ECHO_PINS[i]);

      if (raw > 0) {
        localDistances[i] = kalmanUpdate(&kalman[i], raw);
      } else {
        localDistances[i] = kalman[i].x; // mantener estado
      }

      delayMicroseconds(INTER_SENSOR_DELAY);
    }

    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      for (int i = 0; i < NUM_SENSORS; i++) {
        distances[i] = localDistances[i];
      }
      timeStamp = millis();
      xSemaphoreGive(dataMutex);
    }

    vTaskDelayUntil(&lastWakeTime, period);
  }
}

/* ================= TASK 2: SERIAL ================= */
void TaskSerial(void *pvParameters) {

  while (true) {

    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {

      Serial.print(timeStamp);
      Serial.print(",");

      Serial.print(distances[0]); Serial.print(",");
      Serial.print(distances[1]); Serial.print(",");
      Serial.print(distances[2]); Serial.print(",");
      Serial.print(distances[3]); Serial.print(",");
      Serial.print(distances[4]); Serial.print(",");
      Serial.print(distances[5]); Serial.print(",");
      Serial.print(distances[6]); Serial.print(",");
      Serial.print(distances[7]); Serial.print(",");
      Serial.println(distances[8]);

      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* ================= SETUP ================= */
void setup() {

  Serial.begin(115200);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    pinMode(ECHO_PINS[i], INPUT);
    digitalWrite(TRIG_PINS[i], LOW);
  }

  initKalman();
  dataMutex = xSemaphoreCreateMutex();

  Serial.println("time_ms,S1,S2,S3,S4,S5,S6,S7,S8,S9");

  xTaskCreatePinnedToCore(
    TaskUltrasonic,
    "UltrasonicTask",
    4096,
    NULL,
    2,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    TaskSerial,
    "SerialTask",
    4096,
    NULL,
    1,
    NULL,
    1
  );
}

/* ================= LOOP ================= */
void loop() {
  // No se usa
}
