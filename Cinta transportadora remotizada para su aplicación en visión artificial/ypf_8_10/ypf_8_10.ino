#include <WiFi.h>
#include <WebSocketsServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

/* PASOS DE CONEXION
1. Agregar nombre exacto de ssid y pass
2. Poner las IP correctas en el ESP32 y Jetson
*/

// CONFIGURACIÓN DE RED
const char* ssid = "Jetson_Lab_Net";
const char* password = "Laboratorio2026";
const String jetson_url = "http://10.42.0.1:8765/esp32/evento";

IPAddress local_IP(10, 42, 0, 100); // IP fija del ESP32
IPAddress gateway(10, 42, 0, 1);    // IP de la Jetson
IPAddress subnet(255, 255, 255, 0);   // Máscara de subred

WebSocketsServer webSocket = WebSocketsServer(81);

const int SERVO_ILUM_PIN = 22; // GPIO22 para la señal PWM del MG 946R
Servo servoIluminacion;

// configuracion medicion de velocidad
const byte pinA = 34; // Canal A (Interrupción 0 en Arduino Uno/Nano)
const byte pinB = 35; // Canal B (Interrupción 1 en Arduino Uno/Nano)

// --- PARÁMETROS MECÁNICOS ---
const float PPR = 2000.0;                 // Pulsos por revolución del encoder
const float Z1_dientes = 30.0;            // Dientes del engranaje principal (Encoder/Motor)
const float Z2_dientes = 13.0;            // Dientes del engranaje menor (Rodillo)
const float diametroRodilloMetros = 0.08; // Diámetro del rodillo en metros (Ejemplo: 80 mm = 0.08m)
const float relacionTransmision = Z1_dientes / Z2_dientes; 

// --- VARIABLES PARA EL CONTROL DE INTERRUPCIONES ---
volatile long posicionPulsos = 0;          // Contador absoluto de posición (sumar/restar)
volatile long pulsosUltimoIntervalo = 0;   // Pulsos recorridos en el intervalo de medición

// --- VARIABLES DE TIEMPO ---
const unsigned long intervaloMs = 500;     // Tiempo de refresco del cálculo (500 ms)
unsigned long tiempoAnterior = 0;

// Variables para calcular el promedio al final del ensayo
float sumaVelocidades = 0.0;
int cantidadMuestras = 0;

// pines
const int SENSOR_INICIO_PIN = 32;  // sensor barrera de luz

const int RELE_INICIO_PIN   = 25;  //  Marcha Adelante
const int RELE_FIN_PIN      = 26;  //  Marcha Atrás
const int RELE_AUX_PIN      = 27;  //  Parada

const int POT_CS_PIN  = 18; // Chip Select
const int POT_UD_PIN  = 19; // dirección
const int POT_INC_PIN = 21; // Pulsos

const bool RELE_ON  = HIGH;
const bool RELE_OFF = LOW;

const unsigned long TIEMPO_PULSO_RELE = 300;     // 0.3 segundos como pulsador
const unsigned long TIEMPO_ESPERA_PARADA = 2000; // 2 segundos de parada completa

// TIEMPO DE MOVIEMIENTO DE CINTA HACIA ADELANTE
unsigned long TIEMPO_MOVIMIENTO_IDA = 0; 

bool estadoAnteriorInicio = LOW;


// LÓGICA Y MÁQUINAS DE ESTADOS
enum Estado { IDLE, RUNNING };
Estado estado_actual = IDLE;

// CONTROL DE CINTA
enum FaseCinta { REPOSO, INICIO, AVANZANDO, PAUSA_FIN_IDA, RETROCEDIENDO, PAUSA_FIN_VUELTA };
FaseCinta fase_cinta = REPOSO;

unsigned long timer_fase = 0; // Temporizador interno de la fase

int vel_config = 0;
int ilum_config = 0;
int pasadas_config = 0;

int pasada_actual = 0;
unsigned long tiempo_inicio_ciclo = 0;
float vel_prom = 0;


//||||||||||| velocidad de cinta |||||||||||||||||

void ISR_CanalA() {
  // Evalúa el Canal B en el flanco de subida de A para detectar la dirección
  if (digitalRead(pinB) == LOW) {
    posicionPulsos++;           // Giro Horario (Avance)
    pulsosUltimoIntervalo++;
  } else {
    posicionPulsos--;           // Giro Antihorario (Retroceso)
    pulsosUltimoIntervalo--;
  }
}

void posicionarServoIluminacion(int nivel) {
  int angulo = 0;
  
  // posiciones configuradas a mano para que la iluminacion sea distinta

  if (nivel <= 0) {
    angulo = 0;    
  } else if (nivel <= 30) {
    angulo = 60;   
  } else if (nivel <= 60) {
    angulo = 120;   
  } else {
    angulo = 170;  
  }
//angulo = nivel; // para proar angulos a mano

  Serial.print("Nivel ");
  Serial.print(nivel);
  Serial.print(" = Ángulo: ");
  Serial.print(angulo);
  Serial.println("°");

  servoIluminacion.write(angulo);
}

void actualizarVelocidad() {
  unsigned long ahora = millis();
  
  if (ahora - tiempoAnterior >= intervaloMs) {
    noInterrupts(); // Pausa breve para leer variables volátiles de forma segura
    long deltaPulsos = pulsosUltimoIntervalo;
    pulsosUltimoIntervalo = 0;
    interrupts();

    unsigned long dt = ahora - tiempoAnterior;
    tiempoAnterior = ahora;

    if (dt > 0) {
      float rpmEncoder = ((float)deltaPulsos / PPR) * (60000.0 / (float)dt);
      float rpmRodillo = rpmEncoder * relacionTransmision;
      float velocidadMps = (rpmRodillo * 3.14159265 * diametroRodilloMetros) / 60.0; 

      // Solo acumulamos el promedio cuando la cinta está en pleno movimiento (Ida o Vuelta)
      if (fase_cinta == AVANZANDO || fase_cinta == RETROCEDIENDO) {
        sumaVelocidades += abs(velocidadMps);
        cantidadMuestras++;
        
        // Actualiza vel_prom en vivo para el evento de Jetson
        vel_prom = sumaVelocidades / cantidadMuestras;
      }
    }
  }
}

void esperaSegura(unsigned long tiempo) {
  unsigned long inicio = millis();
  while (millis() - inicio < tiempo) {
    webSocket.loop(); 
    delay(10);
  }
}

void enviarEvento(String jsonStr) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(jetson_url);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(jsonStr);
    http.end();
  }
}

void darPulsoRele(int pin, const char* nombreRele) {
  Serial.print("[RELÉ] -> ACTIVANDO "); Serial.println(nombreRele);
  digitalWrite(pin, RELE_ON);
  esperaSegura(TIEMPO_PULSO_RELE); 
  digitalWrite(pin, RELE_OFF);
}

void detenerCinta() {
  darPulsoRele(RELE_AUX_PIN, "Parada (AUX)");
}

void ajustarVelocidadCinta(int nivel_velocidad) {
  Serial.print("[POTENCIÓMETRO] Ajustando velocidad al nivel: ");
  Serial.println(nivel_velocidad);

  // Activa el potenciómetro
  digitalWrite(POT_CS_PIN, LOW); 

  // Bajar la resistencia a 0 
  digitalWrite(POT_UD_PIN, LOW); 
  for (int i = 0; i < 100; i++) {
    digitalWrite(POT_INC_PIN, LOW);
    delayMicroseconds(2); // Retardo mínimo para el pulso
    digitalWrite(POT_INC_PIN, HIGH);
    delay(1);
  }

  // Subir la resistencia hasta el valor configurado
  digitalWrite(POT_UD_PIN, HIGH); 
  for (int i = 0; i < nivel_velocidad; i++) {
    digitalWrite(POT_INC_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(POT_INC_PIN, HIGH);
    delay(1);
  }
  
  digitalWrite(POT_INC_PIN, HIGH);
  digitalWrite(POT_CS_PIN, HIGH); 
  Serial.println("[POTENCIÓMETRO] Velocidad ajustada.");
}

void iniciarCiclo() {
  estado_actual = RUNNING;
  pasada_actual = 0;
  tiempo_inicio_ciclo = millis();
  
  posicionarServoIluminacion(ilum_config);

  
  esperaSegura(500); // estabilización 
  
  enviarEvento("{\"evt\":\"started\",\"pasadas_obj\":" + String(pasadas_config) + "}");
  
  // aplica config de velocidad 
  ajustarVelocidadCinta(vel_config);

  // PASO ATRAS PARA PONER LATAS EN POSICION
  if (digitalRead(SENSOR_INICIO_PIN) == LOW) {
    Serial.println("[INICIO CICLO] Fuera de posición. Retrocediendo al inicio (Homing)...");
    darPulsoRele(RELE_FIN_PIN, "Atrás");
    fase_cinta = INICIO; // Entra en fase de búsqueda
  } else {
    // Si ya está en posición, arranca normal
    Serial.println("[INICIO CICLO] Objeto en posición. Arrancando ensayo hacia Adelante...");
    darPulsoRele(RELE_INICIO_PIN, "Adelante");
    fase_cinta = AVANZANDO;
    timer_fase = millis();
  }
  
  // Reiniciar variables del encoder para el cálculo del promedio
  sumaVelocidades = 0.0;
  cantidadMuestras = 0;
  posicionPulsos = 0;
  pulsosUltimoIntervalo = 0;
  tiempoAnterior = millis();
}

// COMUNICACIÓN (WEBSOCKETS)
void procesarComando(uint8_t num, uint8_t * payload) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) return;

  String cmd = doc["cmd"];

  if (cmd == "config") {
    if (estado_actual == IDLE) {
      if (doc["velocidad"] == 100){
        vel_config = 0;
        TIEMPO_MOVIMIENTO_IDA = 2700;
        vel_prom = 0.31; 
      } else if (doc["velocidad"] == 60){
        vel_config = 30;
        TIEMPO_MOVIMIENTO_IDA = 3850;
        vel_prom = 0.22;
      } else if (doc["velocidad"] == 30){
        vel_config = 60;
        TIEMPO_MOVIMIENTO_IDA = 4300;
        vel_prom = 0.19;
      }

      ilum_config = doc["iluminacion"];
      pasadas_config = doc["pasadas"];
      Serial.print("Velocidad: ");
      Serial.println(vel_config);
      Serial.print("Iluminacion: ");
      Serial.println(ilum_config);
      Serial.print("Pasadas: ");
      Serial.println(pasadas_config);

      webSocket.sendTXT(num, "{\"ok\":true}");
    } else {
      webSocket.sendTXT(num, "{\"ok\":false,\"error\":\"ciclo en curso\"}");
      enviarEvento("{\"evt\":\"rejected\",\"motivo\":\"ciclo en curso\"}");
    }
  } 
  else if (cmd == "start") {
    if (estado_actual == IDLE) {
      webSocket.sendTXT(num, "{\"ok\":true}");
      iniciarCiclo();
    } else {
      webSocket.sendTXT(num, "{\"ok\":false,\"error\":\"ciclo en curso\"}");
      enviarEvento("{\"evt\":\"rejected\",\"motivo\":\"ciclo en curso\"}");
    }
  } 
  else if (cmd == "abort") {
    Serial.println("Comando ABORT recibido!");
    webSocket.sendTXT(num, "{\"ok\":true}");
    detenerCinta();
    estado_actual = IDLE;
    fase_cinta = REPOSO; 
    enviarEvento("{\"evt\":\"aborted\",\"pasada_actual\":" + String(pasada_actual) + "}");
  } 
  else if (cmd == "estado") {
    String estado_str = (estado_actual == IDLE) ? "IDLE" : "RUNNING";
    webSocket.sendTXT(num, "{\"ok\":true,\"estado\":\"" + estado_str + "\",\"pasada\":" + String(pasada_actual) + "}");
    enviarEvento("{\"evt\":\"estado_actual\",\"estado\":\"" + estado_str + "\",\"pasada\":" + String(pasada_actual) + "}");
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    Serial.print("Comando recibido de Jetson: ");
    Serial.println((char*)payload);
    procesarComando(num, payload);
  }
}

// SETUP
void setup() {
  Serial.begin(115200);
  delay(500);

// Inicialización del servo MG 946R
  servoIluminacion.setPeriodHertz(50);                 // 50 Hz estándar
  servoIluminacion.attach(SERVO_ILUM_PIN, 500, 2400); // Rango de pulso MG 946R
  servoIluminacion.write(0);                          // Posición cero inicial

  pinMode(RELE_INICIO_PIN, OUTPUT);
  pinMode(RELE_FIN_PIN, OUTPUT);
  pinMode(RELE_AUX_PIN, OUTPUT);
  digitalWrite(RELE_INICIO_PIN, RELE_OFF);
  digitalWrite(RELE_FIN_PIN, RELE_OFF);
  digitalWrite(RELE_AUX_PIN, RELE_OFF);

  // Configuración de potenciómetro digital
  pinMode(POT_CS_PIN, OUTPUT);
  pinMode(POT_UD_PIN, OUTPUT);
  pinMode(POT_INC_PIN, OUTPUT);
  digitalWrite(POT_CS_PIN, HIGH); 
  digitalWrite(POT_INC_PIN, HIGH);

  pinMode(SENSOR_INICIO_PIN, INPUT);
  estadoAnteriorInicio = digitalRead(SENSOR_INICIO_PIN);

  // Configuración del Encoder
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), ISR_CanalA, RISING);

  // Conexión WiFi
  WiFi.mode(WIFI_STA);

  Serial.print("Conectando a Hotspot Jetson: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Fallo al configurar IP fija de Jetson");
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n¡Conectado al WiFi!");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("Servidor WebSocket iniciado en puerto 81");
}

void loop() {
  webSocket.loop(); // Escucha constante de comandos de la Jetson

  // --- ESCUCHA DE COMANDOS POR MONITOR SERIE ---
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim(); 

    if (comando == "start") {
      if (estado_actual == IDLE) {
        Serial.println("[SERIAL] Comando START recibido.");
        iniciarCiclo();
      } else {
        Serial.println("[SERIAL] Error: Ciclo en curso.");
      }
    } 
    else if (comando == "abort") {
      Serial.println("[SERIAL] Comando ABORT recibido.");
      detenerCinta();
      estado_actual = IDLE;
      fase_cinta = REPOSO;
      enviarEvento("{\"evt\":\"aborted\",\"pasada_actual\":" + String(pasada_actual) + "}");
    } 
    else if (comando.startsWith("config")) {
      int velocidad_deseada = 60;
      int pasadas_deseadas = 1;
      int iluminacion_deseadas = 0;

      sscanf(comando.c_str(), "config %d %d %d", &velocidad_deseada, &pasadas_deseadas, &iluminacion_deseadas);

      if (estado_actual == IDLE) {
        if (velocidad_deseada == 100) {
          vel_config = 0;
          TIEMPO_MOVIMIENTO_IDA = 2700;
          vel_prom = 0.31;
        } else if (velocidad_deseada == 60) {
          vel_config = 30;
          TIEMPO_MOVIMIENTO_IDA = 3850;
          vel_prom = 0.22;
        } else if (velocidad_deseada == 30) {
          vel_config = 60;
          TIEMPO_MOVIMIENTO_IDA = 4300;
          vel_prom = 0.19;
        }
        
        pasadas_config = pasadas_deseadas;
        ilum_config = iluminacion_deseadas;
        Serial.print("[SERIAL] Configuración lista -> Velocidad: ");
        Serial.print(velocidad_deseada);
        Serial.print(" | Pasadas: ");
        Serial.println(pasadas_config);
        Serial.print(" | Iluminacion: ");
        Serial.println(ilum_config);
      } else {
        Serial.println("[SERIAL] Error: No se puede configurar durante un ciclo.");
      }
    }
  }
  // --- FIN DE ESCUCHA POR MONITOR SERIE ---

  if (estado_actual == RUNNING) {
    unsigned long ahora = millis();
    bool estadoActualInicio = digitalRead(SENSOR_INICIO_PIN);

    actualizarVelocidad();

    switch (fase_cinta) {
      
      // POSICIONAMIENTO INICIAL
      case INICIO:
        if (estadoActualInicio == HIGH) { 
          Serial.println("\n[HOMING] Sensor de inicio detectado. Arrancando ensayo...");
          
          detenerCinta(); 
          esperaSegura(TIEMPO_ESPERA_PARADA);
          darPulsoRele(RELE_INICIO_PIN, "ADELANTE"); 
          
          fase_cinta = AVANZANDO;
          timer_fase = millis();
        }
        break;

      // PASO 1: Cinta va hacia adelante hasta cumplir el tiempo
      case AVANZANDO:
        if (ahora - timer_fase >= TIEMPO_MOVIMIENTO_IDA) {
          Serial.println("\n[TIEMPO] Tiempo de ida completado. Frenando...");
          detenerCinta(); 
          fase_cinta = PAUSA_FIN_IDA;
          timer_fase = millis(); 
        }
        break;

      // PASO 2: la cinta frena con el tiempo de espera por la inercia
      case PAUSA_FIN_IDA:
        if (ahora - timer_fase >= TIEMPO_ESPERA_PARADA) {
          Serial.println("\n[SECUENCIA] Arrancando hacia Atrás...");
          darPulsoRele(RELE_FIN_PIN, "Atrás");
          fase_cinta = RETROCEDIENDO;
        }
        break;

      // PASO 3: Cinta vuelve hasta que el sensor de inicio detecte
      case RETROCEDIENDO:
        if (estadoActualInicio == HIGH && estadoAnteriorInicio == LOW) {
          Serial.println("\n[SENSOR] Objeto detectado en INICIO. Fin de pasada.");
          detenerCinta();
          pasada_actual++;
               
          float vel_promedio = (cantidadMuestras > 0) ? (sumaVelocidades / cantidadMuestras) : 0.0;
          // progreso jetson
          
          Serial.println("{\"evt\":\"progress\",\"pasada\":" + String(pasada_actual) + ",\"vel_prom\":" + String(vel_promedio) + "}");

          enviarEvento("{\"evt\":\"progress\",\"pasada\":" + String(pasada_actual) + ",\"vel_prom\":" + String(vel_promedio) + "}");


          fase_cinta = PAUSA_FIN_VUELTA;
          timer_fase = millis(); 
        }
        break;

      // PASO 4: chequea num de pasada
      case PAUSA_FIN_VUELTA:
        if (ahora - timer_fase >= TIEMPO_ESPERA_PARADA) {
          if (pasada_actual >= pasadas_config) {
          
            unsigned long duracion = (millis() - tiempo_inicio_ciclo) / 1000;
            
            // CÁLCULO DEL PROMEDIO FINAL
            float vel_promedio = (cantidadMuestras > 0) ? (sumaVelocidades / cantidadMuestras) : 0.0;

            Serial.println("{\"evt\":\"finished\",\"pasadas\":" + String(pasadas_config) + ",\"vel_prom\":" + String(vel_prom) + ",\"duracion_s\":" + String(duracion) + "}");
           
            enviarEvento("{\"evt\":\"finished\",\"pasadas\":" + String(pasadas_config) + ",\"vel_prom\":" + String(vel_prom) + ",\"duracion_s\":" + String(duracion) + "}");
            estado_actual = IDLE;
            fase_cinta = REPOSO;
            Serial.println("[ESTADO] Ensayo Completado. Volviendo a IDLE.");
          } else {
            // Arranca la siguiente pasada 
            Serial.println("\n[SECUENCIA] Iniciando pasada " + String(pasada_actual + 1) + ". Adelante...");
            darPulsoRele(RELE_INICIO_PIN, "Adelante");
            fase_cinta = AVANZANDO;
            timer_fase = millis();
          }
        }
        break;

      case REPOSO:
        break;
    }

    // Actualizar estado del sensor para leer los flancos (pulsos)
    estadoAnteriorInicio = estadoActualInicio;
    
  } else {
    // para el caso de "abort", se detiene la cinta
    fase_cinta = REPOSO;
  }
}