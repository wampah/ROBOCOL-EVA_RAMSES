//Librerias
#include <AccelStepper.h>
#include "HX711.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Parámetros STEPPER EVA
#define INTERFACE_EVA 1
#define MOTOR_STEPS_EVA 200
#define SPEED_EVA 600
#define ACCEL_EVA 2000
#define MICROSTEPS_EVA 4


//Parámetros STEPPER RAMSES
#define INTERFACE_RAMSES 1
#define MOTOR_STEPS_RAMSES 200
#define SPEED_RAMSES 100
#define ACCEL_RAMSES 20
#define MICROSTEPS_RAMSES 1

//Pines
#define EN_EVA 5
#define DIR_EVA 7
#define STEP_EVA 6

#define EN_RAMSES 8
#define DIR_RAMSES 10
#define STEP_RAMSES 9

#define PIN_FDC_B2 A3
#define PIN_FDC_B1 A2
#define PIN_FDC_EVA A1

#define PIN_VIB 12
#define PIN_TAMIZ 11

#define PIN_CELDA_DT A0
#define PIN_CELDA_SCK 13

#define PIN_LIMP_A 4
#define PIN_LIMP_B 2
#define PIN_LIMP_CTRL 3

//Canales PCA
#define servo_caja1 3
#define servo_caja2 1
#define servo_caja3 2
#define servo_caja4 0
#define servo_disp 4
#define servo_niv 5
#define servo_ex 6

//Constantes globales
const double factor_EVA = (200.0 / (59.54)) - 0.05; //Factor actuador lineal del EVA. Medidio y ajustado experimentalmente. Unidades [pasos/mm]

const double factor_RAMSES = 15.3 * 200.0 * (1.0 / 360.0); //Factor motor RAMSES. Calculado con la reducción de la caja. Unidaddes [pasos/grados]

const double distancias_cajas_mm[] =
{
  0,
  15.08333, 28.16667, 41.25000, 54.33333, 67.41667, 80.50000,
  97.58333, 110.66667, 123.75000, 136.83333, 149.91667, 163.00000,
  182.08333, 195.16667, 208.25000, 221.33333, 234.41667, 247.50000,
  264.58333, 277.66667, 290.75000, 303.83333, 316.91667, 330.00000
}; //Datos de distancias entre compartimientos. Obtenidos del diseño CAD. Unidades [mm]

const unsigned int pos0 = 172; // ancho de pulso para pocicion 0° (PCA)
const unsigned int pos180 = 565; // ancho de pulso para la pocicion 180° (PCA)
const unsigned int degs_reposo = 95; //Grados de reposo para servos de rotación continua

//Variables globales
String inputString = "";
char comando = "";
double peso_inicial = 0;
unsigned long tiempo;
boolean estado_EVA = false;
boolean estado_RAMSES = false;
String error_msg = "ERROR: ";

//Declaración STEPPER EVA
AccelStepper stepper_EVA(INTERFACE_EVA, STEP_EVA, DIR_EVA);

//Declaración STEPPER RAMSES
AccelStepper stepper_RAMSES(INTERFACE_RAMSES, STEP_RAMSES, DIR_RAMSES);

//Declaración PCA9685
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

//Declaración de celda de carga
HX711 celda;

//Inicialización
void setup() {
  //Inicialización serial
  Serial.begin(57600);

  Serial.println("Initializing...");

  //Inicialización I2C
  Wire.begin();

  //Inicialización PCA9685 y señales iniciales. Frecuencia es 60Hz para servos. Servo dispensador esta quieto en 90/255.
  servos.begin();
  servos.setPWMFreq(60);
  servos.setPWM(servo_caja1, 0, pos0);
  servos.setPWM(servo_caja2, 0, pos0);
  servos.setPWM(servo_caja3, 0, pos0);
  servos.setPWM(servo_caja4, 0, pos0);
  servos.setPWM(servo_disp, 0, 1);
  servos.setPWM(servo_niv, 0, pos0);
  servos.setPWM(servo_ex, 0, pos0);

  //Inicialización STEPPERs
  stepper_EVA.setMaxSpeed(SPEED_EVA);
  stepper_EVA.setAcceleration(ACCEL_EVA);
  stepper_EVA.setEnablePin(EN_EVA);
  stepper_EVA.setPinsInverted(false, false, false);
  stepper_EVA.disableOutputs();


  stepper_RAMSES.setMaxSpeed(SPEED_RAMSES);
  stepper_RAMSES.setAcceleration(ACCEL_RAMSES);
  stepper_RAMSES.setEnablePin(EN_RAMSES);
  stepper_RAMSES.setPinsInverted(false, false, false);

  stepper_RAMSES.disableOutputs();
  estado_EVA = false;
  estado_RAMSES = false;

  //Inicialización y señal inicial pin rele de motores de vibración
  pinMode(PIN_VIB, OUTPUT);
  digitalWrite(PIN_VIB, HIGH);

  //Inicialización y señal inicial pin rele de tamiz
  pinMode(PIN_TAMIZ, OUTPUT);
  digitalWrite(PIN_TAMIZ, HIGH);

  //Inicalización pines DT y CLK celda de carga
  celda.begin(PIN_CELDA_DT, PIN_CELDA_SCK);

  //Inicialización de interruptores fin de carrera.
  pinMode(PIN_FDC_EVA, INPUT_PULLUP);
  pinMode(PIN_FDC_B1, INPUT_PULLUP);
  pinMode(PIN_FDC_B2, INPUT_PULLUP);

  //Inicialización motor de limpieza y señal inicial
  pinMode(PIN_LIMP_A, OUTPUT);
  pinMode(PIN_LIMP_B, OUTPUT);
  pinMode(PIN_LIMP_CTRL, OUTPUT);

  digitalWrite(PIN_LIMP_A, LOW);
  digitalWrite(PIN_LIMP_B, LOW);
  analogWrite(PIN_LIMP_CTRL, 0);

  Serial.println("EVA & RAMSES ready!");
}

//No se corre nada en el loop
void loop() {
}

//Función para encender o apagar el stepper del EVA.
void io_EVA(boolean estado) {
  if (estado) {
    stepper_EVA.enableOutputs();
    estado_EVA = true;
  }
  else {
    stepper_EVA.disableOutputs();
    estado_EVA = false;
  }
}

//Función para encender o apagar el stepper del RAMSES.
void io_RAMSES(boolean estado) {
  if (estado) {
    stepper_RAMSES.enableOutputs();
    estado_RAMSES = true;
  }
  else {
    stepper_RAMSES.disableOutputs();
    estado_RAMSES = false;
  }
}

//Función para hacer el home del EVA
void homear_EVA() {
  if (estado_EVA) {
    tiempo = millis();
    stepper_EVA.move(20000 * MICROSTEPS_EVA);
    while (digitalRead(PIN_FDC_EVA) == HIGH) {
      stepper_EVA.run();
      if (millis() - tiempo > 15000) {
        Serial.println(error_msg + "Timeout");
        return;
      }
    }
    stepper_EVA.stop();
    stepper_EVA.setCurrentPosition(0);
    stepper_EVA.runToNewPosition(-40);
    stepper_EVA.setMaxSpeed(SPEED_EVA * 0.25);
    stepper_EVA.move(20000 * MICROSTEPS_EVA);
    while (digitalRead(PIN_FDC_EVA) == HIGH) {
      stepper_EVA.run();
    }
    stepper_EVA.stop();
    stepper_EVA.setMaxSpeed(SPEED_EVA);
    stepper_EVA.runToNewPosition(-10 * MICROSTEPS_EVA);
    stepper_EVA.setCurrentPosition(0);

    delay(1000);
  }
  else {
    Serial.println(error_msg + "EVA stepper is not turned ON");
  }
}

//Función para hacer el home del RAMSES
void homear_RAMSES() {
  if (estado_RAMSES) {
    tiempo = millis();
    stepper_RAMSES.move(20000 * MICROSTEPS_RAMSES);
    while (digitalRead(PIN_FDC_B1) == HIGH) {
      stepper_RAMSES.run();
      if (millis() - tiempo > 15000) {
        Serial.println(error_msg + "Timeout");
        return;
      }
    }
    stepper_RAMSES.stop();
    stepper_RAMSES.setCurrentPosition(0);
    stepper_RAMSES.runToNewPosition(-40);
    stepper_RAMSES.setMaxSpeed(SPEED_RAMSES * 0.25);
    stepper_RAMSES.move(20000 * MICROSTEPS_RAMSES);
    while (digitalRead(PIN_FDC_B1) == HIGH) {
      stepper_RAMSES.run();
    }
    stepper_RAMSES.stop();
    stepper_RAMSES.setMaxSpeed(SPEED_RAMSES);
    stepper_RAMSES.runToNewPosition(-10 * MICROSTEPS_RAMSES);
    stepper_RAMSES.setCurrentPosition(0);
    delay(1000);
  }
  else {
    Serial.println(error_msg + "RAMSES stepper is not turned ON");
  }
}

//Función que mueve el dispensador del EVA automáticamente entre cajas.
void actuador_auto_EVA(int obj) {
  if (estado_EVA) {
    stepper_EVA.moveTo((int)(-MICROSTEPS_EVA *  factor_EVA * distancias_cajas_mm[obj]));
    while (stepper_EVA.distanceToGo() != 0) {
      stepper_EVA.run();
    }
  }
  else {
    Serial.println(error_msg + "EVA stepper is not turned ON");
  }
}

//Función que mueve el dispensador del EVA manualmente
void actuador_manual_EVA(boolean dir, double distancia_mm) {
  if (estado_EVA) {
    if (dir) {
      stepper_EVA.move((int)(MICROSTEPS_EVA * (factor_EVA * distancia_mm)));
      while (stepper_EVA.distanceToGo() != 0) {
        stepper_EVA.run();
      }
    }
    else {
      stepper_EVA.move((int)(-MICROSTEPS_EVA * (factor_EVA * distancia_mm)));
      while (stepper_EVA.distanceToGo() != 0) {
        stepper_EVA.run();
      }
    }
  }
  else {
    Serial.println(error_msg + "EVA stepper is not turned ON");
  }
}

//Función que mueve manualmente el brazo del RAMSES
void actuador_manual_RAMSES(boolean dir, double distancia_grados) {
  if (dir) {
    stepper_RAMSES.moveTo((int)(MICROSTEPS_RAMSES * factor_RAMSES * distancia_grados));
    while (stepper_RAMSES.distanceToGo() != 0) {
      stepper_RAMSES.run();
    }
  }
  else {
    stepper_RAMSES.moveTo((int)(-MICROSTEPS_RAMSES * factor_RAMSES * distancia_grados));
    while (stepper_RAMSES.distanceToGo() != 0) {
      stepper_RAMSES.run();
    }
  }
}

//Función que mueve automáticamente el brazo del RAMSES
void actuador_autom_RAMSES(boolean pos) {
  if (pos) {
    stepper_RAMSES.moveTo(0);
  }
  else {
    stepper_RAMSES.moveTo(600);
  }
}

//Función para abrir o cerrar las compuertas del EVA
void caja_EVA(int caja, boolean estado) {
  if (estado == true) {
    if (caja == 1) {
      servos.setPWM(servo_caja1, 0, deg2pulse(180));
    }
    else if (caja == 2) {
      servos.setPWM(servo_caja2, 0, deg2pulse(180));
    }
    else if (caja == 3) {
      servos.setPWM(servo_caja3, 0, deg2pulse(180));
    }
    else if (caja == 4) {
      servos.setPWM(servo_caja4, 0, deg2pulse(180));
    }
    else {}
  }
  else if (estado == false) {
    if (caja == 1) {
      servos.setPWM(servo_caja1, 0, deg2pulse(0));
    }
    else if (caja == 2) {
      servos.setPWM(servo_caja2, 0, deg2pulse(0));
    }
    else if (caja == 3) {
      servos.setPWM(servo_caja3, 0, deg2pulse(0));
    }
    else if (caja == 4) {
      servos.setPWM(servo_caja4, 0, deg2pulse(0));
    }
    else {}
  }
  else {}
  delay(1000);
}

//Función para activar los motores de vibración del EVA
void mezclar_EVA(int t_segundos) {
  digitalWrite(PIN_VIB, LOW);
  delay(t_segundos * 1000);
  digitalWrite(PIN_VIB, HIGH);
  delay(1000);
}
void limpiar_EVA(int vel) {
  if (vel == 0) {
    digitalWrite(PIN_LIMP_A, LOW);
    digitalWrite(PIN_LIMP_B, LOW);
  }
  else {
    digitalWrite(PIN_LIMP_A, LOW);
    digitalWrite(PIN_LIMP_B, HIGH);
    analogWrite(PIN_LIMP_CTRL, (int)(255.0 * ( (double)vel / 5)));
  }
  delay(1000);
}

void dispensar_manual_EVA(int vel, int t_segundos) {
  servos.setPWM(servo_disp, 0, deg2pulse((int)(90.0 - (90.0 * ( (double)vel / 5)))));
  delay(t_segundos * 1000);
  servos.setPWM(servo_disp, 0, deg2pulse(degs_reposo));
  delay(1000);
}

//FALTAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
void tara() {
  peso_inicial = 0;
}
double pesar() {
  if (celda.is_ready()) {
    long reading = celda.read();
    Serial.print("HX711 reading: ");
    Serial.println(reading);
  } else {
    Serial.println("HX711 not found.");
  }
}

void serialEvent() {
  while (Serial.available()) {

    //Recibir Datos de Raspberry
    String inputString = Serial.readStringUntil('\n');
    Serial.print("You sent me: ");
    Serial.println(inputString);
    char comando = inputString.charAt(0);

    if (comando == 'm' || comando == 'M') {
      int compartimiento_temp = String(inputString.charAt(1)).toInt() * 10 + String(inputString.charAt(2)).toInt();
      if (compartimiento_temp <= 24 && compartimiento_temp >= 0) {
        Serial.println("Moving to comparment " + String(compartimiento_temp) + "...");
        actuador_auto_EVA(compartimiento_temp);
        Serial.println("Move to " + String(compartimiento_temp)+" finished");
      }
      else {
        Serial.println(error_msg + "Invalid compartment");
      }
    }
    if (comando == 'h' || comando == 'H') {
      char sistema_temp = inputString.charAt(1);
      if (sistema_temp == 'E' || sistema_temp == 'e') {
        Serial.println("Homing EVA...");
        homear_EVA();
        Serial.println("Homing EVA finished");
      }
      else if (sistema_temp == 'X' || sistema_temp == 'x') {
        Serial.println("Homing RAMSES...");
        homear_RAMSES();
        Serial.println("Homing RAMSES finished");
      }
      else {
        Serial.println(error_msg + "Invalid home");
      }
    }
    if (comando == 'e' || comando == 'E') {
      char sistema_temp = inputString.charAt(1);
      boolean estado_temp = false;

      if (sistema_temp == 'E' || sistema_temp == 'e') {
        if (String(inputString.charAt(2)).toInt() == 1) {
          io_EVA(true);
          Serial.println("EVA turned ON");
        }
        else if (String(inputString.charAt(2)).toInt() == 0) {
          io_EVA(false);
          Serial.println("EVA turned OFF");
        }
        else {
          Serial.println(error_msg + "Invalid status");
        }
      }
      else if (sistema_temp == 'X' || sistema_temp == 'x') {
        if (String(inputString.charAt(2)).toInt() == 1) {
          io_RAMSES(true);
          Serial.println("RAMSES turned ON");
        }
        else if (String(inputString.charAt(2)).toInt() == 0) {
          io_RAMSES(false);
          Serial.println("RAMSES turned OFF");
        }
        else {
          Serial.println(error_msg + "Invalid status");
        }
      }
      else {
        Serial.println(error_msg + "Invalid system");
      }
    }
    if (comando == 'b' || comando == 'B') {
      int caja_temp = String(inputString.charAt(1)).toInt();
      if (caja_temp >= 1 && caja_temp <= 4) {
        if (String(inputString.charAt(2)).toInt() == 1) {
          Serial.println("Opening box " + String(caja_temp));
          caja_EVA(caja_temp, false);
          Serial.println("Box " + String(caja_temp) + " is open");
        }
        else if (String(inputString.charAt(2)).toInt() == 0) {
          Serial.println("Closing box " + String(caja_temp));
          caja_EVA(caja_temp, true);
          Serial.println("Box " + String(caja_temp) + " is closed");
        }
        else {
          Serial.println(error_msg + "Invalid box state");
        }
      }
      else {
        Serial.println(error_msg + "Invalid box");
      }
    }
    if (comando == 's' || comando == 'S') {
      int t_mezclado_temp = String(inputString.charAt(1)).toInt() * 10 + String(inputString.charAt(2)).toInt();
      if (t_mezclado_temp > 0 && t_mezclado_temp <= 30) {
        Serial.println("Shaking samples for " + String(t_mezclado_temp) + " seconds");
        mezclar_EVA(t_mezclado_temp);
        Serial.println("Shaking finished");
      }
      else {
        Serial.println(error_msg + "Invalid shaking time");
      }
    }
    if (comando == 'd' || comando == 'D') {
      int vel_temp = String(inputString.charAt(1)).toInt();
      int t_disp_temp = String(inputString.charAt(2)).toInt();
      if (vel_temp > 0 && vel_temp <= 5) {
        if (t_disp_temp > 0 && t_disp_temp <= 9) {
          Serial.println("Dispensing sample at speed " + String(vel_temp) + " for " + String(t_disp_temp) + " seconds");
          dispensar_manual_EVA(vel_temp, t_disp_temp);
          Serial.println("Sample dispensed");
        }
        else {
          Serial.println(error_msg + "Invalid dispensing time");
        }
      }
      else {
        Serial.println(error_msg + "Invalid dispensing speed");
      }

    }
    if (comando == 'c' || comando == 'C') {
      int vel_temp = String(inputString.charAt(1)).toInt();
      if (vel_temp >= 0 && vel_temp <= 5) {
        limpiar_EVA(vel_temp);
        Serial.println("Cleaning speed set to " + String(vel_temp));
      }
      else {
        Serial.println(error_msg + "Invalid cleaning speed");
      }
    }
    if (comando == '-') {
      char comando_temp = inputString.charAt(1);
      if (comando_temp == 'd' || comando_temp == 'D') {
        int vel_temp = String(inputString.charAt(2)).toInt();
        int t_disp_temp = String(inputString.charAt(3)).toInt();
        if (vel_temp > 0 && vel_temp <= 5) {
          if (t_disp_temp > 0 && t_disp_temp <= 9) {
            Serial.println("Dispensing sample at speed " + String(vel_temp) + " for " + String(t_disp_temp) + " seconds");
            dispensar_manual_EVA(vel_temp, t_disp_temp);
            Serial.println("Sample dispensed");
          }
          else {
            Serial.println(error_msg + "Invalid dispensing time");
          }
        }
        else {
          Serial.println(error_msg + "Invalid dispensing speed");
        }

      }
      else if (comando_temp == 'a' || comando_temp == 'A') {
        int dir_temp = String(inputString.charAt(2)).toInt();
        double grados_temp = 100.0 * ((double)String(inputString.charAt(3)).toInt()) + 10.0 * ((double)String(inputString.charAt(4)).toInt()) + ((double)String(inputString.charAt(5)).toInt()) + 0.1 * ((double)String(inputString.charAt(6)).toInt());
        if (dir_temp == 1) {
          actuador_manual_RAMSES(true,  grados_temp);
        }
        else if (dir_temp == 0) {
          actuador_manual_RAMSES(false,  grados_temp);
        }
        else {
          Serial.println(error_msg + "Invalid sign");
        }
      }
      else if (comando_temp == 'm' || comando_temp == 'M') {
        int dir_temp = String(inputString.charAt(2)).toInt();
        double milimetros_temp = 100.0 * ((double)String(inputString.charAt(3)).toInt()) + 10.0 * ((double)String(inputString.charAt(4)).toInt()) + ((double)String(inputString.charAt(5)).toInt()) + 0.1 * ((double)String(inputString.charAt(6)).toInt());
        if (dir_temp == 1) {
          actuador_manual_EVA(true, milimetros_temp);
        }
        else if (dir_temp == 0) {
          actuador_manual_EVA(false, milimetros_temp);
        }
        else {
          Serial.println(error_msg + "Invalid sign");
        }
      }
      else {
        Serial.println(error_msg + "Invalid manual move command");
      }
    }
    comando = "";
    inputString = "";
  }
}

int deg2pulse(int deg) {
  return map(deg, 0, 180, pos0, pos180);
}
