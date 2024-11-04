//Librerias
#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "HX711.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Parámetros STEPPER EVA
#define MOTOR_STEPS_EVA 200
#define RPM_EVA 50
#define MICROSTEPS_EVA 4


//Parámetros STEPPER RAMSES
#define MOTOR_STEPS_RAMSES 200
#define RPM_RAMSES 20
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

//Constantes globales
const double factor_EVA = (200 / (59.54)) - 0.05; //Factor actuador lineal del EVA. Medidio y ajustado experimentalmente. Unidades [pasos/mm]

const double factor_RAMSES = 15.3 * 200.0 * (1.0 / 360.0); //Factor motor RAMSES. Calculado con la reducción de la caja. Unidaddes [pasos/grados]

const double distancias[] =
{
  0,
  15.08333, 28.16667, 41.25000, 54.33333, 67.41667, 80.50000,
  97.58333, 110.66667, 123.75000, 136.83333, 149.91667, 163.00000,
  182.08333, 195.16667, 208.25000, 221.33333, 234.41667, 247.50000,
  264.58333, 277.66667, 290.75000, 303.83333, 316.91667, 330.00000
}; //Datos de distancias entre compartimientos. Obtenidos del diseño CAD. Unidades [mm]

const unsigned int pos0 = 172; // ancho de pulso en cuentas para pocicion 0° (PCA)
const unsigned int pos180 = 565; // ancho de pulso en cuentas para la pocicion 180° (PCA)
const unsigned int degs_reposo = 95; //Grados de reposo para servos de rotación continua

const boolean abierto = false;
const boolean cerrado = true;

const unsigned int caja1 = 1;
const unsigned int caja2 = 2;
const unsigned int caja3 = 3;
const unsigned int caja4 = 4;

//Canales PCA9685
const unsigned int servo_caja1 = 0;
const unsigned int servo_caja2 = 1;
const unsigned int servo_caja3 = 2;
const unsigned int servo_caja4 = 3;
const unsigned int servo_disp = 4;
const unsigned int servo_niv = 5;
const unsigned int servo_ex = 6;


//Variables globales
String inputString = "";
char comando = "";
int compartimiento_actual = 0;
double peso_inicial = 0;
unsigned long tiempo;
boolean estado_EVA = false;
boolean estado_RAMSES = false;
boolean falla = false;


//Declaración STEPPER EVA
BasicStepperDriver stepper_EVA(MOTOR_STEPS_EVA, DIR_EVA, STEP_EVA);

//Declaración STEPPER RAMSES
BasicStepperDriver stepper_RAMSES(MOTOR_STEPS_RAMSES, DIR_RAMSES, STEP_RAMSES);

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
  servos.setPWM(servo_disp, 0, deg2pulse(degs_reposo));
  servos.setPWM(servo_niv, 0, pos0);
  servos.setPWM(servo_ex, 0, pos0);

  //Inicialización STEPPERs
  pinMode(EN_EVA, OUTPUT);
  pinMode(EN_RAMSES, OUTPUT);
  digitalWrite(EN_EVA, HIGH);
  digitalWrite(EN_RAMSES, HIGH);
  estado_EVA = false;
  estado_RAMSES = false;
  stepper_EVA.begin(RPM_EVA, MICROSTEPS_EVA);
  stepper_RAMSES.begin(RPM_RAMSES, MICROSTEPS_RAMSES);

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
  Serial.println("For command info press \"?\" ");
}

//No se corre nada en el loop
void loop() {
}

void io_EVA(boolean estado) {
  if (estado) {
    digitalWrite(EN_EVA, LOW);
    estado_EVA = true;
  }
  else {
    digitalWrite(EN_EVA, HIGH);
    estado_EVA = false;
  }
}
void io_RAMSES(boolean estado) {
  if (estado) {
    digitalWrite(EN_RAMSES, LOW);
    estado_EVA = true;
  }
  else {
    digitalWrite(EN_RAMSES, HIGH);
    estado_EVA = false;
  }
}
void homear_EVA() {
  if (estado_EVA) {
    tiempo = millis();
    stepper_EVA.startMove(20000 * MICROSTEPS_EVA);
    while (digitalRead(PIN_FDC_EVA) == HIGH) {
      stepper_EVA.nextAction();
      if (millis() - tiempo > 15000) {
        falla = true;
        Serial.println("ERROR: Timeout");
        break;
      }
    }
    if (!falla) {
      stepper_EVA.stop();
      stepper_EVA.move(-40);
      stepper_EVA.begin(RPM_EVA * 0.5, MICROSTEPS_EVA);
      stepper_EVA.startMove(20000 * MICROSTEPS_EVA);
      while (digitalRead(PIN_FDC_EVA) == HIGH) {
        stepper_EVA.nextAction();
      }
      stepper_EVA.begin(RPM_EVA, MICROSTEPS_EVA);
      stepper_EVA.move(-10 * MICROSTEPS_EVA);
      compartimiento_actual = 0;
    }
    falla = false;
    delay(1000);
  }
  else {
    Serial.println("ERROR: EVA stepper is not turned ON");
  }
}
void homear_RAMSES() {
  if (estado_RAMSES) {
    tiempo = millis();
    stepper_RAMSES.startMove(-20000 * MICROSTEPS_RAMSES);
    while (digitalRead(PIN_FDC_B1) == HIGH) {
      stepper_RAMSES.nextAction();
      if (millis() - tiempo > 15000) {
        falla = true;
        Serial.println("ERROR: Timeout");
        break;
      }
    }
    if (!falla) {
      stepper_RAMSES.stop();
      stepper_RAMSES.move(40);
      stepper_RAMSES.begin(RPM_RAMSES * 0.5, MICROSTEPS_RAMSES);
      stepper_RAMSES.startMove(-20000 * MICROSTEPS_RAMSES);
      while (digitalRead(PIN_FDC_B1) == HIGH) {
        stepper_RAMSES.nextAction();
      }
      stepper_RAMSES.begin(RPM_RAMSES, MICROSTEPS_RAMSES);
      stepper_RAMSES.move(-10 * MICROSTEPS_RAMSES);
    }
    falla = false;
    delay(1000);
  }
  else {
    Serial.println("ERROR: RAMSES stepper is not turned ON");
  }
}
void actuador_auto_EVA(int act, int obj) {
  if (estado_EVA) {
    stepper_EVA.move(-MICROSTEPS_EVA * ((int) (factor_EVA * distancias[obj]) - (int) (factor_EVA * distancias[act])));
    compartimiento_actual = obj;
  }
  else {
    Serial.println("ERROR: EVA stepper is not turned ON");

  }
}
void actuador_manual_EVA(boolean dir, double distancia_mm) {
  if (estado_EVA) {
    if (dir) {
      stepper_EVA.move(MICROSTEPS_EVA * ((int)(factor_EVA * distancia_mm)));
    }
    else {
      stepper_EVA.move(-MICROSTEPS_EVA * ((int)(factor_EVA * distancia_mm)));
    }
  }
  else {
    Serial.println("ERROR: EVA stepper is not turned ON");
  }


}
void actuador_manual_RAMSES(boolean dir, double distancia_grados) {
  if (dir) {
    stepper_RAMSES.move(MICROSTEPS_RAMSES * ((int)(factor_RAMSES * distancia_grados)));
  }
  else {
    stepper_RAMSES.move(-MICROSTEPS_RAMSES * ((int)(factor_RAMSES * distancia_grados)));
  }
}
void caja_EVA(int caja, boolean estado) {
  if (estado == true) {
    if (caja == caja1) {
      servos.setPWM(servo_caja1, 0, deg2pulse(180));
    }
    else if (caja == caja2) {
      servos.setPWM(servo_caja2, 0, deg2pulse(180));
    }
    else if (caja == caja3) {
      servos.setPWM(servo_caja3, 0, deg2pulse(180));
    }
    else if (caja == caja4) {
      servos.setPWM(servo_caja4, 0, deg2pulse(180));
    }
    else {}
  }
  else if (estado == false) {
    if (caja == caja1) {
      servos.setPWM(servo_caja1, 0, deg2pulse(0));
    }
    else if (caja == caja2) {
      servos.setPWM(servo_caja2, 0, deg2pulse(0));
    }
    else if (caja == caja3) {
      servos.setPWM(servo_caja3, 0, deg2pulse(0));
    }
    else if (caja == caja4) {
      servos.setPWM(servo_caja4, 0, deg2pulse(0));
    }
    else {}
  }
  else {}
  delay(1000);
}
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
        actuador_auto_EVA(compartimiento_actual, compartimiento_temp);
        Serial.println("Arrived at comparment " + String(compartimiento_temp));
      }
      else {
        Serial.println("ERROR: Invalid compartment");
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
        Serial.println("ERROR: Invalid home");
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
          Serial.println("ERROR: Invalid status");
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
          Serial.println("ERROR: Invalid status");
        }
      }
      else {
        Serial.println("ERROR: Invalid system");
      }
    }
    if (comando == 'b' || comando == 'B') {
      int caja_temp = String(inputString.charAt(1)).toInt();
      if (caja_temp >= 1 && caja_temp <= 4) {
        if (String(inputString.charAt(2)).toInt() == 1) {
          Serial.println("Opening box " + String(caja_temp));
          caja_EVA(caja_temp, abierto);
          Serial.println("Box " + String(caja_temp) + " is open");
        }
        else if (String(inputString.charAt(2)).toInt() == 0) {
          Serial.println("Closing box " + String(caja_temp));
          caja_EVA(caja_temp, cerrado);
          Serial.println("Box " + String(caja_temp) + " is closed");
        }
        else {
          Serial.println("ERROR: Invalid box state");
        }
      }
      else {
        Serial.println("ERROR: Invalid box");
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
        Serial.println("ERROR: Invalid shaking time");

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
          Serial.println("ERROR: Invalid dispensing time");
        }
      }
      else {
        Serial.println("ERROR: Invalid dispensing speed");
      }

    }
    if (comando == 'c' || comando == 'C') {
      int vel_temp = String(inputString.charAt(1)).toInt();
      if (vel_temp >= 0 && vel_temp <= 5) {
        limpiar_EVA(vel_temp);
        Serial.println("Cleaning speed set to " + String(vel_temp));
      }
      else {
        Serial.println("ERROR: Invalid cleaning speed");
      }
    }
    
    comando = "";
    inputString = "";
  }
  if (comando == '?') {
    Serial.println("Enable steppers: E[subsystem][status] {E,X}{1,0}");
    Serial.println("Home steppers: H[subsystem] {E,X}");

    Serial.println("EVA - Auto move dispenser: M[#compartment] {00,01,02,...,24}");
    Serial.println("EVA - Manual move dispenser: -M[direction][10*milimeters] {0,1}{000,0001,0002,...,9999}");
    Serial.println("EVA - Open/close lids: B[#box][status] {1,2,3,4}(0,1}");
    Serial.println("EVA - Shake: S[time_seconds] {01,02,03,...,30}");
    Serial.println("EVA - Funnel cleaner speed C[speed] {0,1,2,...,5}");
    Serial.println("EVA - Auto dispense D[grams]");
    Serial.println("EVA - Manual dispense -D[speed][time_seconds] {1,2,3,4,5}{1,2,3,...,9}");

    Serial.println("RAMSES - Auto move arm A[position] {U,D}");
    Serial.println("RAMSES - Manual move arm -A[direction][10*degrees] {0,1}{0000,0001,0002,...,9999}");
    Serial.println("RAMSES - Gather samples");
  }
  comando = "";
  inputString = "";
}


int deg2pulse(int deg) {
  return map(deg, 0, 180, pos0, pos180);
}
