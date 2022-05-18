#include <Arduino.h>
#include "robot_paralelo.h"//esta clase es la que implementa la cinematica inversa del robot
#include "parametros.h"


//############Variables de los servos#######################################################################
#define PINSERVO1 13//pines a los que se conectan los servomotores
#define PINSERVO2 14
#define PINSERVO3 17
//##########################################################################################################

//############Variables del joystick########################################################################
#define PINJOY_CABE 33
#define PINJOY_ALAB 32
#define PINJOY_ELEV 2

float JoyCabeceo, JoyAlabeo, JoyElevacion;
float CabeceoOffset, AlabeoOffset;//para eliminar el offset del joystick
//##########################################################################################################

//###################MANEJO DE BOTON CON ISR(INTERRUPT)############

struct Button {
  const uint8_t PIN;
  uint8_t numberKeyPresses;
  bool pressed;
};
Button JoyButton = {21, 0, false};//se crea estructura Button en GPIO21
volatile int pressLast = 0;

void IRAM_ATTR isr() {

  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200)
  {
    JoyButton.numberKeyPresses = (JoyButton.numberKeyPresses + 1) % 10;

    if (JoyButton.numberKeyPresses % 2 == 0) {
      JoyButton.pressed = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
      JoyButton.pressed = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  last_interrupt_time = interrupt_time;

}

//###################OLED I2C######################################
#include "SSD1306Wire.h" //libreria de la OLED
#define SDA 4
#define SCL 15
#define RST 16
SSD1306Wire  display(0x3c, SDA, SCL);

//###################MPU60503###############################################################################
#include "MPU6050_tockn.h"//biblioteca para usar la IMU

MPU6050 imu(Wire);//se crea un objeto de la clase mpu6050
float Xoffset = 0, Yoffset = 0;
float anguloX = 0, anguloY = 0;
float errorX = 0, errorY = 0;
//##########################################################################################################

//###########Variables de la Cinematica Inversa#############################################################
int cabeceo, alabeo, elevacion; //variables de la pose deseada
String cabeceoStr, alabeoStr, elevacionStr; //variables de la pose en cadena String dado que se obtienen por Serial
robot_paralelo Robot(PINSERVO1, PINSERVO2, PINSERVO3);//se crea un objeto de la clase robot_paralelo
//##########################################################################################################

//###########Declaracion de funciones usadas################################################################
float mapeo(float x, float in_min, float in_max, float out_min, float out_max);
String leerSerial();
void dibujarOLED(float pitch, float roll, float heave);
void RobotModoOLED(String tipo);
void warningIMU(void);
//##########################################################################################################

void Trayectorias();

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(1);// 1 ms  de timeout para el puerto serial, es la demora que el puerto serial se queda esperando a recibir datos

  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH); // para activar la OLED
  delay(100);
  display.init();
  display.flipScreenVertically();


  pinMode(JoyButton.PIN, INPUT_PULLUP);//se declara el boton del joystick como entrada con resistencia pull-up interna activada
  //attachInterruptArg(JoyButton.PIN, isr, &JoyButton, FALLING);//se declara interrupcion sobre el pin JoyButton, se ejecutara rutina isr cuando la seÃ±al caiga -|_
  attachInterrupt(JoyButton.PIN, isr, FALLING);
  pinMode(LED_BUILTIN, OUTPUT); //se usa para fines de debbugin, si se enciende indica mando manual por joystick y potenciometro


  analogReadResolution(9);//puede cambiarse a 9,10,11 o 12 bits, para cambiar la resolucion de los conversores ADC
  //los conversores ADC poseen una atenuacion de 11dB por defecto, lo que le permite poder leer voltajes de 0 a 3.3V



  //  warningIMU();//se alerta por la OLED de no mover el robot  y se efectua la calibracion de la IMU

  Robot.setServosPos(90, 90, 90);
  RobotModoOLED("Modo Automático");
  dibujarOLED(0.0, 0.0, 135.0);

}

void loop() {

  //###########################MODO AUTOMATICO############################################################################

  while (!(JoyButton.pressed)) {//segun la isr esta bandera cambia de estado cada vez q se presione el boton del joystick

    Serial.println("Introduzca el angulo de cabeceo deseado (20 a -20 grados)...");//rotacion sobre eje Y, pitch
    cabeceoStr = leerSerial();//cabeceo = GetSerialFloat();
    Serial.println("Introduzca el angulo de alabeo deseado (20 a -20 grados)...");//rotacion sobre eje X, roll
    alabeoStr = leerSerial();
    Serial.println("Introduzca la elevacion (100 a 135mm)...");//desplazamiento sobre Z, heave
    elevacionStr = leerSerial();

    cabeceo = cabeceoStr.toFloat();//se convierten los datos a flotantes atof(charArray)
    alabeo = alabeoStr.toFloat();//******No esta funcionando la conversion, se quedan en enteros
    elevacion = elevacionStr.toFloat();

    RobotModoOLED("Modo Automático");
    dibujarOLED(cabeceo, alabeo, elevacion);
    Robot.setPose(cabeceo, alabeo, elevacion);
    delay(1);
  }

  //###############################MODO MANUAL################################################################################################

  JoyCabeceo = analogRead(PINJOY_CABE);
  JoyAlabeo = analogRead(PINJOY_ALAB);

  JoyCabeceo = CabeceoOffset - JoyCabeceo;//(CabeceoOffset -511) a CabeceoOffset
  JoyAlabeo = AlabeoOffset  - JoyAlabeo; //(AlabeoOffset -511) a AlabeoOffset

  float cabOFF = mapeo(JoyCabeceo, (CabeceoOffset - 511), CabeceoOffset, 30, -30); //en posicion 0;0 quedan estos valores residuales offset
  float alabOFF = mapeo(JoyAlabeo, (AlabeoOffset - 511), AlabeoOffset, 30, -30);

  warningIMU();//se alerta por la OLED de no mover el robot  y se efectua la calibracion de la IMU
  Serial.print("CabDeseado"); Serial.print(" ; ");
  Serial.print("CabReal"); Serial.print(" ; ");
  Serial.print("AlabDeseado"); Serial.print(" ; ");
  Serial.print("AlabReal"); Serial.print(" ; ");
  Serial.print("Elevacion"); Serial.print(" ; ");
  Serial.print("Servo1"); Serial.print(" ; ");
  Serial.print("Servo2"); Serial.print(" ; ");
  Serial.println("Servo3");

  while (JoyButton.pressed) {//segun la isr esta bandera cambia de estado cada vez q se presione el boton del joystick

    //con 9bits de resolucion las lecturas del ADC estan entre 0-511
    JoyCabeceo = analogRead(PINJOY_CABE);//centrado en 246
    JoyAlabeo = analogRead(PINJOY_ALAB); //centrado en 239
    JoyElevacion = analogRead(PINJOY_ELEV); //va entre el maximo y el minimo (0-511)

    //se centran en cero
    JoyCabeceo = CabeceoOffset - JoyCabeceo;//(CabeceoOffset -511) a CabeceoOffset
    JoyAlabeo = AlabeoOffset  - JoyAlabeo; //(AlabeoOffset -511) a AlabeoOffset

    JoyCabeceo = mapeo(JoyCabeceo, (CabeceoOffset - 511), CabeceoOffset, 30, -30);
    JoyAlabeo = mapeo(JoyAlabeo, (AlabeoOffset - 511), AlabeoOffset, 30, -30);
    JoyElevacion = mapeo(JoyElevacion, 0, 511, 100, 138);

    JoyCabeceo -= cabOFF;//se corrige el error de posicionamiento dado por el offset
    JoyAlabeo -= alabOFF;

    if (JoyCabeceo > 25) {
      JoyCabeceo = 25; //para limitar valores
    }
    if (JoyCabeceo < -25) {
      JoyCabeceo = -25;
    }
    if (JoyAlabeo > 25) {
      JoyAlabeo = 25;
    }
    if (JoyAlabeo < -25) {
      JoyAlabeo = -25;
    }

    Robot.setPose(JoyCabeceo, JoyAlabeo, JoyElevacion);
    imu.update();
    //anguloX = Xoffset - imu.getAngleX();
    //anguloY = Yoffset - imu.getAngleY();
    anguloX = imu.getGyroAngleX();//getGyroX()
    anguloY = imu.getGyroAngleY();

    Serial.print(JoyCabeceo, 4); Serial.print(" ; ");
    Serial.print(anguloX, 4); Serial.print(" ; ");
    Serial.print(JoyAlabeo, 4); Serial.print(" ; ");
    Serial.print(anguloY, 4); Serial.print(" ; ");
    Serial.print(JoyElevacion, 4); Serial.print(" ; ");
    Serial.print(Robot.posGrados1, 4); Serial.print(" ; ");
    Serial.print(Robot.posGrados2, 4); Serial.print(" ; ");
    Serial.println(Robot.posGrados3, 4);

    //Metodo para refrescar la pantalla OLED a una frecuencia diferente a la del cambio de los datos
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= 200) {// la OLED va a actualizarse cada 500ms
      previousMillis = currentMillis;// se salva la ultima vez

      RobotModoOLED("Modo Manual");
      dibujarOLED(JoyCabeceo, JoyAlabeo, JoyElevacion);
    }


    delay(1);
  }
  //##################TRAYECTORIAS PREPROGRAMADAS########################################


  warningIMU();//se alerta por la OLED de no mover el robot  y se efectua la calibracion de la IMU
  RobotModoOLED("Trayectorias");
  dibujarOLED(0, 0, 120);

  while (!(JoyButton.pressed)) {

    Trayectorias();
  }

  Robot.setServosPos(45, 45, 45); //se posicionan todos los servos en 45 grados
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Presione el botón para"); //"Por favor no mueva el robot"
  display.drawString(0, 10, "volver al modo automático");
  display.display();

  while (JoyButton.pressed) {
    delay(100);//ciclo infinito esperando a que se presione nuevamente el boton del joystick
  }

  RobotModoOLED("Modo Automático");
  dibujarOLED(0, 0, 120);
  Robot.setServosPos(45, 45, 45); //se posicionan todos los servos en 45 grados

}


void Trayectorias() {

  //###########################SENO SIN  AMORTIGUACION POR CABECEO####################

  JoyAlabeo = 0;
  JoyElevacion = 120;

  for (float i = 0; i < 40 * 3.14159265; i += ((float)3.14159265) / 30) {
    if (!(JoyButton.pressed)) {
      //    delay(15);
      JoyCabeceo = ((float) 15 * sin(0.159 * i));//w=f/2pi 1Hz = 0.159rad/s
      RobotModoOLED("Trayectorias");
      dibujarOLED(JoyCabeceo, JoyAlabeo, JoyElevacion);
      Robot.setPose(JoyCabeceo, JoyAlabeo, JoyElevacion);

      imu.update();
      //anguloX = Xoffset - imu.getAngleX();
      //anguloY = Yoffset - imu.getAngleY();
      anguloX = imu.getGyroAngleX();//getGyroX()
      anguloY = imu.getGyroAngleY();
      Serial.print(JoyCabeceo);
      Serial.print(" ; ");
      Serial.print(anguloX);//imu.getAngleX()
      Serial.print(" ; ");
      Serial.print(JoyAlabeo);
      Serial.print(" ; ");
      Serial.print(anguloY);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados1, 4);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados2, 4);
      Serial.print(" ; ");
      Serial.println(Robot.posGrados3, 4);
    } else {
      return;
    }
  }

  for (int i = 0; i < 10 * 3; i++) {
    if (!(JoyButton.pressed)) {
      imu.update();
      delay(15);
      anguloX = imu.getGyroAngleX();//getGyroX()
      anguloY = imu.getGyroAngleY();
      Serial.print(JoyCabeceo);
      Serial.print(" ; ");
      Serial.print(anguloX);//imu.getAngleX()
      Serial.print(" ; ");
      Serial.print(JoyAlabeo);
      Serial.print(" ; ");
      Serial.print(anguloY);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados1, 4);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados2, 4);
      Serial.print(" ; ");
      Serial.println(Robot.posGrados3, 4);

    } else return;
    delay(15);
  }
  //##################################################################################

  //#######################seno sin amortiguacion por alabeo##########################
  JoyCabeceo = 0;
  JoyElevacion = 120;
  for (float i = 0; i < 10 * 3.14159265; i += ((float)3.14159265) / 30) {
    delay(15);
    if (!(JoyButton.pressed)) {
      JoyAlabeo = ((float) 15 * sin(0.8 * i));
      RobotModoOLED("Trayectorias");
      dibujarOLED(JoyCabeceo, JoyAlabeo, JoyElevacion);
      Robot.setPose(JoyCabeceo, JoyAlabeo, JoyElevacion);

      //    delay(150);
      imu.update();
      anguloX = imu.getGyroAngleX();//getGyroX()
      anguloY = imu.getGyroAngleY();

      Serial.print(JoyCabeceo);
      Serial.print(" ; ");
      Serial.print(anguloX);
      Serial.print(" ; ");
      Serial.print(JoyAlabeo);
      Serial.print(" ; ");
      Serial.print(anguloY);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados1, 4);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados2, 4);
      Serial.print(" ; ");
      Serial.println(Robot.posGrados3, 4);

    } else return;
  }
  for (int i = 0; i < 10 * 3; i++) {
    imu.update();
    delay(15);
    anguloX = imu.getGyroAngleX();//getGyroX()
    anguloY = imu.getGyroAngleY();
    if (!(JoyButton.pressed)) {
      Serial.print(JoyCabeceo);
      Serial.print(" ; ");
      Serial.print(anguloX);
      Serial.print(" ; ");
      Serial.print(JoyAlabeo);
      Serial.print(" ; ");
      Serial.print(anguloY);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados1, 4);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados2, 4);
      Serial.print(" ; ");
      Serial.println(Robot.posGrados3, 4);
    } else return;
    delay(15);
  }
  //#######################seno sin amortiguacion por elevacion##########################
  JoyCabeceo = 0;
  JoyAlabeo = 0;
  for (float i = 0; i < 10 * 3.14159265; i += ((float)3.14159265) / 30) {
    delay(15);
    if (!(JoyButton.pressed)) {
      JoyElevacion = (115 +(float) 10 * sin(0.8 * i));
      RobotModoOLED("Trayectorias");
      dibujarOLED(JoyCabeceo, JoyAlabeo, JoyElevacion);
      Robot.setPose(JoyCabeceo, JoyAlabeo, JoyElevacion);

      //    delay(150);
      imu.update();
      anguloX = imu.getGyroAngleX();//getGyroX()
      anguloY = imu.getGyroAngleY();

      Serial.print(JoyCabeceo);
      Serial.print(" ; ");
      Serial.print(anguloX);
      Serial.print(" ; ");
      Serial.print(JoyAlabeo);
      Serial.print(" ; ");
      Serial.print(anguloY);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados1, 4);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados2, 4);
      Serial.print(" ; ");
      Serial.println(Robot.posGrados3, 4);

    } else return;
  }
  for (int i = 0; i < 10 * 3; i++) {
    imu.update();
    delay(15);
    anguloX = imu.getGyroAngleX();//getGyroX()
    anguloY = imu.getGyroAngleY();
    if (!(JoyButton.pressed)) {
      Serial.print(JoyCabeceo);
      Serial.print(" ; ");
      Serial.print(anguloX);
      Serial.print(" ; ");
      Serial.print(JoyAlabeo);
      Serial.print(" ; ");
      Serial.print(anguloY);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados1, 4);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados2, 4);
      Serial.print(" ; ");
      Serial.println(Robot.posGrados3, 4);
    } else return;
    delay(15);
  }
  //####################seno con amortiguacion por cabeceo############################
  JoyAlabeo = 0;
  JoyElevacion = 120;
  for (float i = 0; i < 20 * 3.14159265; i += ((float)3.14159265) / 30) {
    delay(15);
    if (!(JoyButton.pressed)) {
      JoyCabeceo = ((float) 30 * pow(2, -0.03 * i) * sin(0.4 * i));
      RobotModoOLED("Trayectorias");
      dibujarOLED(JoyCabeceo, JoyAlabeo, JoyElevacion);
      Robot.setPose(JoyCabeceo, JoyAlabeo, JoyElevacion);

      imu.update();
      anguloX = imu.getGyroAngleX();
      anguloY = imu.getGyroAngleY();
      Serial.print(JoyCabeceo);
      Serial.print(" ; ");
      Serial.print(anguloX);//imu.getAngleX()
      Serial.print(" ; ");
      Serial.print(JoyAlabeo);
      Serial.print(" ; ");
      Serial.print(anguloY);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados1, 4);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados2, 4);
      Serial.print(" ; ");
      Serial.println(Robot.posGrados3, 4);
    } else return;
  }

  for (int i = 0; i < 10 * 3; i++) {
    if (!(JoyButton.pressed)) {
      imu.update();
      delay(15);
      anguloX = imu.getGyroAngleX();//getGyroX()
      anguloY = imu.getGyroAngleY();
      Serial.print(JoyCabeceo);
      Serial.print(" ; ");
      Serial.print(anguloX);//imu.getAngleX()
      Serial.print(" ; ");
      Serial.print(JoyAlabeo);
      Serial.print(" ; ");
      Serial.print(anguloY);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados1, 4);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados2, 4);
      Serial.print(" ; ");
      Serial.println(Robot.posGrados3, 4);
    } else return;
    delay(15);
  }
  //###################seno con amortiguacion por alabeo##############################
  JoyCabeceo = 0;
  JoyElevacion = 120;
  for (float i = 0; i < 20 * 3.14159265; i += ((float)3.14159265) / 30) {
    delay(15);
    if (!(JoyButton.pressed)) {
      JoyAlabeo = ((float) 30 * pow(2, -0.03 * i) * sin(0.4 * i));
      RobotModoOLED("Trayectorias");
      dibujarOLED(JoyCabeceo, JoyAlabeo, JoyElevacion);
      Robot.setPose(JoyCabeceo, JoyAlabeo, JoyElevacion);

      imu.update();
      anguloX = imu.getGyroAngleX();
      anguloY = imu.getGyroAngleY();
      Serial.print(JoyCabeceo);
      Serial.print(" ; ");
      Serial.print(anguloX);//imu.getAngleX()
      Serial.print(" ; ");
      Serial.print(JoyAlabeo);
      Serial.print(" ; ");
      Serial.print(anguloY);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados1, 4);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados2, 4);
      Serial.print(" ; ");
      Serial.println(Robot.posGrados3, 4);
    } else return;
  }
  for (int i = 0; i < 10 * 3; i++) {
    if (!(JoyButton.pressed)) {
      imu.update();
      delay(15);
      anguloX = imu.getGyroAngleX();//getGyroX()
      anguloY = imu.getGyroAngleY();
      Serial.print(JoyCabeceo);
      Serial.print(" ; ");
      Serial.print(anguloX);//imu.getAngleX()
      Serial.print(" ; ");
      Serial.print(JoyAlabeo);
      Serial.print(" ; ");
      Serial.print(anguloY);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados1, 4);
      Serial.print(" ; ");
      Serial.print(Robot.posGrados2, 4);
      Serial.print(" ; ");
      Serial.println(Robot.posGrados3, 4);

    } else return;
    delay(15);
  }
  //##################################################################################
}


void RobotModoOLED(String modo) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, modo);

}
void warningIMU(void) {
  Robot.setServosPos(45, 45, 45); //se posicionan todos los servos en 45 grados
  display.clear();
  delay(500);//para eliminar efectos de la inercia en la calibracion de la IMU

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Inicializando IMU");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 16, "Por favor no mueva el robot");
  display.drawString(0, 26, "Proceso de calibracion");
  display.drawString(0, 36, "de senores en la IMU...");

  display.drawProgressBar(10, 50, 100, 10, 0);
  display.display();

  imu.begin();
  imu.calcGyroOffsets(false);
  delay(250);
  display.drawProgressBar(10, 50, 100, 10, 15);
  display.display();
  delay(250);

  //  float X = 0, Y = 0;
  //  for (int i = 0; i < 3000; i++) {
  //    X += imu.getAngleX();//se suman 3000 muestras
  //    Y += imu.getAngleY();
  //  }

  for (int i = 0; i < 8500; i++) {
    display.drawProgressBar(10, 50, 100, 10, 15 + (i / 100));
    display.display();
  }
  //  Xoffset = ((float)X) / 3000; //se halla el promedio de las lecturas para X e Y
  //  Yoffset = ((float)Y) / 3000;
  //  delay(1000);

}
void dibujarOLED(float pitch, float roll, float heave) {

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(3, 29, "Pitch:");
  display.drawString(3, 39, "Roll:");
  display.drawString(3, 49, "Heave:");

  display.drawString(35, 29, String(pitch));
  display.drawString(35, 39, String(roll));
  display.drawString(35, 49, String(heave, 1));

  display.drawRect(1, 29, 63, 33);
  display.drawHorizontalLine(70, 30, 40);//pitch
  display.drawVerticalLine(115, 20, 40);//roll

  pitch = mapeo(pitch, MINPITCH, MAXPITCH, 0, 40);
  display.drawVerticalLine(70 + pitch, 27, 7);

  roll = mapeo(roll, MINROLL, MAXROLL, 0, 40);
  display.drawHorizontalLine(112, 60 - roll, 7);
  heave = mapeo(heave, MINHEAVE, MAXHEAVE, 0, 100);

  display.drawProgressBar(70, 52, 40, 8, heave);//heave
  display.display();

}

String leerSerial() {
CICLO_LEER:
  if (JoyButton.pressed) {
    return "0";//cancela los comandos por serial para entrar en modo Mando Joystick+potenciometro
  }
  if (Serial.available() > 0) {
    return Serial.readString();//Serial.parseFloat();
  }
  else
    goto CICLO_LEER;

}

float mapeo(float x, float in_min, float in_max, float out_min, float out_max)//funcion reimplementada de map, para poder obtener puntos flotantes
{
  return (float)(x - in_min) * (float)(out_max - out_min) / (float)(in_max - in_min) + (float)out_min;
}
