#ifndef ROBOT_PARALELO_H
#define ROBOT_PARALELO_H

#include <Servo.h>
class robot_paralelo
{
  public:
    robot_paralelo(uint8_t PinServo1,uint8_t PinServo2,uint8_t PinServo3);//constructor
    
    void CalculaMRot(float phi, float cita, float heave);//funcion que calcula la matriz de rotación le entran los ángulos de cita y phi

    void obtener_L();//se obtiene la longitud de los vectores L1,L2 y L3 los que definen la altura de la cadena cinematica actuada

    void setPose(float cabeceo, float alabeo, float elevacion);
    void setServosPos(int angleServo1,int angleServo2,int angleServo3);

    float getAlfa(float l[3]);//devuelve alfa1 y se le pasa la longitud L1
    //    float getAlfa2(float);//devuelve alfa2 y se le pasa la longitud L2
    //    float getAlfa3(float);//devuelve alfa3 y se le pasa la longitud L3

    float pitch, roll, heave;
    float L1, L2, L3; //modulos de los vectores l1,l2,l3
    float l1[3], l2[3], l3[3]; //vectores que van desde los servos hasta la última articulación
    float alfa1, alfa2, alfa3;
    //Declaración de la Matriz de rotación, esta se calculará en la clase CinematicaInversa
    float MRot[3][3];
    //Definición del vector de elevación
    float T[3];

    float posGrados1, posGrados2, posGrados3; //posicion angular de los servos
    float posGAnt1, posGAnt2, posGAnt3; //posicion anterior de los servos, esta variable se emplea para cuando la cinematica no tenga solucion se quede con la posicion angular anterior
    float microSec1, microSec2, microSec3; //esta es la seÃ±al que se le envÃ­a a los servos, es la duraciÃ³n en microsegundos de la seÃ±al PWM

    Servo Servo1;
    Servo Servo2;
    Servo Servo3;
    
    ~robot_paralelo();//destructor
};

#endif // ROBOT_PARALELO_H
