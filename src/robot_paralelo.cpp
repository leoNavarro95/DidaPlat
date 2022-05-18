#include "robot_paralelo.h"
#include <math.h>
#include "parametros.h"

robot_paralelo::robot_paralelo(uint8_t PinServo1,uint8_t PinServo2,uint8_t PinServo3)
{
  
  Servo1.attach(PinServo1);
  Servo2.attach(PinServo2);
  Servo3.attach(PinServo3);
  
  pitch = 0; roll = 0; heave = 0;
  L1 = 0; L2 = 0; L3 = 0;
  alfa1 = 0; alfa2 = 0; alfa3 = 0;


}

void robot_paralelo::CalculaMRot(float phi, float cita, float heave)
{
  phi = phi * (float)(pi / 180); //convirtiendolos a radianes
  cita = cita * (float)(pi / 180);
  //primera fila
  MRot[0][0] = cos(cita);
  MRot[0][1] = sin(cita) * sin(phi);
  MRot[0][2] = sin(cita) * cos(phi);
  //Segunda fila
  MRot[1][0] = 0;
  MRot[1][1] = cos(phi);
  MRot[1][2] = -sin(phi);
  //Tercera fila
  MRot[2][0] = -sin(cita);
  MRot[2][1] = cos(cita) * sin(phi);
  MRot[2][2] = cos(cita) * cos(phi);

  T[0] = 0;
  T[1] = 0;
  T[2] = heave / 1000; //heave se entra en mm y aqui se convierte a metros

}

void robot_paralelo::obtener_L()
{
  l1[0] = T[0] + (MRot[0][0] * P_1[0] + MRot[0][1] * P_1[1] + MRot[0][2] * P_1[2]) - B_1[0];
  l1[1] = T[1] + (MRot[1][0] * P_1[0] + MRot[1][1] * P_1[1] + MRot[1][2] * P_1[2]) - B_1[1];
  l1[2] = T[2] + (MRot[2][0] * P_1[0] + MRot[2][1] * P_1[1] + MRot[2][2] * P_1[2]) - B_1[2];

  l2[0] = T[0] + (MRot[0][0] * P_2[0] + MRot[0][1] * P_2[1] + MRot[0][2] * P_2[2]) - B_2[0];
  l2[1] = T[1] + (MRot[1][0] * P_2[0] + MRot[1][1] * P_2[1] + MRot[1][2] * P_2[2]) - B_2[1];
  l2[2] = T[2] + (MRot[2][0] * P_2[0] + MRot[2][1] * P_2[1] + MRot[2][2] * P_2[2]) - B_2[2];

  l3[0] = T[0] + (MRot[0][0] * P_3[0] + MRot[0][1] * P_3[1] + MRot[0][2] * P_3[2]) - B_3[0];
  l3[1] = T[1] + (MRot[1][0] * P_3[0] + MRot[1][1] * P_3[1] + MRot[1][2] * P_3[2]) - B_3[1];
  l3[2] = T[2] + (MRot[2][0] * P_3[0] + MRot[2][1] * P_3[1] + MRot[2][2] * P_3[2]) - B_3[2];

  //RESTRICCIONES IMPUESTAS POR CADA ARTICULACION ROTACIONAL
  //cada restriccioin representa un plano por el que se mueve el vector 'l'
  l1[0] = 0;
  l2[0] = sqrt(3) * l2[1];
  l3[0] = -sqrt(3) * l3[1];

  //CAMBIANDO LOS SISTEMAS DE REFERENCIA PARA LOS VECTORES L2 y L3,
  //esto es con el objetivo de que el nuevo sist de ref
  //este orientado de tal forma q con respecto al plano de la restriccion
  //quede perpendicular el eje x
  l2[0] = -0.5 * l2[0] + (((float)sqrt(3)) / 2) * l2[1];
  l2[1] = -0.5 * l2[1] - (((float)sqrt(3)) / 2) * l2[0];

  l3[0] = -0.5 * l3[0] - (((float)sqrt(3)) / 2) * l3[1];
  l3[1] = -0.5 * l3[1] + (((float)sqrt(3)) / 2) * l3[0];

  //Estos vectores realmente no se emplean, pero se dejan para trabajo futuro
  L1 = sqrt(pow(l1[0], 2) + pow(l1[1], 2) + pow(l1[2], 2));
  L2 = sqrt(pow(l2[0], 2) + pow(l2[1], 2) + pow(l2[2], 2));
  L3 = sqrt(pow(l3[0], 2) + pow(l3[1], 2) + pow(l3[2], 2));

}

float robot_paralelo::getAlfa(float l[3])
{
  float K1 = 0, Dis1 = 0, Y11 = 0;

  K1 = (pow(l[1], 2) + pow(l[2], 2) + pow(C, 2) - pow(D, 2)) / (2 * l[2]);
  //Calculo del discriminante
  Dis1 = pow((2 * (K1 * l[1]) / l[2]), 2) - 4 * (1 + pow(l[1] / l[2], 2)) * (pow(K1, 2) - pow(C, 2));

  //Ojo ver como comprobar cuando el discriminante sea nulo, NULL

  if (Dis1 > 0) {
    Y11 = (-(2 * (K1 * l[1]) / l[2]) + sqrt(Dis1)) / (2 * (1 + pow(l[1] / l[2], 2))); //son las coordenadas en y del punto de intersepcion de las circunferencias formadas por C y D
    //Y12 = (-(2*(K1*l1[2])/l1[3]) - sqrt(Dis1))/(2*(1+pow(l1[2]/l1[3],2)));
    alfa1 = acos(Y11 / C);
    alfa1 = alfa1 * (180 / pi);
  } else alfa1 = -1; //indica que la pose deseada para este servo no tiene solucion

  return alfa1;
}


void robot_paralelo::setPose(float cabeceo, float alabeo, float elevacion) {

  this->CalculaMRot(cabeceo, alabeo, elevacion);
  this->obtener_L();
  
  posGrados1 = this->getAlfa(l1);
  posGrados2 = this->getAlfa(l2);
  posGrados3 = this->getAlfa(l3);

//  Serial.print("Alfa1: "); Serial.println(posGrados1, 4);
//  Serial.print("Alfa2: "); Serial.println(posGrados2, 4);
//  Serial.print("Alfa3: "); Serial.println(posGrados3, 4);

  //con esto se asegura de si no se obtuvo una soluciÃ³n para la cinematica inversa,(que es cuando getAlfa devuelve -1)
  //que el servo se quede como estaba ubicado anteriormente
  if ((posGrados1 == -1) || (posGrados2 == -1) || (posGrados3 == -1)) {
    posGrados1 = posGAnt1;
    posGrados2 = posGAnt2;
    posGrados3 = posGAnt3;
  }


  microSec1 = posGrados1 * 10.31111 + 544; //se convierten a microsegundos, esto es para aprovechar toda la resolucion
  microSec2 = posGrados2 * 10.31111 + 544;
  microSec3 = posGrados3 * 10.31111 + 544;

  Servo1.writeMicroseconds(microSec1);//se posicionan los servos segun la KI calculada
  Servo2.writeMicroseconds(microSec2);
  Servo3.writeMicroseconds(microSec3);

  posGAnt1 = posGrados1;//se salvan los valores de las posiciones angulares
  posGAnt2 = posGrados2;
  posGAnt3 = posGrados3;

}
void robot_paralelo::setServosPos(int angleServo1,int angleServo2,int angleServo3){
  Servo1.write(angleServo1);
  Servo2.write(angleServo2);
  Servo3.write(angleServo3);
  
  posGAnt1 = angleServo1;
  posGAnt2 = angleServo2;
  posGAnt3 = angleServo3;
}

robot_paralelo::~robot_paralelo()
{

}
