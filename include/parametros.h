#ifndef PARAMETROS
#define PARAMETROS
//Este archivo contiene todas las constantes del robot

//constantes matematicas
#define pi  3.14159265

#define MAXHEAVE 138 //máxima elevación en miletros
#define MINHEAVE 100 
#define MAXPITCH 30 
#define MAXROLL 30
#define MINPITCH -30
#define MINROLL -30

//vectores constantes de la estructura, medidas en metros
// __1__Vectores de la base, ubicación de los servos
static float B_1[3]={0,-0.043,0};
static float B_2[3]={0.037239,0.0215,0};//[cos(30)*0.043;sin(30)*0.043;0]
static float B_3[3]={-0.037239,0.0215,0};//B3 = [-cos(pi/6)*norm(B1);sin(pi/6)*norm(B1);0]

// __2__Vectores de la plataforma, están en base al origen de la propia plataforma y por eso son iguales a B1, B2 y B3
static float P_1[3]={0,-0.043,0};
static float P_2[3]={0.037239,0.0215,0};
static float P_3[3]={-0.037239,0.0215,0};



static float C = 0.042;//longitud del primer eslabón en metros
static float D = 0.097;//longitud del segundo eslabón en metros



#endif // PARAMETROS
