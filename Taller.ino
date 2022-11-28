

/////////////////// Definiciones previas ///////////////////
#include <Matrices.h>
#include <math.h>

/////////////////// Puertos ///////////////////
#define Conmutador_Maestro 2
#define Valvula_Manual 3
#define Sensor_20 4
//#define Sensor_40 5
//#define Sensor_60 6
#define Sensor_80 5
//#define Sensor_100 


#define ELECTROVALVULA 8
#define INDICACION 9
#define ALERTA 10

/////////////////// Variables Globales ///////////////////
int state = 0;
int ecuacion =0;

double A[3][3]={};
double P[3][1]={};

double S[5]={5,10,18,20,24};
double W[5]={0.5,1.2,2,2.8,3.0};

//////////////////////////////////////////////////////////////



/////////////////// Funcion Especial - Regresion C ///////////////////
void RegresionCuadratica(double x[], double y[], double n){
  double v1=0;
  double v2=0;
  double v3=0;

  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      rMultiplicacion[i][j]=0;
    }
  }

  double Xi=0;
  double Xi2=0;
  double Xi3=0;
  double Xi4=0;
  
  double Yi=0;
  double XiYi=0;
  double Xi2Yi=0;
  
  for(int i=0; i<n; i++)
    {
      Xi += x[i];
      Xi2 += pow(x[i], 2);
      Xi3 += pow(x[i], 3);
      Xi4 += pow(x[i], 4);

      Yi += y[i];
      XiYi += x[i]*y[i];
      Xi2Yi += pow(x[i], 2)*y[i];
    }

  A[0][0]=n;
  A[0][1]=Xi;
  A[0][2]=Xi2;

  A[1][0]=Xi;
  A[1][1]=Xi2;
  A[1][2]=Xi3;
  
  A[2][0]=Xi2;
  A[2][1]=Xi3;
  A[2][2]=Xi4;

  P[0][0]=Yi;
  P[1][0]=XiYi;
  P[2][0]=Xi2Yi;

  MatInversa(A);
  MatMultiplicacion(rInversa, P);
  
  v1 = rMultiplicacion[0][0];
  v2 = rMultiplicacion[1][0];
  v3 = rMultiplicacion[2][0];

  //Serial.print("La ecuación de la regresión cuadrática es la siguiente:\n y = %f X^2 + %f X + %f", v3, v2, v1);
  Serial.print("La ecuacion de la regresion cuadratica es la siguiente: "); 
  Serial.print(v3, 8); Serial.print(" , "); Serial.print(v2, 8); Serial.print(" , "); Serial.println(v1, 8);
}

bool dW_dt(){
 // Funcion para sacar el Diferencial de peso con respecto del diferencial de tiempo
}

bool Nivel_Estimado(){
  //AQUI SE VA A PONER LA ECUACION DE LA REGRESION CUADRATICA
  if(digitalRead(6)==1){
    return true;
  }else{
    return false;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(Conmutador_Maestro, INPUT);
  pinMode(Valvula_Manual, INPUT);
  pinMode(Sensor_20, INPUT);
  //pinMode(Sensor_40, INPUT);
  //pinMode(Sensor_60, INPUT);
  pinMode(Sensor_80, INPUT);
  //pinMode(Sensor_100, INPUT);

  pinMode(ELECTROVALVULA, OUTPUT);
  pinMode(INDICACION, OUTPUT);
  pinMode(ALERTA, OUTPUT);
}

void loop() {

  int CM = digitalRead(Conmutador_Maestro);
  int VM = digitalRead(Valvula_Manual);
  int S20 = digitalRead(Sensor_20);
  int S80 = digitalRead(Sensor_80);
  //int S100 = digitalRead(Sensor_100);

  Serial.print("Estado: "); Serial.print(state); Serial.print(" ");
  Serial.print("CM: "); Serial.print(CM); Serial.print(" ");
  Serial.print("VM: "); Serial.print(VM); Serial.print(" ");
  Serial.print("S20: "); Serial.print(S20); Serial.print(" ");
  Serial.print("S80: "); Serial.print(S80); Serial.print(" ");
  Serial.print("N: "); Serial.print(digitalRead(6)); Serial.println(" ");
  
  if(CM==0 && VM==1 && S20==0 && S80==0 && Nivel_Estimado()==false){
    //NINGUNA SALIDA 
    state = 0;
    Serial.println("Estado 0");
    delay(500);

  }

  if(state == 0 && CM==1 && VM==1 && S20==0 && S80==0){
    //NINGUNA SALIDA
    state = 1;
    Serial.println("Estado 1");
    delay(500);
  }

  if(state == 1 && CM==1 && VM==0 && S20==0 && S80==0){
    //ACTIVA ELECTROVALVULA
    state = 2;
    Serial.println("Estado 2");
    delay(500);
  }

  if(state == 2 && CM==1 && VM==0 && S20==1 && S80==0){
    //ACTIVA ELECTROVALVULA
    state = 3;
    Serial.println("Estado 3");
    delay(500);
  }

  if(state == 3 && CM==1 && VM==0 && S20==1 && S80==1){
    //DESACTIVA ELECTRO VALVULA, INDICACION VISUAL
    state = 4;
    Serial.println("Estado 4");
    delay(500);
  }



///////////////////////////////////////  ESTADO 4  80% ///////////////////////////////////////////////7
  if(state == 4 && CM==1 && VM==1 && S20==1 && S80==1 && Nivel_Estimado()==false){
    //INDICACION VISUAL
    state = 7;
    Serial.println("Estado 7");
    delay(500);
  }

  if(state == 4 && CM==1 && VM==0 && S20==1 && S80==1 && Nivel_Estimado()==true){
    // EXISTE UN ERROR Y LA ELECTROVALVULA SIGUE PASANDO AGUA, SE DEBE DE GENERAR UNA ALERTA
    state = 6;
    Serial.println("Estado 6");
    delay(500);
  }
/////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////  ESTADO 7 ///////////////////////////////////////////////
  if(state == 7 && CM==1 && VM==1 && S20==0 && S80==0 && Nivel_Estimado()==false){
    //INDICACION VISUAL
    state = 5;
    Serial.println("Estado 5");
    delay(500);
  }

  if(state == 7 && CM==0 && VM==1 && S20==0 && S80==0 && Nivel_Estimado()==false){
    state = 0;
    Serial.println("Estado 0");
    delay(500);
  } 
/////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////  ESTADO 5 ///////////////////////////////////////////////
  if(state == 5 && CM==1 && VM==0 && S20==0 && S80==0 && Nivel_Estimado()==false){
    //ELECTROVALVULA
    state = 2;
    Serial.println("Estado 2");
    delay(500);
  }

  if(state == 1 && CM==0 && VM==1 && S20==0 && S80==0 && Nivel_Estimado()==false){
    state = 0;
    Serial.println("Estado 0");
    delay(500);
  }
/////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////  ESTADO 6 ///////////////////////////////////////////////

  if(state == 6 && CM==1 && VM==1 && S20==1 && S80==1 && Nivel_Estimado()==true){
    //indicacion visual
    state = 7;
    Serial.println("Estado 7");
    delay(500);
  }
//////////////////////////////////////////////////////////////////////////////////////////////////





///////////////////////////////////////// SALIDAS //////////////////////////////////////
  switch(state){
  case 0:
    digitalWrite(ELECTROVALVULA, LOW);
    digitalWrite(INDICACION, LOW);
    digitalWrite(ALERTA, LOW);
    Serial.println("Caso 0");
    delay(500);
    break;

  case 1:
    digitalWrite(ELECTROVALVULA, 0);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 0);

    Serial.println("Caso 1");
    delay(500);
    //Serial.println(state);
    break;

  case 2:
    digitalWrite(ELECTROVALVULA, 1);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 0);
    Serial.println("Caso 2");
    delay(500);
    break;

  case 3:
    digitalWrite(ELECTROVALVULA, 1);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 0);
    Serial.println("Caso 3");
    delay(500);
    break;

  case 4:
    digitalWrite(ELECTROVALVULA, 0);
    digitalWrite(INDICACION, 1);
    digitalWrite(ALERTA, 0);
    Serial.println("Caso 4");
    delay(500);
    break;

  case 5:
    digitalWrite(ELECTROVALVULA, 0);
    digitalWrite(INDICACION, 1);
    digitalWrite(ALERTA, 0);
    Serial.println("Caso 5");
    delay(500);
    break;

  case 6:
    digitalWrite(ELECTROVALVULA, 0);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 1);
    Serial.println("Caso 6");
    delay(500);
    break;

  case 7:
    digitalWrite(ELECTROVALVULA, 0);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 0);
    Serial.println("Caso 7");
    delay(500);
    break;


 }
}
