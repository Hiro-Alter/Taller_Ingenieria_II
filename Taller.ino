/////////////////// Definiciones previas ///////////////////
#include <Matrices.h>
#include <math.h>
#include <HX711.h>

// CLASES //
HX711 bascula;

/////////////////// Puertos ///////////////////
#define Conmutador_Maestro 15
#define Valvula_Manual 16
#define Sensor_20 17
//#define Sensor_40 5
//#define Sensor_60 6
#define Sensor_80 18
//#define Sensor_100

#define ELECTROVALVULA 6
#define INDICACION 7
#define ALERTA 8

#define BASCULA_DT 2
#define BASCULA_SCLK 3

/////////////////// Variables Globales ///////////////////
int state = 0;

double v1=0;
double v2=0;
double v3=0;

double A[3][3]={};
double P[3][1]={};

double Sensores[6]={0,0.034,0.068,0.102,0.136,0.17};
double Peso_Sensores[6]={0,0.8,1.5,2.3,3.0,3.9};

float peso_actual=0;
float peso_anterior=0;

float volumen_actual=0;
float volumen_anterior=0;

float volumen_lleno=0;
float volumen_vacio=0;

float volumen_ciclo=0;
float volumen_total=0;

int contador=0;

float tiempo_actual=0;
float tiempo_anterior=0;

float dwdt=0;
float dvdt=0;
float dwdt_inicial=0;


// VARIABLES DE BASCULA //
float factor_calibracion = -20780.0;

//////////////////////////////////////////////////////////////



/////////////////// Funcion Especial - Regresion C ///////////////////
void RegresionCuadratica(double x[], double y[], double n){
  v1=0;
  v2=0;
  v3=0;

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
  float x=0.01*dwdt_inicial;
  if(dwdt<x){
    return true;
  }else{
    return false;
  }
}

bool Nivel_Estimado(){

  RegresionCuadratica(Sensores,Peso_Sensores,6);
  double ecuacion_nivel = v1 + v2*peso_actual + v3*pow(peso_actual,2); 
  /*
  if(ecuacion_nivel>0.9){
    return true;
  }else{
    return false;
  }
*/
  if(digitalRead(A5)==1){
    return true;
  }else{
    return false;
  }
}

void setup() {
  Serial.begin(4800);
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


  //CONFIGURACIÓN BASCULA 
  bascula.begin(BASCULA_DT, BASCULA_SCLK);
  bascula.tare();
  long zero_factor = bascula.read_average();
  //
  bascula.set_scale(factor_calibracion);
  //Funcion para obtener el peso//
  
}

void loop() {

  delay(500);
  
  peso_actual=max(bascula.get_units(),0); // TENER ENCUENTA A LA HORA DE CALIBRAR AL CELDA!!!!!!!
  volumen_actual=((peso_actual/9.8)/997); // Densidad del Agua en litros
  tiempo_actual=(millis()/1000);

  dwdt = (peso_actual-peso_anterior)/((tiempo_actual-tiempo_anterior));
  dvdt = ((volumen_actual-volumen_anterior) / ((tiempo_actual-tiempo_anterior)));

  int CM = digitalRead(Conmutador_Maestro);
  int VM = digitalRead(Valvula_Manual);
  int S20 = digitalRead(Sensor_20);
  int S80 = digitalRead(Sensor_80);
  
  if(CM==0 && VM==1 && S20==0 && S80==0){
    //NINGUNA SALIDA 
    state = 0;
  }

  if(state == 0 && CM==1 && VM==1 && S20==0 && S80==0){
    //NINGUNA SALIDA
    state = 1;
  }

  if(state == 1 && CM==1 && VM==0 && S20==0 && S80==0){
    //ACTIVA ELECTROVALVULA
    state = 2;
  }

  if(state == 2 && CM==1 && VM==0 && S20==1 && S80==0){
    //ACTIVA ELECTROVALVULA
    state = 3;
  }

  if(state == 3 && CM==1 && VM==0 && S20==1 && S80==1){
    //DESACTIVA ELECTRO VALVULA, INDICACION VISUAL
    //peso_anterior=peso;
    state = 4;
    dwdt_inicial=dwdt;
  }


///////////////////////////////////////  ESTADO 4  80% ///////////////////////////////////////////////7
  if(state == 4 && CM==1 && VM==1 && S20==1 && S80==1){
    //APAGA INDICACION VISUAL HASTA QUE SE TERMINE DE LLENAR EL TANQUE
    //variacion_peso_inicial = ((peso_actual-peso_anterior)/1);
    state = 7;
  }

  if(state == 4 && CM==1 && VM==0 && S20==1 && S80==1 && Nivel_Estimado()==true){
    // EXISTE UN ERROR Y LA ELECTROVALVULA SIGUE PASANDO AGUA, SE DEBE DE GENERAR UNA ALERTA
    state = 6;
  }
/////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////  ESTADO 5 ///////////////////////////////////////////////
  if(state == 5 && CM==1 && VM==1 && S20==0 && S80==0 && dW_dt()==true){
    //CIERRE LA VALVULA YA INDICACION INTERMITENTE
    state = 8;
  }

  if(state == 1 && CM==0 && VM==1 && S20==0 && S80==0){
    state = 0;
  }
/////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////  ESTADO 6 ///////////////////////////////////////////////
  if(state == 6 && CM==1 && VM==1 && S20==1 && S80==1 && Nivel_Estimado()==true){
    //indicacion visual
    state = 7;
  }
//////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////  ESTADO 7 ///////////////////////////////////////////////
  if(state == 7 && CM==1 && VM==1 && S20==0 && S80==0){
    //INDICACION VISUAL
    state = 5;
  }

  if(state == 7 && CM==0 && VM==1 && S20==0 && S80==0){
    state = 0;
  } 
/////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////  ESTADO 8 ///////////////////////////////////////////////
if(state == 8 && CM==1 && VM==0 && S20==0 && S80==0){
    //ENCIENDA ELECTROVALVULA
    state = 2;
  }

if(state == 8 && CM==0 && VM==0 && S20==0 && S80==0){
    //APAGADO EL COMUTADOR MAESTRO, VAYA AL ESTADO 0
    state = 0;
  }
//////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////// SALIDAS //////////////////////////////////////
  switch(state){
  case 0:
    digitalWrite(ELECTROVALVULA, LOW);
    digitalWrite(INDICACION, LOW);
    digitalWrite(ALERTA, LOW);
    break;

  case 1:
    digitalWrite(ELECTROVALVULA, 0);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 0);
    break;

  case 2:
    digitalWrite(ELECTROVALVULA, 1);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 0);
    break;

  case 3:
    digitalWrite(ELECTROVALVULA, 1);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 0);
    break;

  case 4:
    digitalWrite(ELECTROVALVULA, 0);
    digitalWrite(INDICACION, 1);
    digitalWrite(ALERTA, 0);
    break;

  case 5:
    digitalWrite(ELECTROVALVULA, 0);
    digitalWrite(INDICACION, 1);
    digitalWrite(ALERTA, 0);
    break;

  case 6:
    digitalWrite(ELECTROVALVULA, 0);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 1);
    break;

  case 7:
    digitalWrite(ELECTROVALVULA, 0);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 0);

    delay(100); //MEDIO SEGUNDO PARA ESPERAR EL AUMENTO DEL DW/DT Y QUE LA VARIABLE NO SEA 0
    
    while(contador<1){ //WHILE PARA TOMAR LOS DATOS SOLO 1 VEZ
      dwdt_inicial=dwdt;
      volumen_lleno=volumen_actual;
      Serial.print("dwdt_inicial: ");Serial.println(dwdt_inicial);
      contador++;
    }
    break;

  case 8:
    digitalWrite(ELECTROVALVULA, 0);
    digitalWrite(INDICACION, 1);
    delay(400);
    digitalWrite(INDICACION, 0);
    digitalWrite(ALERTA, 0);

    volumen_ciclo=(volumen_lleno-volumen_vacio);
    volumen_total+=volumen_ciclo;
    break;
 }


  Serial.println(" ");
 
  peso_anterior=peso_actual;
  volumen_anterior=volumen_actual;
  tiempo_anterior=tiempo_actual;

  Serial.println("//////////////////// ENTRADAS ////////////////////");
  Serial.print("CM: "); Serial.print(CM); Serial.print("\t"); Serial.print("VM: "); Serial.print(VM); Serial.print("\t"); Serial.print("S20: "); Serial.print(S20); Serial.print("\t");
  Serial.print("S80: "); Serial.print(S80); Serial.print("\t"); Serial.print("Nivel: "); Serial.print(Nivel_Estimado()); Serial.print("  "); Serial.print("1%: "); Serial.print(dW_dt()); Serial.println("\t");
  Serial.println("//////////////////// VARIABLES ////////////////////");
  Serial.print("Peso: "); Serial.print(peso_actual); Serial.print("kg"); Serial.print("\t\t"); Serial.print("Peso Anterior: "); Serial.print(peso_anterior); Serial.println("kg");
  Serial.print("Volumen: "); Serial.print(volumen_actual,5); Serial.print("m3"); Serial.print("\t"); Serial.print("Volumen Anterior: "); Serial.print(volumen_anterior,5); Serial.println("m3");
  Serial.print("Tiempo: "); Serial.print(tiempo_actual); Serial.print("\t\t"); Serial.print("Tiempo Anterior: "); Serial.println(tiempo_anterior);
  
  Serial.print("dw/dt: "); Serial.print(dwdt); Serial.print(" kg/s"); Serial.print("\t"); Serial.print("dv/dt: "); Serial.print(dvdt); Serial.println(" m3/s");

  Serial.println("//////////////////// ESTADOS ////////////////////");
  Serial.print("Estado Actual: "); Serial.println(state);

}

