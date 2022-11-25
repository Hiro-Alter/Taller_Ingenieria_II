#include "Arduino.h"
#include "Wire.h"
#include "Matrices.h"

using namespace std;

double rSuma[3][3]={};
double rResta[3][3]={};
double rMultiplicacion[3][1]={};
double rMultX[3][3]={};
double rDivX[3][3]={};
double rAdj[3][3]={};
double rTranspuesta[3][3]={};
double rInversa[3][3]={};
double rIdentidad[3][3]={};

//***************************************************************************
//***************************************************************************
//****************************  FUNCTIONES    *******************************
//***************************************************************************
//***************************************************************************
void ImprimirMat(double A[3][3])
{
  for(int i=0; i < 3; i++)
  {
    for(int j=0; j<3; j++)
    {
      //cout<<A[i][j]<<"\t";
    }
    //cout<<endl;
  }
}

// 
double det(double A[3][3])
{
  double det=0;
  det = (A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1]))-(A[0][1]*(A[1][0]*A[2][2]-A[2][0]*A[1][2]))+
    (A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]));
  return det;
}


void MatSuma(double A[3][3], double B[3][3])
{
  for(int i=0; i < 3; i++)
    {
      for(int j=0; j<3; j++){
        rSuma[i][j] = A[i][j] + B[i][j];
      }
    }
}


void MatResta(double A[3][3], double B[3][3])
{
  for(int i=0; i < 3; i++)
    {
      for(int j=0; j<3; j++){
        rSuma[i][j] = A[i][j] - B[i][j];
      }
    }
}


void MatMultiplicacion(double A[3][3], double B[3][1])
{
  for (int i = 0; i < 3; i++) 
   {
        for (int j = 0; j < 1; j++) 
        {
            for (int k = 0; k < 3; k++) 
            {
                rMultiplicacion[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}


void MatMultEscalar(double Escalar, double A[3][3])
{
  for(int i=0; i<3; i++)
    {
      for(int j=0; j<3; j++)
        {
          rMultX[i][j] = A[i][j]*Escalar;
        }
    }
}


void MatDivEscalar(double Escalar, double A[3][3])
{
  for(int i=0; i<3; i++)
    {
      for(int j=0; j<3; j++)
        {
          rDivX[i][j] = A[i][j]/Escalar;
        }
    }
}


void MatTraspuesta(double A[3][3])
{
  for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rTranspuesta[i][j] = A[j][i];
        }
    }
}


void MatAdjunta(double A[3][3])
{
  double Cof[3][3]={};
  Cof[0][0]=(A[1][1]*A[2][2]-A[2][1]*A[1][2]);
  Cof[0][1]=-(A[1][0]*A[2][2]-A[2][0]*A[1][2]);
  Cof[0][2]=(A[1][0]*A[2][1]-A[2][0]*A[1][1]);

  Cof[1][0]=-(A[0][1]*A[2][2]-A[2][1]*A[0][2]);
  Cof[1][1]=(A[0][0]*A[2][2]-A[2][0]*A[0][2]);
  Cof[1][2]=-(A[0][0]*A[2][1]-A[2][0]*A[0][1]);

  Cof[2][0]=(A[0][1]*A[1][2]-A[1][1]*A[0][2]);
  Cof[2][1]=-(A[0][0]*A[1][2]-A[1][0]*A[0][2]);
  Cof[2][2]=(A[0][0]*A[1][1]-A[1][0]*A[0][1]);

  for(int i=0;i<3;i++)
    {
      for(int j=0;j<3;j++)
        {
          rAdj[i][j] = Cof[j][i];
        }
    }
  
}


void MatInversa(double A[3][3])
{
  if(det(A)!=0)
  {
    MatAdjunta(A);
    MatDivEscalar(det(A), rAdj);
  }
  for(int i=0;i<3;i++)
    {
      for(int j=0;j<3;j++)
        {
          rInversa[i][j] = rDivX[i][j];
        }
    }
}
