#ifndef _MATRICES_H_
#define _MATRICES_H_
#include "Matrices.h"

extern double rSuma[3][3];
extern double rResta[3][3];
extern double rMultiplicacion[3][1];
extern double rMultX[3][3];
extern double rDivX[3][3];
extern double rAdj[3][3];
extern double rTranspuesta[3][3];
extern double rInversa[3][3];
extern double rIdentidad[3][3];

// METODOS //

void ImprimirMat(double A[3][3]);
double det(double A[3][3]);
void MatSuma(double A[3][3], double B[3][3]);
void MatResta(double A[3][3], double B[3][3]);
void MatMultiplicacion(double A[3][3], double B[3][1]);
void MatMultEscalar(double Escalar, double A[3][3]);
void MatDivEscalar(double Escalar, double A[3][3]);
void MatTraspuesta(double A[3][3]);
void MatAdjunta(double A[3][3]);
void MatInversa(double A[3][3]);


#endif