#include "matrix.h"

__ramfunc void MatrixUnit2(float A[2][2])
{
	A[0][0] = 1;	A[0][1] = 0;
	A[1][0] = 0;	A[1][1] = 1;
}

__ramfunc void MatrixZero2(float A[2][2])
{
	A[0][0] = A[0][1] = 0;
	A[1][0] = A[1][1] = 0;
}

__ramfunc void VectorZero2  (float v[2])
{
	v[0] = v[1] = 0;
}

__ramfunc int MatrixInverse2 (float Ai[2][2], float A[2][2])
{
	float den = A[0][0]*A[1][1] - A[0][1]*A[1][0];
	if (!den) return 0;

	Ai[0][0] =  A[1][1]/den;
	Ai[0][1] = -A[0][1]/den;
	Ai[1][0] = -A[1][0]/den;
	Ai[1][1] =  A[0][0]/den;

	return 1;
}

__ramfunc void MatrixVectorMultiply2 (float x[2], float A[2][2], float v[2])
{
	x[0] = A[0][0]*v[0] + A[0][1]*v[1];
	x[1] = A[1][0]*v[0] + A[1][1]*v[1];
}

__ramfunc void MatrixMatrixMultiply2 (float x[2][2], float A[2][2], float B[2][2])
{
	x[0][0] = A[0][0]*B[0][0] + A[0][1]*B[1][0];
	x[0][1] = A[0][0]*B[0][1] + A[0][1]*B[1][1];
	x[1][0] = A[1][0]*B[0][0] + A[1][1]*B[1][0];
	x[1][1] = A[1][0]*B[0][1] + A[1][1]*B[1][1];
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

__ramfunc void MatrixZero3(float A[3][3])
{
	A[0][0] = A[0][1] = A[0][2] = 0;
	A[1][0] = A[1][1] = A[1][2] = 0;
	A[2][0] = A[2][1] = A[2][2] = 0;
}

__ramfunc void VectorZero3  (float v[3])
{
	v[0] = v[1] = v[2] = 0;
}

__ramfunc int MatrixInverse3 (float Ai[3][3], float A[3][3])
{
	float den = A[0][0]*A[1][1]*A[2][2] - A[0][0]*A[1][2]*A[2][1] - A[0][1]*A[1][0]*A[2][2]
			   + A[0][1]*A[1][2]*A[2][0] + A[0][2]*A[1][0]*A[2][1] - A[0][2]*A[1][1]*A[2][0];
	if (!den) return 0;

	Ai[0][0] = (A[1][1]*A[2][2] - A[1][2]*A[2][1])/den;
	Ai[0][1] = (A[0][2]*A[2][1] - A[0][1]*A[2][2])/den;
	Ai[0][2] = (A[0][1]*A[1][2] - A[0][2]*A[1][1])/den;
	Ai[1][0] = (A[1][2]*A[2][0] - A[1][0]*A[2][2])/den;
	Ai[1][1] = (A[0][0]*A[2][2] - A[0][2]*A[2][0])/den;
	Ai[1][2] = (A[0][2]*A[1][0] - A[0][0]*A[1][2])/den;
	Ai[2][0] = (A[1][0]*A[2][1] - A[1][1]*A[2][0])/den;
	Ai[2][1] = (A[0][1]*A[2][0] - A[0][0]*A[2][1])/den;
	Ai[2][2] = (A[0][0]*A[1][1] - A[0][1]*A[1][0])/den;

	return 1;
}

__ramfunc void MatrixVectorMultiply3 (float x[3], float A[3][3], float v[3])
{
	x[0] = A[0][0]*v[0] + A[0][1]*v[1] + A[0][2]*v[2];
	x[1] = A[1][0]*v[0] + A[1][1]*v[1] + A[1][2]*v[2];
	x[2] = A[2][0]*v[0] + A[2][1]*v[1] + A[2][2]*v[2];
}
