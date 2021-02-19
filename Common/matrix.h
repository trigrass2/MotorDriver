#ifndef __MATRIX_H__
#define __MATRIX_H__

#ifdef __cplusplus
 extern "C" {
#endif

__ramfunc extern void MatrixZero2 (float A[2][2]);
__ramfunc extern void MatrixUnit2 (float A[2][2]);
__ramfunc extern void VectorZero2 (float v[2]);
__ramfunc extern int  MatrixInverse2 (float Ai[2][2], float A[2][2]);
__ramfunc extern void MatrixVectorMultiply2 (float x[2], float A[2][2], float v[2]);
__ramfunc extern void MatrixMatrixMultiply2 (float x[2][2], float A[2][2], float B[2][2]);

__ramfunc extern void MatrixZero3 (float A[3][3]);
__ramfunc extern void VectorZero3 (float v[3]);
__ramfunc extern int  MatrixInverse3 (float Ai[3][3], float A[3][3]);
__ramfunc extern void MatrixVectorMultiply3 (float x[3], float A[3][3], float v[3]);

#ifdef __cplusplus
}
#endif

#endif // __MATRIX_H__