
#ifndef LC_MATRIX_H
#define LC_MATRIX_H

#define KALMANX  7

void matrixAdd(int rows, int cols, const float *srca, const float *srcb, float *dst);
void matrixSub(int rows, int cols, const float *srca, const float *srcb, float *dst);
void matrixInverse(int n, const float *src, float *dst);
void matrixTranspose(int rows, int cols, const float *src, float *dst);
void matrixMul(int rowsa, int colsa, const float *srca, int rowsb, int colsb, const float *srcb, float *dst);
void matrixDelta(int rows, int cols, float delta, const float *src, float *dst);
void matrixMulSymm(int rowsa, int colsa, const float *srca, const float *srcb, float *dst);
int  matrixCholesky(float a[], int n);

#endif /* LC_MATRIX_H */
