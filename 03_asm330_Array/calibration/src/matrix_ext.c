
#include "matrix_ext.h"

#include <math.h>
#include <string.h>

#define GET(mat, cols, row, col) ((mat)[(row) * (cols) + (col)])
#define SET(mat, cols, row, col, val) ((mat)[(row) * (cols) + (col)] = (val))
#define SWAP(a, b)                                                                                                     \
    {                                                                                                                  \
        temp = (a);                                                                                                    \
        (a)  = (b);                                                                                                    \
        (b)  = temp;                                                                                                   \
    }

void matrixInverse(int n, const float *src, float *dst) {
    static int    i, icol, irow, j, k, l, ll;
    static float big, dum, pivinv, temp;
    static int    indxc[KALMANX], indxr[KALMANX], ipiv[KALMANX];

    // copy src to dst
    memcpy(dst, src, n * n * sizeof(float));

    for (j = 0; j < n; j++)
        ipiv[j] = 0;
    for (i = 0; i < n; i++) {
        big = 0.0;
        for (j = 0; j < n; j++) {
            if (ipiv[j] != 1) {
                for (k = 0; k < n; k++) {
                    if (ipiv[k] == 0) {
                        if (fabs(*(dst + j * n + k)) >= big) {
                            big  = fabs(*(dst + j * n + k));
                            irow = j;
                            icol = k;
                        }
                    }
                }
            }
        }
        ++(ipiv[icol]);

        if (irow != icol) {
            for (l = 0; l < n; l++) {
                SWAP(*(dst + irow * n + l), *(dst + icol * n + l));
            }
        }

        indxr[i] = irow;
        indxc[i] = icol;

        if (fabs(*(dst + icol * n + icol)) < 1.0e-20) {
            return;
        }

        pivinv                   = 1.0 / (*(dst + icol * n + icol));
        *(dst + icol * n + icol) = 1.0;

        for (l = 0; l < n; l++)
            *(dst + icol * n + l) *= pivinv;

        for (ll = 0; ll < n; ll++)
            if (ll != icol) {
                dum                    = *(dst + ll * n + icol);
                *(dst + ll * n + icol) = 0.0;
                for (l = 0; l < n; l++)
                    *(dst + ll * n + l) -= *(dst + icol * n + l) * dum;
            }
    }

    for (l = n - 1; l >= 0; l--) {
        if (indxr[l] != indxc[l])
            for (k = 0; k < n; k++)
                SWAP(*(dst + k * n + indxr[l]), *(dst + k * n + indxc[l]));
    }
}

void matrixTranspose(int rows, int cols, const float *src, float *dst) {
    static int i, j;

    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            SET(dst, rows, j, i, GET(src, cols, i, j));
        }
    }
}

void matrixMul(int rowsa, int colsa, const float *srca, int rowsb, int colsb, const float *srcb, float *dst) {
    static int    i, j, k;
    static float tmp;
    static int    rows, cols;

    rows = rowsa;
    cols = colsb;

    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            tmp = 0;
            for (k = 0; k < rowsb; k++) {
                tmp += GET(srca, colsa, i, k) * GET(srcb, colsb, k, j);
            }
            SET(dst, cols, i, j, tmp);
        }
    }
}

void matrixDelta(int rows, int cols, float delta, const float *src, float *dst) {
    static int k;

    for (k = 0; k < cols * rows; k++) {
        dst[k] = src[k] * delta;
    }
}

void matrixSub(int rows, int cols, const float *srca, const float *srcb, float *dst) {
    static int k;

    for (k = 0; k < cols * rows; k++) {
        dst[k] = srca[k] - srcb[k];
    }
}

void matrixAdd(int rows, int cols, const float *srca, const float *srcb, float *dst) {
    static int k;

    for (k = 0; k < cols * rows; k++) {
        dst[k] = srca[k] + srcb[k];
    }
}

int matrixCholesky(float a[], int n) {
    static int i, j, k, u, l;

    if (a[0] < 0.0) {
        return -1;
    }
    a[0] = sqrt(a[0]);
    for (i = 1; i <= n - 1; i++) {
        u    = i * n;
        a[u] = a[u] / a[0];
    }

    for (j = 1; j <= n - 1; j++) {
        l = j * n + j;
        for (k = 0; k <= j - 1; k++) {
            u    = j * n + k;
            a[l] = a[l] - a[u] * a[u];
        }
        if (a[l] < 0.0) {
            return -1;
        }
        a[l] = sqrt(a[l]);
        for (i = j + 1; i <= n - 1; i++) {
            u = i * n + j;
            for (k = 0; k <= j - 1; k++)
                a[u] = a[u] - a[i * n + k] * a[j * n + k];
            a[u] = a[u] / a[l];
        }
    }
    for (i = 0; i <= n - 2; i++)
        for (j = i + 1; j <= n - 1; j++)
            a[i * n + j] = 0.0;
    return 0;
}

void matrixMulSymm(int rowsa, int colsa, const float *srca, const float *srcb, float *dst) {
    static int    i, j, k;
    static float sum;

    for (i = 0; i < rowsa; i++) {
        for (j = 0; j < rowsa; j++) {
            sum = 0.0;

            for (k = 0; k < colsa; k++) {
                sum = sum + *(srca + i * colsa + k) * *(srcb + k) * *(srca + j * colsa + k);
            }

            *(dst + i * rowsa + j) = sum;
        }
    }
}
