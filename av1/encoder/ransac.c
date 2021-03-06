/*
 * Copyright (c) 2016, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */
#define _POSIX_C_SOURCE 200112L  // rand_r()
#include <memory.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "av1/encoder/ransac.h"

#define MAX_MINPTS 4
#define MAX_DEGENERATE_ITER 10
#define MINPTS_MULTIPLIER 5

#define INLIER_THRESHOLD 1.0
#define MIN_TRIALS 20

////////////////////////////////////////////////////////////////////////////////
// ransac
typedef int (*IsDegenerateFunc)(double *p);
typedef void (*NormalizeFunc)(double *p, int np, double *T);
typedef void (*DenormalizeFunc)(double *params, double *T1, double *T2);
typedef int (*FindTransformationFunc)(int points, double *points1,
                                      double *points2, double *params);
typedef void (*ProjectPointsDoubleFunc)(double *mat, double *points,
                                        double *proj, const int n,
                                        const int stride_points,
                                        const int stride_proj);

static void project_points_double_translation(double *mat, double *points,
                                              double *proj, const int n,
                                              const int stride_points,
                                              const int stride_proj) {
  int i;
  for (i = 0; i < n; ++i) {
    const double x = *(points++), y = *(points++);
    *(proj++) = x + mat[0];
    *(proj++) = y + mat[1];
    points += stride_points - 2;
    proj += stride_proj - 2;
  }
}

static void project_points_double_rotzoom(double *mat, double *points,
                                          double *proj, const int n,
                                          const int stride_points,
                                          const int stride_proj) {
  int i;
  for (i = 0; i < n; ++i) {
    const double x = *(points++), y = *(points++);
    *(proj++) = mat[2] * x + mat[3] * y + mat[0];
    *(proj++) = -mat[3] * x + mat[2] * y + mat[1];
    points += stride_points - 2;
    proj += stride_proj - 2;
  }
}

static void project_points_double_affine(double *mat, double *points,
                                         double *proj, const int n,
                                         const int stride_points,
                                         const int stride_proj) {
  int i;
  for (i = 0; i < n; ++i) {
    const double x = *(points++), y = *(points++);
    *(proj++) = mat[2] * x + mat[3] * y + mat[0];
    *(proj++) = mat[4] * x + mat[5] * y + mat[1];
    points += stride_points - 2;
    proj += stride_proj - 2;
  }
}

static void project_points_double_hortrapezoid(double *mat, double *points,
                                               double *proj, const int n,
                                               const int stride_points,
                                               const int stride_proj) {
  int i;
  double x, y, Z, Z_inv;
  for (i = 0; i < n; ++i) {
    x = *(points++), y = *(points++);
    Z_inv = mat[7] * y + 1;
    assert(fabs(Z_inv) > 0.000001);
    Z = 1. / Z_inv;
    *(proj++) = (mat[2] * x + mat[3] * y + mat[0]) * Z;
    *(proj++) = (mat[5] * y + mat[1]) * Z;
    points += stride_points - 2;
    proj += stride_proj - 2;
  }
}

static void project_points_double_vertrapezoid(double *mat, double *points,
                                               double *proj, const int n,
                                               const int stride_points,
                                               const int stride_proj) {
  int i;
  double x, y, Z, Z_inv;
  for (i = 0; i < n; ++i) {
    x = *(points++), y = *(points++);
    Z_inv = mat[6] * x + 1;
    assert(fabs(Z_inv) > 0.000001);
    Z = 1. / Z_inv;
    *(proj++) = (mat[2] * x + mat[0]) * Z;
    *(proj++) = (mat[4] * x + mat[5] * y + mat[1]) * Z;
    points += stride_points - 2;
    proj += stride_proj - 2;
  }
}

static void project_points_double_homography(double *mat, double *points,
                                             double *proj, const int n,
                                             const int stride_points,
                                             const int stride_proj) {
  int i;
  double x, y, Z, Z_inv;
  for (i = 0; i < n; ++i) {
    x = *(points++), y = *(points++);
    Z_inv = mat[6] * x + mat[7] * y + 1;
    assert(fabs(Z_inv) > 0.000001);
    Z = 1. / Z_inv;
    *(proj++) = (mat[2] * x + mat[3] * y + mat[0]) * Z;
    *(proj++) = (mat[4] * x + mat[5] * y + mat[1]) * Z;
    points += stride_points - 2;
    proj += stride_proj - 2;
  }
}

///////////////////////////////////////////////////////////////////////////////
// svdcmp
// Adopted from Numerical Recipes in C

static const double TINY_NEAR_ZERO = 1.0E-12;

static INLINE double sign(double a, double b) {
  return ((b) >= 0 ? fabs(a) : -fabs(a));
}

static INLINE double pythag(double a, double b) {
  double ct;
  const double absa = fabs(a);
  const double absb = fabs(b);

  if (absa > absb) {
    ct = absb / absa;
    return absa * sqrt(1.0 + ct * ct);
  } else {
    ct = absa / absb;
    return (absb == 0) ? 0 : absb * sqrt(1.0 + ct * ct);
  }
}

static void multiply_mat(const double *m1, const double *m2, double *res,
                         const int m1_rows, const int inner_dim,
                         const int m2_cols) {
  double sum;

  int row, col, inner;
  for (row = 0; row < m1_rows; ++row) {
    for (col = 0; col < m2_cols; ++col) {
      sum = 0;
      for (inner = 0; inner < inner_dim; ++inner)
        sum += m1[row * inner_dim + inner] * m2[inner * m2_cols + col];
      *(res++) = sum;
    }
  }
}

static int svdcmp(double **u, int m, int n, double w[], double **v) {
  const int max_its = 30;
  int flag, i, its, j, jj, k, l, nm;
  double anorm, c, f, g, h, s, scale, x, y, z;
  double *rv1 = (double *)aom_malloc(sizeof(*rv1) * (n + 1));
  g = scale = anorm = 0.0;
  for (i = 0; i < n; i++) {
    l = i + 1;
    rv1[i] = scale * g;
    g = s = scale = 0.0;
    if (i < m) {
      for (k = i; k < m; k++) scale += fabs(u[k][i]);
      if (scale != 0.) {
        for (k = i; k < m; k++) {
          u[k][i] /= scale;
          s += u[k][i] * u[k][i];
        }
        f = u[i][i];
        g = -sign(sqrt(s), f);
        h = f * g - s;
        u[i][i] = f - g;
        for (j = l; j < n; j++) {
          for (s = 0.0, k = i; k < m; k++) s += u[k][i] * u[k][j];
          f = s / h;
          for (k = i; k < m; k++) u[k][j] += f * u[k][i];
        }
        for (k = i; k < m; k++) u[k][i] *= scale;
      }
    }
    w[i] = scale * g;
    g = s = scale = 0.0;
    if (i < m && i != n - 1) {
      for (k = l; k < n; k++) scale += fabs(u[i][k]);
      if (scale != 0.) {
        for (k = l; k < n; k++) {
          u[i][k] /= scale;
          s += u[i][k] * u[i][k];
        }
        f = u[i][l];
        g = -sign(sqrt(s), f);
        h = f * g - s;
        u[i][l] = f - g;
        for (k = l; k < n; k++) rv1[k] = u[i][k] / h;
        for (j = l; j < m; j++) {
          for (s = 0.0, k = l; k < n; k++) s += u[j][k] * u[i][k];
          for (k = l; k < n; k++) u[j][k] += s * rv1[k];
        }
        for (k = l; k < n; k++) u[i][k] *= scale;
      }
    }
    anorm = fmax(anorm, (fabs(w[i]) + fabs(rv1[i])));
  }

  for (i = n - 1; i >= 0; i--) {
    if (i < n - 1) {
      if (g != 0.) {
        for (j = l; j < n; j++) v[j][i] = (u[i][j] / u[i][l]) / g;
        for (j = l; j < n; j++) {
          for (s = 0.0, k = l; k < n; k++) s += u[i][k] * v[k][j];
          for (k = l; k < n; k++) v[k][j] += s * v[k][i];
        }
      }
      for (j = l; j < n; j++) v[i][j] = v[j][i] = 0.0;
    }
    v[i][i] = 1.0;
    g = rv1[i];
    l = i;
  }
  for (i = AOMMIN(m, n) - 1; i >= 0; i--) {
    l = i + 1;
    g = w[i];
    for (j = l; j < n; j++) u[i][j] = 0.0;
    if (g != 0.) {
      g = 1.0 / g;
      for (j = l; j < n; j++) {
        for (s = 0.0, k = l; k < m; k++) s += u[k][i] * u[k][j];
        f = (s / u[i][i]) * g;
        for (k = i; k < m; k++) u[k][j] += f * u[k][i];
      }
      for (j = i; j < m; j++) u[j][i] *= g;
    } else {
      for (j = i; j < m; j++) u[j][i] = 0.0;
    }
    ++u[i][i];
  }
  for (k = n - 1; k >= 0; k--) {
    for (its = 0; its < max_its; its++) {
      flag = 1;
      for (l = k; l >= 0; l--) {
        nm = l - 1;
        if ((double)(fabs(rv1[l]) + anorm) == anorm || nm < 0) {
          flag = 0;
          break;
        }
        if ((double)(fabs(w[nm]) + anorm) == anorm) break;
      }
      if (flag) {
        c = 0.0;
        s = 1.0;
        for (i = l; i <= k; i++) {
          f = s * rv1[i];
          rv1[i] = c * rv1[i];
          if ((double)(fabs(f) + anorm) == anorm) break;
          g = w[i];
          h = pythag(f, g);
          w[i] = h;
          h = 1.0 / h;
          c = g * h;
          s = -f * h;
          for (j = 0; j < m; j++) {
            y = u[j][nm];
            z = u[j][i];
            u[j][nm] = y * c + z * s;
            u[j][i] = z * c - y * s;
          }
        }
      }
      z = w[k];
      if (l == k) {
        if (z < 0.0) {
          w[k] = -z;
          for (j = 0; j < n; j++) v[j][k] = -v[j][k];
        }
        break;
      }
      if (its == max_its - 1) {
        aom_free(rv1);
        return 1;
      }
      assert(k > 0);
      x = w[l];
      nm = k - 1;
      y = w[nm];
      g = rv1[nm];
      h = rv1[k];
      f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
      g = pythag(f, 1.0);
      f = ((x - z) * (x + z) + h * ((y / (f + sign(g, f))) - h)) / x;
      c = s = 1.0;
      for (j = l; j <= nm; j++) {
        i = j + 1;
        g = rv1[i];
        y = w[i];
        h = s * g;
        g = c * g;
        z = pythag(f, h);
        rv1[j] = z;
        c = f / z;
        s = h / z;
        f = x * c + g * s;
        g = g * c - x * s;
        h = y * s;
        y *= c;
        for (jj = 0; jj < n; jj++) {
          x = v[jj][j];
          z = v[jj][i];
          v[jj][j] = x * c + z * s;
          v[jj][i] = z * c - x * s;
        }
        z = pythag(f, h);
        w[j] = z;
        if (z != 0.) {
          z = 1.0 / z;
          c = f * z;
          s = h * z;
        }
        f = c * g + s * y;
        x = c * y - s * g;
        for (jj = 0; jj < m; jj++) {
          y = u[jj][j];
          z = u[jj][i];
          u[jj][j] = y * c + z * s;
          u[jj][i] = z * c - y * s;
        }
      }
      rv1[l] = 0.0;
      rv1[k] = f;
      w[k] = x;
    }
  }
  aom_free(rv1);
  return 0;
}

static int SVD(double *U, double *W, double *V, double *matx, int M, int N) {
  // Assumes allocation for U is MxN
  double **nrU = (double **)aom_malloc((M) * sizeof(*nrU));
  double **nrV = (double **)aom_malloc((N) * sizeof(*nrV));
  int problem, i;

  problem = !(nrU && nrV);
  if (!problem) {
    for (i = 0; i < M; i++) {
      nrU[i] = &U[i * N];
    }
    for (i = 0; i < N; i++) {
      nrV[i] = &V[i * N];
    }
  } else {
    if (nrU) aom_free(nrU);
    if (nrV) aom_free(nrV);
    return 1;
  }

  /* copy from given matx into nrU */
  for (i = 0; i < M; i++) {
    memcpy(&(nrU[i][0]), matx + N * i, N * sizeof(*matx));
  }

  /* HERE IT IS: do SVD */
  if (svdcmp(nrU, M, N, W, nrV)) {
    aom_free(nrU);
    aom_free(nrV);
    return 1;
  }

  /* aom_free Numerical Recipes arrays */
  aom_free(nrU);
  aom_free(nrV);

  return 0;
}

int pseudo_inverse(double *inv, double *matx, const int M, const int N) {
  double ans;
  int i, j, k;
  double *const U = (double *)aom_malloc(M * N * sizeof(*matx));
  double *const W = (double *)aom_malloc(N * sizeof(*matx));
  double *const V = (double *)aom_malloc(N * N * sizeof(*matx));

  if (!(U && W && V)) {
    return 1;
  }
  if (SVD(U, W, V, matx, M, N)) {
    aom_free(U);
    aom_free(W);
    aom_free(V);
    return 1;
  }
  for (i = 0; i < N; i++) {
    if (fabs(W[i]) < TINY_NEAR_ZERO) {
      aom_free(U);
      aom_free(W);
      aom_free(V);
      return 1;
    }
  }

  for (i = 0; i < N; i++) {
    for (j = 0; j < M; j++) {
      ans = 0;
      for (k = 0; k < N; k++) {
        ans += V[k + N * i] * U[k + N * j] / W[k];
      }
      inv[j + M * i] = ans;
    }
  }
  aom_free(U);
  aom_free(W);
  aom_free(V);
  return 0;
}

static void normalize_homography(double *pts, int n, double *T) {
  double *p = pts;
  double mean[2] = { 0, 0 };
  double msqe = 0;
  double scale;
  int i;
  for (i = 0; i < n; ++i, p += 2) {
    mean[0] += p[0];
    mean[1] += p[1];
  }
  mean[0] /= n;
  mean[1] /= n;
  for (p = pts, i = 0; i < n; ++i, p += 2) {
    p[0] -= mean[0];
    p[1] -= mean[1];
    msqe += sqrt(p[0] * p[0] + p[1] * p[1]);
  }
  msqe /= n;
  scale = sqrt(2) / msqe;
  T[0] = scale;
  T[1] = 0;
  T[2] = -scale * mean[0];
  T[3] = 0;
  T[4] = scale;
  T[5] = -scale * mean[1];
  T[6] = 0;
  T[7] = 0;
  T[8] = 1;
  for (p = pts, i = 0; i < n; ++i, p += 2) {
    p[0] *= scale;
    p[1] *= scale;
  }
}

static void invnormalize_mat(double *T, double *iT) {
  double is = 1.0 / T[0];
  double m0 = -T[2] * is;
  double m1 = -T[5] * is;
  iT[0] = is;
  iT[1] = 0;
  iT[2] = m0;
  iT[3] = 0;
  iT[4] = is;
  iT[5] = m1;
  iT[6] = 0;
  iT[7] = 0;
  iT[8] = 1;
}

static void denormalize_homography(double *params, double *T1, double *T2) {
  double iT2[9];
  double params2[9];
  invnormalize_mat(T2, iT2);
  multiply_mat(params, T1, params2, 3, 3, 3);
  multiply_mat(iT2, params2, params, 3, 3, 3);
}

static void denormalize_homography_reorder(double *params, double *T1,
                                           double *T2) {
  double params_denorm[MAX_PARAMDIM];
  memcpy(params_denorm, params, sizeof(*params) * 8);
  params_denorm[8] = 1.0;
  denormalize_homography(params_denorm, T1, T2);
  params[0] = params_denorm[2];
  params[1] = params_denorm[5];
  params[2] = params_denorm[0];
  params[3] = params_denorm[1];
  params[4] = params_denorm[3];
  params[5] = params_denorm[4];
  params[6] = params_denorm[6];
  params[7] = params_denorm[7];
}

static void denormalize_affine_reorder(double *params, double *T1, double *T2) {
  double params_denorm[MAX_PARAMDIM];
  params_denorm[0] = params[0];
  params_denorm[1] = params[1];
  params_denorm[2] = params[4];
  params_denorm[3] = params[2];
  params_denorm[4] = params[3];
  params_denorm[5] = params[5];
  params_denorm[6] = params_denorm[7] = 0;
  params_denorm[8] = 1;
  denormalize_homography(params_denorm, T1, T2);
  params[0] = params_denorm[2];
  params[1] = params_denorm[5];
  params[2] = params_denorm[0];
  params[3] = params_denorm[1];
  params[4] = params_denorm[3];
  params[5] = params_denorm[4];
  params[6] = params[7] = 0;
}

static void denormalize_rotzoom_reorder(double *params, double *T1,
                                        double *T2) {
  double params_denorm[MAX_PARAMDIM];
  params_denorm[0] = params[0];
  params_denorm[1] = params[1];
  params_denorm[2] = params[2];
  params_denorm[3] = -params[1];
  params_denorm[4] = params[0];
  params_denorm[5] = params[3];
  params_denorm[6] = params_denorm[7] = 0;
  params_denorm[8] = 1;
  denormalize_homography(params_denorm, T1, T2);
  params[0] = params_denorm[2];
  params[1] = params_denorm[5];
  params[2] = params_denorm[0];
  params[3] = params_denorm[1];
  params[4] = -params[3];
  params[5] = params[2];
  params[6] = params[7] = 0;
}

static void denormalize_translation_reorder(double *params, double *T1,
                                            double *T2) {
  double params_denorm[MAX_PARAMDIM];
  params_denorm[0] = 1;
  params_denorm[1] = 0;
  params_denorm[2] = params[0];
  params_denorm[3] = 0;
  params_denorm[4] = 1;
  params_denorm[5] = params[1];
  params_denorm[6] = params_denorm[7] = 0;
  params_denorm[8] = 1;
  denormalize_homography(params_denorm, T1, T2);
  params[0] = params_denorm[2];
  params[1] = params_denorm[5];
  params[2] = params[5] = 1;
  params[3] = params[4] = 0;
  params[6] = params[7] = 0;
}

static int find_translation(const int np, double *pts1, double *pts2,
                            double *mat) {
  int i;
  double sx, sy, dx, dy;
  double sumx, sumy;

  double T1[9], T2[9];
  normalize_homography(pts1, np, T1);
  normalize_homography(pts2, np, T2);

  sumx = 0;
  sumy = 0;
  for (i = 0; i < np; ++i) {
    dx = *(pts2++);
    dy = *(pts2++);
    sx = *(pts1++);
    sy = *(pts1++);

    sumx += dx - sx;
    sumy += dy - sy;
  }
  mat[0] = sumx / np;
  mat[1] = sumy / np;
  denormalize_translation_reorder(mat, T1, T2);
  return 0;
}

static int find_rotzoom(const int np, double *pts1, double *pts2, double *mat) {
  const int np2 = np * 2;
  double *a = (double *)aom_malloc(sizeof(*a) * np2 * 9);
  double *b = a + np2 * 4;
  double *temp = b + np2;
  int i;
  double sx, sy, dx, dy;

  double T1[9], T2[9];
  normalize_homography(pts1, np, T1);
  normalize_homography(pts2, np, T2);

  for (i = 0; i < np; ++i) {
    dx = *(pts2++);
    dy = *(pts2++);
    sx = *(pts1++);
    sy = *(pts1++);

    a[i * 2 * 4 + 0] = sx;
    a[i * 2 * 4 + 1] = sy;
    a[i * 2 * 4 + 2] = 1;
    a[i * 2 * 4 + 3] = 0;
    a[(i * 2 + 1) * 4 + 0] = sy;
    a[(i * 2 + 1) * 4 + 1] = -sx;
    a[(i * 2 + 1) * 4 + 2] = 0;
    a[(i * 2 + 1) * 4 + 3] = 1;

    b[2 * i] = dx;
    b[2 * i + 1] = dy;
  }
  if (pseudo_inverse(temp, a, np2, 4)) {
    aom_free(a);
    return 1;
  }
  multiply_mat(temp, b, mat, 4, np2, 1);
  denormalize_rotzoom_reorder(mat, T1, T2);
  aom_free(a);
  return 0;
}

static int find_affine(const int np, double *pts1, double *pts2, double *mat) {
  const int np2 = np * 2;
  double *a = (double *)aom_malloc(sizeof(*a) * np2 * 13);
  double *b = a + np2 * 6;
  double *temp = b + np2;
  int i;
  double sx, sy, dx, dy;

  double T1[9], T2[9];
  normalize_homography(pts1, np, T1);
  normalize_homography(pts2, np, T2);

  for (i = 0; i < np; ++i) {
    dx = *(pts2++);
    dy = *(pts2++);
    sx = *(pts1++);
    sy = *(pts1++);

    a[i * 2 * 6 + 0] = sx;
    a[i * 2 * 6 + 1] = sy;
    a[i * 2 * 6 + 2] = 0;
    a[i * 2 * 6 + 3] = 0;
    a[i * 2 * 6 + 4] = 1;
    a[i * 2 * 6 + 5] = 0;
    a[(i * 2 + 1) * 6 + 0] = 0;
    a[(i * 2 + 1) * 6 + 1] = 0;
    a[(i * 2 + 1) * 6 + 2] = sx;
    a[(i * 2 + 1) * 6 + 3] = sy;
    a[(i * 2 + 1) * 6 + 4] = 0;
    a[(i * 2 + 1) * 6 + 5] = 1;

    b[2 * i] = dx;
    b[2 * i + 1] = dy;
  }
  if (pseudo_inverse(temp, a, np2, 6)) {
    aom_free(a);
    return 1;
  }
  multiply_mat(temp, b, mat, 6, np2, 1);
  denormalize_affine_reorder(mat, T1, T2);
  aom_free(a);
  return 0;
}

static int find_vertrapezoid(const int np, double *pts1, double *pts2,
                             double *mat) {
  const int np3 = np * 3;
  double *a = (double *)aom_malloc(sizeof(*a) * np3 * 14);
  double *U = a + np3 * 7;
  double S[7], V[7 * 7], H[9];
  int i, mini;
  double sx, sy, dx, dy;
  double T1[9], T2[9];

  normalize_homography(pts1, np, T1);
  normalize_homography(pts2, np, T2);

  for (i = 0; i < np; ++i) {
    dx = *(pts2++);
    dy = *(pts2++);
    sx = *(pts1++);
    sy = *(pts1++);

    a[i * 3 * 7 + 0] = a[i * 3 * 7 + 1] = 0;
    a[i * 3 * 7 + 2] = -sx;
    a[i * 3 * 7 + 3] = -sy;
    a[i * 3 * 7 + 4] = -1;
    a[i * 3 * 7 + 5] = dy * sx;
    a[i * 3 * 7 + 6] = dy;

    a[(i * 3 + 1) * 7 + 0] = sx;
    a[(i * 3 + 1) * 7 + 1] = 1;
    a[(i * 3 + 1) * 7 + 2] = a[(i * 3 + 1) * 7 + 3] = a[(i * 3 + 1) * 7 + 4] =
        0;
    a[(i * 3 + 1) * 7 + 5] = -dx * sx;
    a[(i * 3 + 1) * 7 + 6] = -dx;

    a[(i * 3 + 2) * 7 + 0] = -dy * sx;
    a[(i * 3 + 2) * 7 + 1] = -dy;
    a[(i * 3 + 2) * 7 + 2] = dx * sx;
    a[(i * 3 + 2) * 7 + 3] = dx * sy;
    a[(i * 3 + 2) * 7 + 4] = dx;
    a[(i * 3 + 2) * 7 + 5] = a[(i * 3 + 2) * 7 + 6] = 0;
  }
  if (SVD(U, S, V, a, np3, 7)) {
    aom_free(a);
    return 1;
  } else {
    double minS = 1e12;
    mini = -1;
    for (i = 0; i < 7; ++i) {
      if (S[i] < minS) {
        minS = S[i];
        mini = i;
      }
    }
  }
  H[1] = H[7] = 0;
  for (i = 0; i < 1; i++) H[i] = V[i * 7 + mini];
  for (; i < 6; i++) H[i + 1] = V[i * 7 + mini];
  for (; i < 7; i++) H[i + 2] = V[i * 7 + mini];

  denormalize_homography_reorder(H, T1, T2);
  aom_free(a);
  if (H[8] == 0.0) {
    return 1;
  } else {
    // normalize
    double f = 1.0 / H[8];
    for (i = 0; i < 8; i++) mat[i] = f * H[i];
  }
  return 0;
}

static int find_hortrapezoid(const int np, double *pts1, double *pts2,
                             double *mat) {
  const int np3 = np * 3;
  double *a = (double *)aom_malloc(sizeof(*a) * np3 * 14);
  double *U = a + np3 * 7;
  double S[7], V[7 * 7], H[9];
  int i, mini;
  double sx, sy, dx, dy;
  double T1[9], T2[9];

  normalize_homography(pts1, np, T1);
  normalize_homography(pts2, np, T2);

  for (i = 0; i < np; ++i) {
    dx = *(pts2++);
    dy = *(pts2++);
    sx = *(pts1++);
    sy = *(pts1++);

    a[i * 3 * 7 + 0] = a[i * 3 * 7 + 1] = a[i * 3 * 7 + 2] = 0;
    a[i * 3 * 7 + 3] = -sy;
    a[i * 3 * 7 + 4] = -1;
    a[i * 3 * 7 + 5] = dy * sy;
    a[i * 3 * 7 + 6] = dy;

    a[(i * 3 + 1) * 7 + 0] = sx;
    a[(i * 3 + 1) * 7 + 1] = sy;
    a[(i * 3 + 1) * 7 + 2] = 1;
    a[(i * 3 + 1) * 7 + 3] = a[(i * 3 + 1) * 7 + 4] = 0;
    a[(i * 3 + 1) * 7 + 5] = -dx * sy;
    a[(i * 3 + 1) * 7 + 6] = -dx;

    a[(i * 3 + 2) * 7 + 0] = -dy * sx;
    a[(i * 3 + 2) * 7 + 1] = -dy * sy;
    a[(i * 3 + 2) * 7 + 2] = -dy;
    a[(i * 3 + 2) * 7 + 3] = dx * sy;
    a[(i * 3 + 2) * 7 + 4] = dx;
    a[(i * 3 + 2) * 7 + 5] = a[(i * 3 + 2) * 7 + 6] = 0;
  }

  if (SVD(U, S, V, a, np3, 7)) {
    aom_free(a);
    return 1;
  } else {
    double minS = 1e12;
    mini = -1;
    for (i = 0; i < 7; ++i) {
      if (S[i] < minS) {
        minS = S[i];
        mini = i;
      }
    }
  }
  H[3] = H[6] = 0;
  for (i = 0; i < 3; i++) H[i] = V[i * 7 + mini];
  for (; i < 5; i++) H[i + 1] = V[i * 7 + mini];
  for (; i < 7; i++) H[i + 2] = V[i * 7 + mini];

  denormalize_homography_reorder(H, T1, T2);
  aom_free(a);
  if (H[8] == 0.0) {
    return 1;
  } else {
    // normalize
    double f = 1.0 / H[8];
    for (i = 0; i < 8; i++) mat[i] = f * H[i];
  }
  return 0;
}

static int find_homography(const int np, double *pts1, double *pts2,
                           double *mat) {
  // Implemented from Peter Kovesi's normalized implementation
  const int np3 = np * 3;
  double *a = (double *)aom_malloc(sizeof(*a) * np3 * 18);
  double *U = a + np3 * 9;
  double S[9], V[9 * 9], H[9];
  int i, mini;
  double sx, sy, dx, dy;
  double T1[9], T2[9];

  normalize_homography(pts1, np, T1);
  normalize_homography(pts2, np, T2);

  for (i = 0; i < np; ++i) {
    dx = *(pts2++);
    dy = *(pts2++);
    sx = *(pts1++);
    sy = *(pts1++);

    a[i * 3 * 9 + 0] = a[i * 3 * 9 + 1] = a[i * 3 * 9 + 2] = 0;
    a[i * 3 * 9 + 3] = -sx;
    a[i * 3 * 9 + 4] = -sy;
    a[i * 3 * 9 + 5] = -1;
    a[i * 3 * 9 + 6] = dy * sx;
    a[i * 3 * 9 + 7] = dy * sy;
    a[i * 3 * 9 + 8] = dy;

    a[(i * 3 + 1) * 9 + 0] = sx;
    a[(i * 3 + 1) * 9 + 1] = sy;
    a[(i * 3 + 1) * 9 + 2] = 1;
    a[(i * 3 + 1) * 9 + 3] = a[(i * 3 + 1) * 9 + 4] = a[(i * 3 + 1) * 9 + 5] =
        0;
    a[(i * 3 + 1) * 9 + 6] = -dx * sx;
    a[(i * 3 + 1) * 9 + 7] = -dx * sy;
    a[(i * 3 + 1) * 9 + 8] = -dx;

    a[(i * 3 + 2) * 9 + 0] = -dy * sx;
    a[(i * 3 + 2) * 9 + 1] = -dy * sy;
    a[(i * 3 + 2) * 9 + 2] = -dy;
    a[(i * 3 + 2) * 9 + 3] = dx * sx;
    a[(i * 3 + 2) * 9 + 4] = dx * sy;
    a[(i * 3 + 2) * 9 + 5] = dx;
    a[(i * 3 + 2) * 9 + 6] = a[(i * 3 + 2) * 9 + 7] = a[(i * 3 + 2) * 9 + 8] =
        0;
  }

  if (SVD(U, S, V, a, np3, 9)) {
    aom_free(a);
    return 1;
  } else {
    double minS = 1e12;
    mini = -1;
    for (i = 0; i < 9; ++i) {
      if (S[i] < minS) {
        minS = S[i];
        mini = i;
      }
    }
  }

  for (i = 0; i < 9; i++) H[i] = V[i * 9 + mini];
  denormalize_homography_reorder(H, T1, T2);
  aom_free(a);
  if (H[8] == 0.0) {
    return 1;
  } else {
    // normalize
    double f = 1.0 / H[8];
    for (i = 0; i < 8; i++) mat[i] = f * H[i];
  }
  return 0;
}

static int get_rand_indices(int npoints, int minpts, int *indices,
                            unsigned int *seed) {
  int i, j;
  int ptr = rand_r(seed) % npoints;
  if (minpts > npoints) return 0;
  indices[0] = ptr;
  ptr = (ptr == npoints - 1 ? 0 : ptr + 1);
  i = 1;
  while (i < minpts) {
    int index = rand_r(seed) % npoints;
    while (index) {
      ptr = (ptr == npoints - 1 ? 0 : ptr + 1);
      for (j = 0; j < i; ++j) {
        if (indices[j] == ptr) break;
      }
      if (j == i) index--;
    }
    indices[i++] = ptr;
  }
  return 1;
}

static int ransac(int *matched_points, int npoints, int *number_of_inliers,
                  double *best_params, const int minpts,
                  IsDegenerateFunc is_degenerate,
                  FindTransformationFunc find_transformation,
                  ProjectPointsDoubleFunc projectpoints) {
  static const double PROBABILITY_REQUIRED = 0.9;
  static const double EPS = 1e-12;

  int N = 10000, trial_count = 0;
  int i;
  int ret_val = 0;
  unsigned int seed = (unsigned int)npoints;

  int max_inliers = 0;
  double best_variance = 0.0;
  double params[MAX_PARAMDIM];
  WarpedMotionParams wm;
  double points1[2 * MAX_MINPTS];
  double points2[2 * MAX_MINPTS];
  int indices[MAX_MINPTS] = { 0 };

  double *best_inlier_set1;
  double *best_inlier_set2;
  double *inlier_set1;
  double *inlier_set2;
  double *corners1;
  double *corners2;
  double *image1_coord;

  double *cnp1, *cnp2;

  *number_of_inliers = 0;
  if (npoints < minpts * MINPTS_MULTIPLIER || npoints == 0) {
    return 1;
  }

  memset(&wm, 0, sizeof(wm));
  best_inlier_set1 =
      (double *)aom_malloc(sizeof(*best_inlier_set1) * npoints * 2);
  best_inlier_set2 =
      (double *)aom_malloc(sizeof(*best_inlier_set2) * npoints * 2);
  inlier_set1 = (double *)aom_malloc(sizeof(*inlier_set1) * npoints * 2);
  inlier_set2 = (double *)aom_malloc(sizeof(*inlier_set2) * npoints * 2);
  corners1 = (double *)aom_malloc(sizeof(*corners1) * npoints * 2);
  corners2 = (double *)aom_malloc(sizeof(*corners2) * npoints * 2);
  image1_coord = (double *)aom_malloc(sizeof(*image1_coord) * npoints * 2);

  if (!(best_inlier_set1 && best_inlier_set2 && inlier_set1 && inlier_set2 &&
        corners1 && corners2 && image1_coord)) {
    ret_val = 1;
    goto finish_ransac;
  }

  for (cnp1 = corners1, cnp2 = corners2, i = 0; i < npoints; ++i) {
    *(cnp1++) = *(matched_points++);
    *(cnp1++) = *(matched_points++);
    *(cnp2++) = *(matched_points++);
    *(cnp2++) = *(matched_points++);
  }
  matched_points -= 4 * npoints;

  while (N > trial_count) {
    int num_inliers = 0;
    double sum_distance = 0.0;
    double sum_distance_squared = 0.0;

    int degenerate = 1;
    int num_degenerate_iter = 0;
    while (degenerate) {
      num_degenerate_iter++;
      if (!get_rand_indices(npoints, minpts, indices, &seed)) {
        ret_val = 1;
        goto finish_ransac;
      }
      i = 0;
      while (i < minpts) {
        int index = indices[i];
        // add to list
        points1[i * 2] = corners1[index * 2];
        points1[i * 2 + 1] = corners1[index * 2 + 1];
        points2[i * 2] = corners2[index * 2];
        points2[i * 2 + 1] = corners2[index * 2 + 1];
        i++;
      }
      degenerate = is_degenerate(points1);
      if (num_degenerate_iter > MAX_DEGENERATE_ITER) {
        ret_val = 1;
        goto finish_ransac;
      }
    }

    if (find_transformation(minpts, points1, points2, params)) {
      trial_count++;
      continue;
    }

    projectpoints(params, corners1, image1_coord, npoints, 2, 2);

    for (i = 0; i < npoints; ++i) {
      double dx = image1_coord[i * 2] - corners2[i * 2];
      double dy = image1_coord[i * 2 + 1] - corners2[i * 2 + 1];
      double distance = sqrt(dx * dx + dy * dy);

      if (distance < INLIER_THRESHOLD) {
        inlier_set1[num_inliers * 2] = corners1[i * 2];
        inlier_set1[num_inliers * 2 + 1] = corners1[i * 2 + 1];
        inlier_set2[num_inliers * 2] = corners2[i * 2];
        inlier_set2[num_inliers * 2 + 1] = corners2[i * 2 + 1];
        num_inliers++;
        sum_distance += distance;
        sum_distance_squared += distance * distance;
      }
    }

    if (num_inliers >= max_inliers && num_inliers > 1) {
      int temp;
      double fracinliers, pNoOutliers, mean_distance, variance;

      mean_distance = sum_distance / ((double)num_inliers);
      variance = sum_distance_squared / ((double)num_inliers - 1.0) -
                 mean_distance * mean_distance * ((double)num_inliers) /
                     ((double)num_inliers - 1.0);
      if ((num_inliers > max_inliers) ||
          (num_inliers == max_inliers && variance < best_variance)) {
        best_variance = variance;
        max_inliers = num_inliers;
        // Save parameters, excluding the implicit '1' in the bottom-right
        // entry of the parameter matrix
        memcpy(best_params, params, (MAX_PARAMDIM - 1) * sizeof(*best_params));
        memcpy(best_inlier_set1, inlier_set1,
               num_inliers * 2 * sizeof(*best_inlier_set1));
        memcpy(best_inlier_set2, inlier_set2,
               num_inliers * 2 * sizeof(*best_inlier_set2));

        assert(npoints > 0);
        fracinliers = (double)num_inliers / (double)npoints;
        pNoOutliers = 1 - pow(fracinliers, minpts);
        pNoOutliers = fmax(EPS, pNoOutliers);
        pNoOutliers = fmin(1 - EPS, pNoOutliers);
        temp = (int)(log(1.0 - PROBABILITY_REQUIRED) / log(pNoOutliers));
        if (temp > 0 && temp < N) {
          N = AOMMAX(temp, MIN_TRIALS);
        }
      }
    }
    trial_count++;
  }
  find_transformation(max_inliers, best_inlier_set1, best_inlier_set2,
                      best_params);
  *number_of_inliers = max_inliers;
finish_ransac:
  aom_free(best_inlier_set1);
  aom_free(best_inlier_set2);
  aom_free(inlier_set1);
  aom_free(inlier_set2);
  aom_free(corners1);
  aom_free(corners2);
  aom_free(image1_coord);
  return ret_val;
}

static int is_collinear3(double *p1, double *p2, double *p3) {
  static const double collinear_eps = 1e-3;
  const double v =
      (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0]);
  return fabs(v) < collinear_eps;
}

static int is_degenerate_translation(double *p) {
  return (p[0] - p[2]) * (p[0] - p[2]) + (p[1] - p[3]) * (p[1] - p[3]) <= 2;
}

static int is_degenerate_affine(double *p) {
  return is_collinear3(p, p + 2, p + 4);
}

static int is_degenerate_homography(double *p) {
  return is_collinear3(p, p + 2, p + 4) || is_collinear3(p, p + 2, p + 6) ||
         is_collinear3(p, p + 4, p + 6) || is_collinear3(p + 2, p + 4, p + 6);
}

int ransac_translation(int *matched_points, int npoints, int *number_of_inliers,
                       double *best_params) {
  return ransac(matched_points, npoints, number_of_inliers, best_params, 3,
                is_degenerate_translation, find_translation,
                project_points_double_translation);
}

int ransac_rotzoom(int *matched_points, int npoints, int *number_of_inliers,
                   double *best_params) {
  return ransac(matched_points, npoints, number_of_inliers, best_params, 3,
                is_degenerate_affine, find_rotzoom,
                project_points_double_rotzoom);
}

int ransac_affine(int *matched_points, int npoints, int *number_of_inliers,
                  double *best_params) {
  return ransac(matched_points, npoints, number_of_inliers, best_params, 3,
                is_degenerate_affine, find_affine,
                project_points_double_affine);
}

int ransac_homography(int *matched_points, int npoints, int *number_of_inliers,
                      double *best_params) {
  return ransac(matched_points, npoints, number_of_inliers, best_params, 4,
                is_degenerate_homography, find_homography,
                project_points_double_homography);
}

int ransac_hortrapezoid(int *matched_points, int npoints,
                        int *number_of_inliers, double *best_params) {
  return ransac(matched_points, npoints, number_of_inliers, best_params, 4,
                is_degenerate_homography, find_hortrapezoid,
                project_points_double_hortrapezoid);
}

int ransac_vertrapezoid(int *matched_points, int npoints,
                        int *number_of_inliers, double *best_params) {
  return ransac(matched_points, npoints, number_of_inliers, best_params, 4,
                is_degenerate_homography, find_vertrapezoid,
                project_points_double_vertrapezoid);
}
