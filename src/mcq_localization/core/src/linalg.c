#include "mcq/linalg.h"

#include <math.h>
#include <string.h>

#define MCQ_LINALG_MAX 8

void mcq_mat_mul(const double * a, const double * b, double * c, int n)
{
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      double sum = 0.0;
      for (int k = 0; k < n; ++k) {
        sum += a[i * n + k] * b[k * n + j];
      }
      c[i * n + j] = sum;
    }
  }
}

void mcq_mat_mul_transpose(const double * a, const double * b, double * c, int n)
{
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      double sum = 0.0;
      for (int k = 0; k < n; ++k) {
        sum += a[i * n + k] * b[j * n + k];
      }
      c[i * n + j] = sum;
    }
  }
}

void mcq_mat_add(const double * a, const double * b, double * c, int n)
{
  for (int i = 0; i < n * n; ++i) {
    c[i] = a[i] + b[i];
  }
}

void mcq_mat_symmetrize(double * a, int n)
{
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const double mean = 0.5 * (a[i * n + j] + a[j * n + i]);
      a[i * n + j] = mean;
      a[j * n + i] = mean;
    }
  }
}

void mcq_mat_identity(double * a, int n)
{
  memset(a, 0, (size_t)(n * n) * sizeof(double));
  for (int i = 0; i < n; ++i) {
    a[i * n + i] = 1.0;
  }
}

bool mcq_mat_inverse(const double * a, double * out, int m)
{
  if (m < 1 || m > MCQ_LINALG_MAX) {
    return false;
  }
  // Work on [a | I] and reduce the left half to the identity.
  double work[MCQ_LINALG_MAX * MCQ_LINALG_MAX * 2];
  const int w = 2 * m;
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < m; ++j) {
      work[i * w + j] = a[i * m + j];
      work[i * w + m + j] = (i == j) ? 1.0 : 0.0;
    }
  }

  for (int col = 0; col < m; ++col) {
    int pivot = col;
    for (int row = col + 1; row < m; ++row) {
      if (fabs(work[row * w + col]) > fabs(work[pivot * w + col])) {
        pivot = row;
      }
    }
    if (fabs(work[pivot * w + col]) < 1e-300) {
      return false;
    }
    if (pivot != col) {
      for (int j = 0; j < w; ++j) {
        const double tmp = work[col * w + j];
        work[col * w + j] = work[pivot * w + j];
        work[pivot * w + j] = tmp;
      }
    }
    const double inv_pivot = 1.0 / work[col * w + col];
    for (int j = 0; j < w; ++j) {
      work[col * w + j] *= inv_pivot;
    }
    for (int row = 0; row < m; ++row) {
      if (row == col) {
        continue;
      }
      const double factor = work[row * w + col];
      if (factor == 0.0) {
        continue;
      }
      for (int j = 0; j < w; ++j) {
        work[row * w + j] -= factor * work[col * w + j];
      }
    }
  }

  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < m; ++j) {
      out[i * m + j] = work[i * w + m + j];
    }
  }
  return true;
}
