// Small dense linear algebra for the estimator. Fixed sizes, no allocation.
//
// Only what a Kalman update needs: square multiply, transpose multiply, add,
// and an inverse of the tiny innovation matrix. Sizes are passed in so the
// functions stay testable on 2x2 matrices by hand.
#ifndef MCQ_LINALG_H
#define MCQ_LINALG_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// C = A * B, all n x n, row major. C may not alias A or B.
void mcq_mat_mul(const double * a, const double * b, double * c, int n);

// C = A * B^T, all n x n, row major. C may not alias A or B.
void mcq_mat_mul_transpose(const double * a, const double * b, double * c, int n);

// C = A + B, n x n.
void mcq_mat_add(const double * a, const double * b, double * c, int n);

// Forces exact symmetry: A = (A + A^T) / 2. A covariance that drifts
// asymmetric through rounding is the classic way a filter quietly diverges.
void mcq_mat_symmetrize(double * a, int n);

// A = I, n x n.
void mcq_mat_identity(double * a, int n);

// Inverts the m x m matrix a into out by Gauss-Jordan with partial pivoting.
// Returns false and leaves out untouched when a is singular. Intended for the
// m <= 3 innovation matrices of this filter, not for anything large.
bool mcq_mat_inverse(const double * a, double * out, int m);

#ifdef __cplusplus
}
#endif

#endif  // MCQ_LINALG_H
