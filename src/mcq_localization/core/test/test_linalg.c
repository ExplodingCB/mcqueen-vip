// The matrix helpers the Kalman update is built on. Small enough to check by
// hand, which is the point: an error here looks like a filter that diverges.
#include <math.h>
#include <stdio.h>

#include "check.h"
#include "mcq/linalg.h"

static int test_multiply_by_identity(void)
{
  const double a[4] = {1.0, 2.0, 3.0, 4.0};
  double id[4];
  double out[4];
  mcq_mat_identity(id, 2);
  mcq_mat_mul(a, id, out, 2);
  for (int i = 0; i < 4; ++i) {
    CHECK_NEAR(out[i], a[i], 1e-12);
  }
  return 0;
}

static int test_multiply_known(void)
{
  const double a[4] = {1.0, 2.0, 3.0, 4.0};
  const double b[4] = {5.0, 6.0, 7.0, 8.0};
  double out[4];
  mcq_mat_mul(a, b, out, 2);
  CHECK_NEAR(out[0], 19.0, 1e-12);
  CHECK_NEAR(out[1], 22.0, 1e-12);
  CHECK_NEAR(out[2], 43.0, 1e-12);
  CHECK_NEAR(out[3], 50.0, 1e-12);
  return 0;
}

static int test_multiply_transpose(void)
{
  const double a[4] = {1.0, 2.0, 3.0, 4.0};
  const double b[4] = {5.0, 6.0, 7.0, 8.0};
  double out[4];
  // A * B^T, so out[0][0] = 1*5 + 2*6 = 17.
  mcq_mat_mul_transpose(a, b, out, 2);
  CHECK_NEAR(out[0], 17.0, 1e-12);
  CHECK_NEAR(out[1], 23.0, 1e-12);
  CHECK_NEAR(out[2], 39.0, 1e-12);
  CHECK_NEAR(out[3], 53.0, 1e-12);
  return 0;
}

static int test_add_and_symmetrize(void)
{
  const double a[4] = {1.0, 2.0, 3.0, 4.0};
  const double b[4] = {10.0, 20.0, 30.0, 40.0};
  double out[4];
  mcq_mat_add(a, b, out, 2);
  CHECK_NEAR(out[3], 44.0, 1e-12);

  double skew[4] = {1.0, 2.0, 4.0, 5.0};
  mcq_mat_symmetrize(skew, 2);
  CHECK_NEAR(skew[1], 3.0, 1e-12);
  CHECK_NEAR(skew[2], 3.0, 1e-12);
  CHECK_NEAR(skew[0], 1.0, 1e-12);
  return 0;
}

static int test_inverse_2x2(void)
{
  const double a[4] = {4.0, 7.0, 2.0, 6.0};  // det 10
  double inv[4];
  CHECK(mcq_mat_inverse(a, inv, 2));
  CHECK_NEAR(inv[0], 0.6, 1e-12);
  CHECK_NEAR(inv[1], -0.7, 1e-12);
  CHECK_NEAR(inv[2], -0.2, 1e-12);
  CHECK_NEAR(inv[3], 0.4, 1e-12);
  return 0;
}

static int test_inverse_3x3_round_trip(void)
{
  const double a[9] = {2.0, -1.0, 0.0, -1.0, 2.0, -1.0, 0.0, -1.0, 2.0};
  double inv[9];
  double product[9];
  CHECK(mcq_mat_inverse(a, inv, 3));
  mcq_mat_mul(a, inv, product, 3);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      CHECK_NEAR(product[i * 3 + j], i == j ? 1.0 : 0.0, 1e-12);
    }
  }
  return 0;
}

static int test_inverse_needs_pivoting(void)
{
  // A zero in the first pivot position: without partial pivoting this divides
  // by zero and the filter silently produces nonsense.
  const double a[4] = {0.0, 1.0, 1.0, 0.0};
  double inv[4];
  CHECK(mcq_mat_inverse(a, inv, 2));
  CHECK_NEAR(inv[0], 0.0, 1e-12);
  CHECK_NEAR(inv[1], 1.0, 1e-12);
  CHECK_NEAR(inv[2], 1.0, 1e-12);
  return 0;
}

static int test_singular_is_refused(void)
{
  const double a[4] = {1.0, 2.0, 2.0, 4.0};  // rank 1
  double inv[4];
  CHECK(!mcq_mat_inverse(a, inv, 2));
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(test_multiply_by_identity);
  RUN(test_multiply_known);
  RUN(test_multiply_transpose);
  RUN(test_add_and_symmetrize);
  RUN(test_inverse_2x2);
  RUN(test_inverse_3x3_round_trip);
  RUN(test_inverse_needs_pivoting);
  RUN(test_singular_is_refused);
  printf("%s: %d failure(s)\n", __FILE__, failures);
  return failures ? 1 : 0;
}
