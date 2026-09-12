// Minimal test framework: no dependencies, readable failures, one binary per area.
#ifndef GATEWAY_TEST_CHECK_H
#define GATEWAY_TEST_CHECK_H

#include <math.h>
#include <stdio.h>

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                                \
    }                                                                          \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                                   \
  do {                                                                                          \
    double _a = (double)(a), _b = (double)(b);                                                  \
    if (fabs(_a - _b) > (double)(tol)) {                                                        \
      fprintf(                                                                                  \
        stderr, "%s:%d: CHECK_NEAR failed: %s = %g, %s = %g\n", __FILE__, __LINE__, #a, _a, #b, \
        _b);                                                                                    \
      return 1;                                                                                 \
    }                                                                                           \
  } while (0)

#define RUN(test)                                    \
  do {                                               \
    int _r = test();                                 \
    printf("%-56s %s\n", #test, _r ? "FAIL" : "ok"); \
    failures += _r;                                  \
  } while (0)

#endif  // GATEWAY_TEST_CHECK_H
