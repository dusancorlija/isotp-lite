/* Minimal test harness: no dependencies, works on host and semihosted targets. */
#ifndef CHECK_H
#define CHECK_H

#include <math.h>
#include <stdio.h>

static int check_failures;
static int check_count;

#define CHECK(cond)                                                            \
    do {                                                                       \
        check_count++;                                                         \
        if (!(cond)) {                                                         \
            check_failures++;                                                  \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                      \
    } while (0)

#define CHECK_EQ(a, b)                                                         \
    do {                                                                       \
        long long va_ = (long long)(a), vb_ = (long long)(b);                  \
        check_count++;                                                         \
        if (va_ != vb_) {                                                      \
            check_failures++;                                                  \
            printf("  FAIL %s:%d  %s == %s  (%lld vs %lld)\n", __FILE__,       \
                   __LINE__, #a, #b, va_, vb_);                                \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                  \
    do {                                                                       \
        double va_ = (a), vb_ = (b);                                           \
        check_count++;                                                         \
        if (!(fabs(va_ - vb_) <= (tol))) {                                     \
            check_failures++;                                                  \
            printf("  FAIL %s:%d  |%s - %s| <= %g  (%.9g vs %.9g)\n",          \
                   __FILE__, __LINE__, #a, #b, (double)(tol), va_, vb_);       \
        }                                                                      \
    } while (0)

#define RUN(test)                                                              \
    do {                                                                       \
        int before_ = check_failures;                                          \
        test();                                                                \
        printf("%s %s\n", check_failures == before_ ? "ok  " : "FAIL", #test); \
    } while (0)

#define CHECK_DONE()                                                           \
    (printf("\n%d checks, %d failed\n", check_count, check_failures),          \
     check_failures ? 1 : 0)

#endif /* CHECK_H */
