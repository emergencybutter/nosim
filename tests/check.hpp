#pragma once
// Minimal assertion harness so the tests need no third-party dependency.

#include <cmath>
#include <cstdio>

namespace check_detail {
inline int failures = 0;
inline int checks = 0;
}  // namespace check_detail

#define CHECK(cond)                                                                  \
    do {                                                                             \
        ++check_detail::checks;                                                      \
        if (!(cond)) {                                                               \
            ++check_detail::failures;                                                \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                            \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                                     \
    do {                                                                                      \
        ++check_detail::checks;                                                               \
        const double _a = (actual), _e = (expected), _t = (tol);                              \
        if (!(std::fabs(_a - _e) <= _t)) {                                                    \
            ++check_detail::failures;                                                         \
            std::fprintf(stderr, "%s:%d: CHECK_NEAR failed: %s = %.10g, expected %.10g ± %g\n", \
                         __FILE__, __LINE__, #actual, _a, _e, _t);                            \
        }                                                                                     \
    } while (0)

#define TEST_MAIN(run_fn)                                                                   \
    int main() {                                                                            \
        run_fn();                                                                           \
        std::printf("%d checks, %d failures\n", check_detail::checks, check_detail::failures); \
        return check_detail::failures == 0 ? 0 : 1;                                         \
    }
