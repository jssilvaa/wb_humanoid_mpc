# OCS2 clang/macOS compatibility patches

The standalone (non-ROS 2) build (`-DBUILD_OCS2=ON`) compiles the OCS2 packages in
`lib/ocs2_ros2/` (a submodule) with **Apple clang**, which is stricter than the gcc
the OCS2 ament build assumes. This file tracks the source patches applied to the
submodule for clang-compat, so they can be reproduced / turned into a patch set or
an `ocs2_ros2` fork.

**Portability goal:** build on **both** Ubuntu (gcc) and macOS (clang). Each patch
below is a *portable* standard-compliant rewrite (compiles on gcc too) unless it is
explicitly wrapped in `#if defined(__APPLE__)`.

To regenerate the actual diff at any time: `git -C lib/ocs2_ros2 diff`.

---

## 1. `ocs2_core/src/penalties/MultidimensionalPenalty.cpp` — portable
Explicit specializations of the templated constructors were written gcc-style with
explicit template args on the constructor name:
`MultidimensionalPenalty::MultidimensionalPenalty<T>(...)`. Clang rejects naming
explicit template args on a constructor ("constructor name rather than a type").
**Fix (portable):** drop the `<T>`; the parameter type deduces it. 4 spots (the
`std::vector<std::unique_ptr<T>>` and `std::unique_ptr<T>` ctors, for
`T = augmented::AugmentedPenaltyBase` and `T = PenaltyBase`):
```
MultidimensionalPenalty::MultidimensionalPenalty<T>(...)  ->  MultidimensionalPenalty::MultidimensionalPenalty(...)
```

## 2. `ocs2_sqp/ocs2_sqp/src/SqpSolver.cpp` — portable
`std::chrono::high_resolution_clock::to_time_t(...)` (logging timestamp, ~line 96).
On libstdc++ `high_resolution_clock` aliases `system_clock` (has `to_time_t`); on
libc++ (macOS) it aliases `steady_clock` (no `to_time_t`). **Fix (portable):**
```
std::chrono::high_resolution_clock::to_time_t(... high_resolution_clock::now())  ->  std::chrono::system_clock::to_time_t(... system_clock::now())
```
