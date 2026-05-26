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

## 3. `std::result_of` → `std::invoke_result` (C++20) — portable
`std::result_of` was removed in C++20 (gone on libc++). OCS2 used it in 3 headers;
replaced with `std::invoke_result` (available since C++17, works in both standards):
- `ocs2_core/include/ocs2_core/thread_support/ThreadPool.h` (3 spots): `std::result_of<Functor(int)>::type` → `std::invoke_result<Functor, int>::type`
- `ocs2_core/include/ocs2_core/misc/LinearInterpolation.h` (2 spots) and `.../misc/implementation/LinearInterpolation.h` (2 spots): `std::result_of<AccessFun(const std::vector<Data, Alloc>&, size_t)>::type` → `std::invoke_result<AccessFun, const std::vector<Data, Alloc>&, size_t>::type`

**Why C++20 (not the earlier C++17):** the standalone build was unified to **C++20** so that
`CentroidalMpcMrtJointController` can include BOTH OCS2 headers and `robot_model` headers
(which use C++20 `concepts`/`span` via `IDMapBase.h`) in one TU — the closed-loop bridge
needs both. The controller also uses `std::jthread` (C++20). This patch is what makes OCS2
C++20-clean. Verified: full OCS2+MPC rebuild at C++20 is clean, G1 solve unchanged (dyn-viol 9.1e-6).

## 4. `ocs2_core/.../CppAdInterface.cpp` — genuine OOB bug (NOT clang-specific) — portable
`CppAdInterface::getGaussNewtonApproximation` builds the sparse Gauss-Newton Hessian
`H = JᵀJ` by walking the sparse-Jacobian `rows[]`/`cols[]` arrays (length `nnzJacobian_`).
The inner off-diagonal loop reads `rows[j]` with **no upper bound on `j`**:
```
size_t j = i + 1;
while (rows[j] == row_i) { ... ++j; }   // rows[] has only nnzJacobian_ entries
```
For the last nonzero (`i = nnzJacobian_-1`) this reads `rows[nnzJacobian_]` — out of bounds.
Latent: usually the OOB read returns garbage `≠ row_i` and the loop exits harmlessly, so it
never crashed with the shipped cost terms. **Enabling the ICP/capturability cost changed the
Jacobian sparsity → the OOB read chained into unmapped memory → SIGSEGV** in every SQP worker
(found via lldb: `getGaussNewtonApproximation` +1740, EXC_BAD_ACCESS). **Fix (portable):**
```
while (j < nnzJacobian_ && rows[j] == row_i) { ... }
```
This is a real upstream bug, not a clang/macOS issue — worth upstreaming to ocs2_ros2.
