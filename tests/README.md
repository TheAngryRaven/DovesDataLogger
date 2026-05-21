# Host-side unit tests

Lightweight test harness for the pure-logic units extracted from the
Arduino sketch. Runs on any machine with CMake ≥ 3.14 and a C++17
compiler — no Arduino tooling required.

## Layout

```
tests/
├── CMakeLists.txt          # host build
├── doctest.h               # vendored single-header framework (MIT, v2.4.11)
├── test_main.cpp           # defines doctest's main()
└── <unit>_test.cpp         # one file per unit under test
```

Each unit-under-test lives in `../BirdsEye/<unit>.{h,cpp}`. The same
`.cpp` is compiled by Arduino (firmware build) and by CMake (host test
build); there is no copy-paste.

## Running locally

```sh
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
```

Or run the binary directly to see per-test output:

```sh
./tests/build/birdseye_tests
./tests/build/birdseye_tests --help          # doctest CLI flags
./tests/build/birdseye_tests -tc='haversine*' # run a specific test case
```

## CI

Runs in `.github/workflows/unit-tests.yml` on every push and pull
request. The badge at the top of the project README reflects the
latest `master` run.

## Adding a test

1. Extract the function into `BirdsEye/<unit>.h` (declaration) and
   `BirdsEye/<unit>.cpp` (implementation). The `.cpp` must not pull
   in Arduino headers — use plain C++ / `<math.h>` / `<stdint.h>` /
   `<cstring>`. The corresponding `.ino` that used to define the
   function gets an `#include "<unit>.h"` instead.
2. Create `tests/<unit>_test.cpp`:
   ```cpp
   #include "doctest.h"
   #include "<unit>.h"

   TEST_CASE("…") {
       CHECK(...);
   }
   ```
3. Add the new sources to `tests/CMakeLists.txt`:
   ```cmake
   add_executable(birdseye_tests
     test_main.cpp
     <unit>_test.cpp
     ${BIRDSEYE_DIR}/<unit>.cpp
     ...
   )
   ```
