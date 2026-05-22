# Contributing to BirdsEye

Thanks for your interest in improving BirdsEye! This guide covers how to
build the firmware, run the tests, and get a change merged.

By participating you agree to abide by our [Code of Conduct](CODE_OF_CONDUCT.md).

## Before you start

- For anything beyond a small fix, open an issue first so we can agree on
  the approach before you invest time.
- If you found a security problem, **do not open a public issue** — see
  [SECURITY.md](SECURITY.md).
- New to the codebase? Read [ARCHITECTURE.md](ARCHITECTURE.md) for the
  subsystem layout and the design decisions behind the tricky parts.

## Project layout

The Arduino sketch lives in [`BirdsEye/`](BirdsEye) (the folder name must
match `BirdsEye.ino`). Each subsystem is an `.ino` module with a matching
`.h` describing its public surface. Pure, Arduino-free logic lives in
`BirdsEye/*.{h,cpp}` so it can be unit-tested on a desktop — see
ARCHITECTURE.md for the full map.

## Building the firmware

**Target board:** Seeed XIAO nRF52840 Sense.

### Arduino IDE
1. Install the **"Seeed nRF52 Boards"** package (the non-mbed variant — the
   mbed one uses ArduinoBLE instead of Bluefruit and won't compile).
2. Install the libraries listed in the README's *Required Libraries* table.
3. Open `BirdsEye/BirdsEye.ino`, select **Seeed XIAO nRF52840 Sense**, and
   compile.

### arduino-cli
The exact invocation CI uses is in
[`.github/workflows/compile-sketch.yml`](.github/workflows/compile-sketch.yml)
— copy the platform/library install steps from there.

## Running the host tests

The pure-logic units build and run on any machine with CMake ≥ 3.14 and a
C++17 compiler — no Arduino tooling needed:

```sh
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
```

Run the binary directly for per-case output or to filter:

```sh
./tests/build/birdseye_tests
./tests/build/birdseye_tests -tc='haversine*'
```

If you change or add logic in a `BirdsEye/*.cpp` pure unit, add or update
the matching `tests/<unit>_test.cpp`. See [`tests/README.md`](tests/README.md).

## Static analysis

CI runs `clang-tidy` over the pure units with warnings-as-errors. To run it
locally:

```sh
cmake -S tests -B tests/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
clang-tidy -p tests/build BirdsEye/haversine.cpp BirdsEye/gps_time.cpp \
  BirdsEye/gps_validation.cpp BirdsEye/dovex_header.cpp \
  BirdsEye/filename_validator.cpp
```

Prefer fixing a finding over suppressing it. If a suppression is genuinely
correct (e.g. an intentional non-null-terminated buffer), use a targeted
`// NOLINT(check-name)` with a comment explaining why.

## Code coverage

Coverage is measured **self-hosted** — no third-party service ever receives
our data. The [`coverage`](.github/workflows/coverage.yml) workflow builds
the tests with gcov instrumentation, runs `gcovr` over the host-testable
`BirdsEye/*.cpp` units, and:

- **fails the job** if line coverage drops below the floor,
- publishes a live `coverage` badge (a shields.io endpoint JSON committed to
  the orphan `badges` branch on default-branch pushes),
- posts a per-file summary as a sticky comment on each PR.

The floor is a single env var at the top of `coverage.yml`:

```yaml
env:
  COVERAGE_MIN: 1   # raise as coverage grows
```

It's set deliberately low so it passes today; **raise it as coverage grows,
never lower it to make a red build pass.** Reproduce locally:

```sh
cmake -S tests -B tests/build -DENABLE_COVERAGE=ON
cmake --build tests/build --parallel
ctest --test-dir tests/build
gcovr --root . --filter 'BirdsEye/.*\.cpp$' --print-summary
```

## Pull request workflow

1. Branch off `master` with a descriptive name (`feat/...`, `fix/...`,
   `refactor/...`, `docs/...`, `ci/...`).
2. Keep each PR focused on one thing. Split refactors from behavior
   changes and from new tests so each can be reviewed — and reverted —
   independently.
3. Write clear commit messages that explain the *why*, not just the *what*.
4. Add a `CHANGELOG.md` entry under `[Unreleased]` for anything
   user-visible.
5. Make sure all CI checks pass: **compile-sketch** (+ flash-size gate),
   **arduino-lint**, **unit-tests**, **clang-tidy**, **coverage**.
6. Open the PR against `master` and describe what changed and how you
   verified it. Hardware-affecting changes should say what you tested on a
   real device.

## Code style & conventions

These are the conventions the existing code follows — match them:

- **No Arduino `String` in hot paths** — heap fragmentation is a real risk
  on 256 KB. Use fixed `char` buffers + `snprintf`.
- **All SD access goes through `acquireSDAccess()` / `releaseSDAccess()`** —
  the BLE callback runs in a separate FreeRTOS task and SdFat is not
  thread-safe.
- **Never call `analogRead()` on any GPIO** — on the nRF52840 it permanently
  disables that pin's digital input buffer, and every analog-capable pin on
  the XIAO is a critical digital function. Use `micros()` or the hardware
  RNG for entropy.
- **`TIMER3` is reserved** for the GPS serial-buffer ISR. Use `TIMER4` if you
  need another hardware timer (`TIMER0` belongs to the SoftDevice).
- Keep ISRs trivially short; do real work in the matching `*_LOOP()`.
- Put new unit-testable logic in a pure `BirdsEye/*.cpp` (Arduino-free) and
  cover it with a test.
- Update [ARCHITECTURE.md](ARCHITECTURE.md) and the file map in `CLAUDE.md`
  when you add/remove a module or change a subsystem interface.

## License

By contributing you agree that your contributions are licensed under the
project's [GPL v3 license](LICENSE).
