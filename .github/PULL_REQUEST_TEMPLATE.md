<!-- Thanks for contributing! Keep PRs focused on one thing — see CONTRIBUTING.md. -->

## Summary
<!-- What does this change and why? -->

## Type of change
- [ ] Bug fix (no user-visible behavior change beyond the fix)
- [ ] New feature / behavior
- [ ] Refactor (no behavior change)
- [ ] Tests only
- [ ] CI / tooling / docs
- [ ] Breaking change (track files, log format, BLE protocol, or a removed mode)

## How it was verified
<!-- Be specific. -->
- [ ] Host unit tests pass (`ctest --test-dir tests/build`)
- [ ] `clang-tidy` clean
- [ ] Compiles for the XIAO nRF52840 Sense
- [ ] Tested on real hardware — describe what you exercised:
<!-- e.g. "ran a session, confirmed .dovex logs + instant replay, BLE download round-trip" -->

## Checklist
- [ ] `CHANGELOG.md` updated under `[Unreleased]` (if user-visible)
- [ ] `ARCHITECTURE.md` / `CLAUDE.md` updated (if a module or interface changed)
- [ ] New testable logic has a matching test in `tests/`
- [ ] Branch is focused — refactors / behavior / tests are not mixed together

## Related issues
<!-- e.g. Closes #123 -->
