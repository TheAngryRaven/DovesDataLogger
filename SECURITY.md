# Security Policy

## Reporting a vulnerability

**Please do not report security issues through public GitHub issues.**

Instead, use GitHub's private vulnerability reporting:

1. Go to the [Security tab](https://github.com/TheAngryRaven/DovesDataLogger/security)
   of this repository.
2. Click **Report a vulnerability** to open a private advisory.

If you can't use that, open a regular issue that says only "security
contact requested" with no details, and a maintainer will arrange a
private channel.

Please include, where possible:
- the affected version / commit,
- a description of the issue and its impact,
- steps to reproduce or a proof of concept,
- any suggested remediation.

We'll acknowledge your report, work with you on a fix, and credit you in
the release notes unless you'd prefer to stay anonymous.

## Scope

BirdsEye is firmware for a battery-powered, SD-logging, Bluetooth LE
device. The most relevant attack surfaces:

- **Bluetooth LE command interface** — file listing/download/delete,
  settings read/write, and track upload/download/delete. Filenames from
  BLE are validated against path traversal and FAT-unsafe input before any
  SD access (see `BirdsEye/filename_validator.{h,cpp}`).
- **SD card contents** — track JSON and settings JSON are parsed on-device;
  malformed input should fail closed, not crash or corrupt the card.
- **Logged data** — `.dovex` files contain GPS traces (i.e. where the
  device has physically been).

## Known posture

These are understood limitations, not undisclosed vulnerabilities:

- **BLE is currently unauthenticated.** When the device is advertising,
  any client in radio range can list and download logs. A PIN-gated
  access mode ("Pin Lock") is on the roadmap; until then, treat BLE as
  open while it's enabled and turn it off when not in use.

## Supported versions

This is a hobbyist project maintained on a best-effort basis. Security
fixes target the latest release on `master`; older tagged releases are not
back-patched.
