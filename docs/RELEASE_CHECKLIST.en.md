# Release and Version Maintenance Checklist

Run this checklist before every important push, version tag, or GitHub Release.

## Code and builds

- [ ] `git status` contains only intentional release files.
- [ ] `git diff --check` reports no whitespace errors.
- [ ] Host tests pass with both GCC and Clang under ASan/UBSan.
- [ ] Arm GNU Toolchain compiles the portable core for Cortex-M4 with warnings as errors.
- [ ] The installed CMake package works through `find_package(BNO085 CONFIG)`.
- [ ] Keil `Rebuild all target files` finishes with zero errors and zero warnings.
- [ ] The `.ioc` SPI mode, prescaler, DMA, and GPIO settings match generated code.
- [ ] Build artifacts, user settings, logs, and IDE state are ignored.

## Hardware validation

- [ ] A cold boot reads the Product ID reliably.
- [ ] Enabled reports stream for at least 60 seconds without growing timeout,
      invalid-packet, I/O-error, or recovery counters.
- [ ] Stationary acceleration magnitude is close to 9.81 m/s².
- [ ] Yaw, roll, and pitch directions match the documented coordinate convention.
- [ ] Disconnect/reconnect behavior follows the documented recovery policy.
- [ ] If SPI timing changed, a scope or logic analyzer confirms SCK does not exceed 3 MHz.

## Documentation, privacy, and licensing

- [ ] Wiring, public APIs, units, coordinates, gravity inclusion, and current
      limitations are documented in English and Simplified Chinese.
- [ ] Release notes distinguish hardware-verified behavior from host-only tests.
- [ ] User names, private absolute paths, debugger serial numbers, credentials,
      keys, and other private data have been removed.
- [ ] Original STM32 HAL/CMSIS notices and third-party licenses remain intact.
- [ ] Screenshots, datasheets, and other assets are redistributable; otherwise
      link to the official source instead of copying them.

## GitHub release

- [ ] Repository owner, visibility, description, topics, and remote URL are correct.
- [ ] CI and the API-documentation deployment both pass for the release commit.
- [ ] The version in `VERSION`, CMake, the public header, and changelog agrees.
- [ ] The tag uses the `vMAJOR.MINOR.PATCH` format and points to the tested commit.
- [ ] Release notes name supported hardware, toolchains, features, limitations,
      and exact validation results without exposing private machine details.
