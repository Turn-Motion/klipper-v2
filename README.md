Welcome to the Klipper project!

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

https://www.klipper3d.org/

The Klipper firmware controls 3d-Printers. It combines the power of a
general purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more
information on why you should use the Klipper software.

Start by [installing Klipper software](https://www.klipper3d.org/Installation.html).

Klipper software is Free Software. See the [license](COPYING) or read
the [documentation](https://www.klipper3d.org/Overview.html). We
depend on the generous support from our
[sponsors](https://www.klipper3d.org/Sponsors.html).


# Load Cell Community Testing Branch

If you are testing load cells in klipper this is the branch to use!

Please review the documentation: [Load Cell](./docs/Load_Cell.md)

There is a debugging tool that will let you see the load cell output and probing taps:[Klipper Load Cell Debugging Tool](https://observablehq.com/@garethky/klipper-load-cell-debugging-tool). If you are testing you should get this set up to see how things are working.

#### Progress Tracker

This is being slowly shipped to klipper's mainline branch. It's a very large change and has to be broken up into multiple PRs to make reviewing easier:

##### PRs to klipper Mainline

1. ✅ [Bulk ADC Sensors](https://github.com/Klipper3d/klipper/pull/6555) - Shipped
2. ✅ [Add input_mux and vref options to ADS1220 sensor](https://github.com/Klipper3d/klipper/pull/6713) - Shipped
3. ☑️ [PR: Enable multiple z_thermal_adjust sections](https://github.com/Klipper3d/klipper/pull/6855) - Active
4. ✅ [Load cell gram scale](https://github.com/Klipper3d/klipper/pull/6729) - Shipped
5. ✅ [PR: Load Cell Probe](https://github.com/Klipper3d/klipper/pull/6871) - Shipped
6. ☑️ [PR: Load Cell Tap Analysis](https://github.com/Klipper3d/klipper/pull/) - Active
7. ☑️ [PR: Adaptive horizontal_move_z](https://github.com/Klipper3d/klipper/pull/6933]) - Active

##### PRs not yet submitted

2. `average_delta` metric for `PROBE_ACCURACY` command

---

## Feature Updates

### ADS1220 Driver Improvements (29.07.2025)

- Added robust SPI communication with retry logic and improved error handling
- Added detailed debug logging (enabled by default) to help diagnose communication issues
- Reduced default SPI speed for better reliability with problematic hardware
- Improved reset sequence with proper delays and verification
- Added configurable SPI speed option (`spi_speed` parameter)

---