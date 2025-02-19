## m33 firmware at

This firmware has three purposes:
- bridge m33 hardware with linux
- minimize power consumption during suspend
- allow user code to run

### Design overview

#### Power management

Power management is done in `main.c` as well as `lpm.c`

#### Hardware bridge

Hardware bridge to linux is split by component:
- `app_srtm.c`: main linux communication driver
- `srtm/services/*`: individual protocols for each driver
- `app_*.c`: hardware implementation for each driver

#### User defined code

The `custom` directory contains hooks that can be customized for specific applications.

- `custom/cli_custom.c`: add commands for CLI (ttyrpmsg0 from linux or console)
- `custom/app_tty_custom.c`: free form console that can be bound as ttyrpmsgX through linux DTS

### Acknowledgement

This was originally based on NXP's `power_mode_switch` demo as found in
https://github.com/nxp-mcuxpresso/mcux-sdk-examples
`evkmimx8ulp/demo_apps/power_mode_switch` directory.
