# emu68-xhci

**emu68-xhci** is an Amiga OS driver for XHCI USB controllers.
The driver is based on [Das U-Boot](https://source.denx.de/u-boot/u-boot) XHCI driver.

This is currently a work in progress... you **may** get data corruption.
The driver has got a Poseidon compatible API, although it operates at a different level than a typical Poseidon driver. Current status is:
- enumerates root hub, USB 2.0 hub and devices
- external hubs work
- control, bulk, interrupt and RT isochronous (both directions) transfers seem to be working
- HID devices, thumb drives, audio cards are useable
- experimental support for SuperSpeed devices - note this is for Pi 4B only, as CM4 has got a single 2.0 port.

**Breaking change**
The unit numbering was changed in order to support the OTG port.
From now on:
- unit 0 is the OTG port
- unit 1 and up are PCIe devices

So eg. for stock PI4B unit 0 is the OTG port and unit 1 is VL805 with four USB-A ports.
For CM4 unit 0 is the OTG port and there is no unit 1 unless you connect something to PCIe.

The OTG port requires `otg_mode=1` in config.txt

## Unimplemented / Planned Features

- the non-RT isochronous is not tested
- fix for RT isochronous audio glitches
- SuperSpeed support
- AHI 4.x support
- KS 3.1 support

## Requirements

well, unsure yet. What I'm using:
- AmigaOS 3.2.3 + AmiKit 12.8.3 on an A1200
- Poseidon 4.5
- Pistorm32-lite with:
    - Raspberry Pi 4B - both the VIA VL805 USB controller on PCIe bus and the OTG port
    - CM4
- Emu68... 1.1 or 1.0.99 This is necessary to set up MMU mapping for the PCIe BAR window into low 4GB.
- gic400.library - https://github.com/rondoval/emu68-gic400-library/releases

## Building

TBD
