# emu68-xhci

**emu68-xhci** is an Amiga OS driver for a XHCI USB controller.
The driver is based on [Das U-Boot](https://source.denx.de/u-boot/u-boot) XHCI driver.

This is currently a work in progress... you **will** get data corruption.
It's a Poseidon v4.5 driver. Current status is:
- enumerates root hub, USB 2.0 hub and devices
- external hubs work
- control, bulk, interrupt and RT isochronous (both directions) transfers seem to be working
- HID devices, thumb drives, audio cards are useable
- experimental support for SuperSpeed devices connected directly to root hub port

## Unimplemented / Planned Features

- the non-RT isochronous is not tested
- support for CM4 and OTG port on Pi4
- SuperSpeed support - hub devices

## Requirements

well, unsure yet. What I'm using:
- AmigaOS 3.2.3 + AmiKit 12.8.3 on an A1200
- Poseidon 4.5
- Pistorm32-lite with Raspberry Pi 4B (the USB controller here is Via VL805 XHCI on PCIe bus. **Note that CM4 is not yet supported**)
- Emu68... 1.1 or 1.0.99 This is necessary to set up MMU mapping for the PCIe BAR window into low 4GB.
- gic400.library - https://github.com/rondoval/emu68-gic400-library/releases

## Building

TBD
