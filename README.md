# emu68-xhci

**emu68-xhci** is an Amiga OS driver for a XHCI USB controller.
The driver is based on [Das U-Boot](https://source.denx.de/u-boot/u-boot) XHCI driver.

This is currently a work in progress... you **will** get data corruption.
It's a Poseidon v4.5 driver. Current status is:
- enumerates root hub, USB 2.0 hub and devices
- external hubs work
- control, bulk, interrupt and isochronous (including RT) transfers seem to be working
- HID devices, thumb drives, audio cards are useable
- stall recovery added, but overall error handling is still fragile

It's slow; there's a lot of debug enabled. Not providing a binary build yet.
It's, let's say, a bit unorthodox in that it ignores certain fields in Poseidon requests and uses data sniffed from the traffic instead.

## Unimplemented / Planned Features

- the non-RT isochronous is not tested
- error recovery needs more work
- performance improvements, refactoring
- sleep states
- support for CM4
.. likely a lot more

## Requirements

well, unsure yet. What I'm using:
- AmigaOS 3.2.3 on an A1200
- Poseidon 4.5
- Pistorm with Raspberry Pi 4B (the USB controller here is Via VL805 XHCI on PCIe bus. Note that CM4 is currently not supported)
- Emu68... needs to be patched: https://github.com/michalsc/Emu68/pull/306 This is to set up MMU mapping for a BAR window.

## Building

TBD


