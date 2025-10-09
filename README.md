# emu68-xhci

**emu68-xhci** is an Amiga OS driver for a XHCI USB controller.
The driver is based on [Das U-Boot](https://source.denx.de/u-boot/u-boot) XHCI driver.

This is currently a work in progress... you **will** get data corruption.
It's a Poseidon v4.5 driver. Current status is:
- it's able to enumerate the root hub, USB 2.0 hub and devices
- bulk, interrupt and control transfers are somewhat working
- whoa, I can type on USB keyboard and letters appear :)
- whoa, I just transferred a file to a PC using a thumb drive... and it is actually readable :)
- error recovery is broken. one failed request and it's down
- no isochronous transfers

It's slow; there's a lot of debug enabled. Not providing a binary build yet.
It's, let's say, a bit unorthodox in that it ignores certain fields in Poseidon requests and uses data sniffed from the traffic instead.

## Features

- control, bulk and interrupt traffic

## Unimplemented / Planned Features

- isochronous transfers
- interrupts
.. likely a lot more

## Requirements

well, unsure yet. What I'm using:
- AmigaOS 3.2.3 on an A1200
- Poseidon 4.5
- Pistorm with Raspberry Pi 4B (the USB controller here is Via VL805 XHCI on PCIe bus)
- Emu68... needs to be patched: https://github.com/michalsc/Emu68/pull/306 This is to set up MMU mapping for a BAR window.

## Building

TBD


