#ifndef __DRIVER_IFACE_H
#define __DRIVER_IFACE_H

#include <emu_types.h>
#include <exec/io.h>

#pragma pack(2)

struct USBSetupPacket
{
    u8 bmRequestType;
    u8 bRequest;
    u16 wValue;
    u16 wIndex;
    u16 wLength;
};

struct USBBufferRequest
{
    u8 *data;
    u32 length;
    u16 frame;
    u16 flags;
};

struct USBRealtimeHooks
{
    struct Node *node;
    struct Hook *input_request_hook;
    struct Hook *output_request_hook;
    struct Hook *input_done_hook;
    struct Hook *output_done_hook;
    u32 max_output_prefetch;
};

struct USBIORequest
{
    struct IORequest req;
    u16 flags;
    u16 state;
    u16 direction;
    u16 virtual_address;
    u16 endpoint;
    u16 reserved1;
    u32 actual_length;
    u32 data_buffer_length;
    APTR data_buffer;
    u16 reserved2;
    u32 timeout;
    struct USBSetupPacket setup;
    u32 reserved3;
    u16 reserved4;
    u16 usb_frame;
    u32 reserved5;
    u32 driver_private_flags;
    void *driver_private_dma_address;
};

#define DIRECTION_IN 2
#define DIRECTION_OUT 1

#define DRIVER_FLAG_TIMEOUT_DEFINED (1 << 3)
#define DRIVER_FLAG_IGNORE_SHORT_TRANSFER (1 << 4)

/* driver states */
#define DRIVER_STATE_OPERATIONAL (1 << 0)
#define DRIVER_STATE_SUSPENDED (1 << 2)
#define DRIVER_STATE_RESUMING (1 << 1)
#define DRIVER_STATE_RESETING (1 << 3)

/* driver tags */
#define TAG_DEVICE_VENDOR (TAG_USER + 0x4721)
#define TAG_DEVICE_PRODUCT (TAG_USER + 0x4722)
#define TAG_DEVICE_VERSION (TAG_USER + 0x4723)
#define TAG_DEVICE_REVISION (TAG_USER + 0x4724)

#define TAG_DRIVER_STATE (TAG_USER + 0x4712)
#define TAG_DRIVER_DESCRIPTION (TAG_USER + 0x4725)
#define TAG_DRIVER_LICENSE (TAG_USER + 0x4726)
#define TAG_DRIVER_VERSION (TAG_USER + 0x4731)
#define TAG_DRIVER_FEATURES (TAG_USER + 0x4732)

/* driver feature flags */
#define DRIVER_FEAT_USB2 (1 << 0)
#define DRIVER_FEAT_USB3 (1 << 31)
#define DRIVER_FEAT_QUICK_IO (1 << 3)
#define DRIVER_FEAT_ISOCHRONOUS (1 << 1)
#define DRIVER_FEAT_ISOCHRONOUS_HOOKS (1 << 2)

/* driver commands */
#define CMD_DEVICE_QUERY (CMD_NONSTD + 0)
#define CMD_DEVICE_RESET (CMD_NONSTD + 1)
#define CMD_DEVICE_RESUME (CMD_NONSTD + 2)
#define CMD_REQUEST_CONTROL (CMD_NONSTD + 3)
#define CMD_REQUEST_ISOCHRONOUS (CMD_NONSTD + 4)
#define CMD_REQUEST_INTERRUPT (CMD_NONSTD + 5)
#define CMD_REQUEST_BULK (CMD_NONSTD + 6)
#define CMD_REGISTER_ISOCHRONOUS_HOOKS (CMD_NONSTD + 7)
#define CMD_UNREGISTER_ISOCHRONOUS_HOOKS (CMD_NONSTD + 8)
#define CMD_START_REALTIME_ISOCHRONOUS (CMD_NONSTD + 9)
#define CMD_STOP_REALTIME_ISOCHRONOUS (CMD_NONSTD + 10)

/* error codes */
#define ERR_NO_ERROR 0
#define ERR_SHORT_TRANSFER 9

#define ERR_ALLOC_ERROR 12
#define ERR_HCI_ERROR 3
#define ERR_BAD_PARAMETERS 11

#define ERR_TIMEOUT 6
#define ERR_DEVICE_STALL 4
#define ERR_DEVICE_BABBLE 13
#define ERR_ISOC_OVERRUN 7

#pragma pack()

#endif