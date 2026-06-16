/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * xHCI register space: MMIO register block layouts, their bit definitions and
 * the raw/typed accessors.
 */

#ifndef XHCI_REGS_H_
#define XHCI_REGS_H_

#include <bits.h>
#include <iomem.h>

/* Max number of USB devices for any host controller - limit in section 6.1 */
#define MAX_HC_SLOTS 256
/* Section 5.3.3 - MaxPorts */
#define MAX_HC_PORTS 255

/*
 * These bits are Read Only (RO) and should be saved and written to the
 * registers: 0, 3, 10:13, 30
 * connect status, over-current status, port speed, and device removable.
 * connect status and port speed are also sticky - meaning they're in
 * the AUX well and they aren't changed by a hot, warm, or cold reset.
 */
#define XHCI_PORT_RO (BIT(0) | BIT(3) | (0xf << 10) | BIT(24) | BIT(30))
/*
 * These bits are RW; writing a 0 clears the bit, writing a 1 sets the bit:
 * bits 5:8, 9, 14:15, 25:27
 * link state, port power, port indicator state, "wake on" enable state
 */
#define XHCI_PORT_RWS ((0xf << 5) | BIT(9) | (0x3 << 14) | (0x7 << 25))
/*
 * These bits are RW; writing a 1 sets the bit, writing a 0 has no effect:
 * bit 4 (port reset)
 */
#define XHCI_PORT_RW1S (BIT(4) | BIT(31))
/*
 * These bits are RW; writing a 1 clears the bit, writing a 0 has no effect:
 * bits 1, 17, 18, 19, 20, 21, 22, 23
 * port enable/disable, and
 * change bits: connect, PED,
 * warm port reset changed (reserved zero for USB 2.0 ports),
 * over-current, reset, link state, and L1 change
 */
#define XHCI_PORT_RW1CS (BIT(1) | (0x7f << 17))
/*
 * Bit 16 is RW, and writing a '1' to it causes the link state control to be
 * latched in
 */
#define XHCI_PORT_RW BIT(16)
/*
 * These bits are Reserved Zero (RsvdZ) and zero should be written to them:
 * bits 2, 24, 28:31
 */
#define XHCI_PORT_RZ (BIT(2) | (0x3 << 28))

/*
 * XHCI Register Space.
 */
struct xhci_hccr
{
	u32 cr_capbase;
	u32 cr_hcsparams1;
	u32 cr_hcsparams2;
	u32 cr_hcsparams3;
	u32 cr_hccparams1;
	u32 cr_dboff;
	u32 cr_rtsoff;
	u32 cr_hccparams2;

/* hc_capbase bitmasks */
/* bits 7:0 - how long is the Capabilities register */
#define HC_LENGTH(p) XHCI_HC_LENGTH(p)
/* bits 31:16	*/
#define HC_VERSION(p) (((p) >> 16) & 0xffff)

/* HCSPARAMS1 - hcs_params1 - bitmasks */
/* bits 0:7, Max Device Slots */
#define HCS_MAX_SLOTS(p) (((p) >> 0) & 0xff)
#define HCS_SLOTS_MASK 0xffU
/* bits 8:18, Max Interrupters */
#define HCS_MAX_INTRS(p) (((p) >> 8) & 0x7ff)
/* bits 24:31, Max Ports - max value is 0x7F = 127 ports */
#define HCS_MAX_PORTS(p) (((p) >> 24) & 0xff)

/* HCSPARAMS2 - hcs_params2 - bitmasks */
/* bits 0:3, frames or uframes that SW needs to queue transactions
 * ahead of the HW to meet periodic deadlines */
#define HCS_IST(p) (((p) >> 0) & 0xf)
/* bits 4:7, max number of Event Ring segments, encoded as 2^n */
#define HCS_ERST_MAX(p) (1U << (((p) >> 4) & 0xf))
/* bits 21:25 Hi 5 bits of Scratchpad buffers SW must allocate for the HW */
/* bit 26 Scratchpad restore - for save/restore HW state - not used yet */
/* bits 27:31 Lo 5 bits of Scratchpad buffers SW must allocate for the HW */
#define HCS_MAX_SCRATCHPAD(p) ((((p) >> 16) & 0x3e0) | (((p) >> 27) & 0x1f))

/* HCSPARAMS3 - hcs_params3 - bitmasks */
/* bits 0:7, Max U1 to U0 latency for the roothub ports */
#define HCS_U1_LATENCY(p) (((p) >> 0) & 0xff)
/* bits 16:31, Max U2 to U0 latency for the roothub ports */
#define HCS_U2_LATENCY(p) (((p) >> 16) & 0xffff)

/* HCCPARAMS1 - hcc_params1 - bitmasks */
/* true: HC can use 64-bit address pointers */
#define HCC_64BIT_ADDR(p) ((p) & BIT(0))
/* true: HC can do bandwidth negotiation */
#define HCC_BANDWIDTH_NEG(p) ((p) & BIT(1))
/* true: HC uses 64-byte Device Context structures
 * FIXME 64-byte context structures aren't supported yet.
 */
#define HCC_64BYTE_CONTEXT(p) ((p) & BIT(2))
/* true: HC has port power switches */
#define HCC_PPC(p) ((p) & BIT(3))
/* true: HC has port indicators */
#define HCS_INDICATOR(p) ((p) & BIT(4))
/* true: HC has Light HC Reset Capability */
#define HCC_LIGHT_RESET(p) ((p) & BIT(5))
/* true: HC supports latency tolerance messaging */
#define HCC_LTC(p) ((p) & BIT(6))
/* true: no secondary Stream ID Support */
#define HCC_NSS(p) ((p) & BIT(7))
/* true: HC supports Parse All Event Data */
#define HCC_PAE(p) ((p) & BIT(8))
/* true: HC supports Stopped - Short Packet Capability */
#define HCC_SPC(p) ((p) & BIT(9))
/* true: HC supports Stopped EDTLA Capability */
#define HCC_SEC(p) ((p) & BIT(10))
/* true: HC supports Configure Frame ID Capability */
#define HCC_CFC(p) ((p) & BIT(11))
/* Max size for Primary Stream Arrays - 2^(n+1), where n is bits 12:15 */
#define HCC_MAX_PSA(p) (1 << ((((p) >> 12) & 0xf) + 1))
/* Extended Capabilities pointer from PCI base - section 5.3.6 */
#define HCC_EXT_CAPS(p) (((p) >> 16) & 0xffff)

/* HCCPARAMS2 - hcc_params2 - bitmasks */
/* U3 Entry Capability */
#define HCC_U3C(p) ((p) & BIT(0))
/* Configure Endpoint Command Max Exit Latency Too Large Capability (CMC) */
#define HCC_CMC(p) ((p) & BIT(1))
/* Force Save Context Capability (FSC) */
#define HCC_FSC(p) ((p) & BIT(2))
/* Compliance Transition Capability (CTC) */
#define HCC_CTC(p) ((p) & BIT(3))
/* Large ESIT Payload Capability (LEC) */
#define HCC_LEC(p) ((p) & BIT(4))
/* Configuration Information Capability (CIC) */
#define HCC_CIC(p) ((p) & BIT(5))
/* Extended TBC Capability (ETC) */
#define HCC_ETC(p) ((p) & BIT(6))
/* Extended TBC TRB Status Capability (ETC_TSC) */
#define HCC_ETC_TSC(p) ((p) & BIT(7))
/* Get/Set Extended Property Capability (GSC) */
#define HCC_GSC(p) ((p) & BIT(8))
/* Virtualization Based Trusted I/O Capability (VTC) */
#define HCC_VTC(p) ((p) & BIT(9))

/* db_off bitmask - bits 0:1 reserved */
#define DBOFF_MASK (~0x3U)

/* run_regs_off bitmask - bits 0:4 reserved */
#define RTSOFF_MASK (~0x1fU)
};

struct xhci_hcor_port_regs
{
	volatile u32 or_portsc;
	volatile u32 or_portpmsc;
	volatile u32 or_portli;
	volatile u32 or_porthlpmc; /* offset 0Ch: USB2 Port Hardware LPM Control (PORTHLPMC) */
};

struct xhci_hcor
{
	volatile u32 or_usbcmd;
	volatile u32 or_usbsts;
	volatile u32 or_pagesize;
	volatile u32 reserved_0[2];
	volatile u32 or_dnctrl;
	volatile u64 or_crcr;
	volatile u32 reserved_1[4];
	volatile u64 or_dcbaap;
	volatile u32 or_config;
	volatile u32 reserved_2[241];
	struct xhci_hcor_port_regs portregs[MAX_HC_PORTS];
};

/* USBCMD - USB command - command bitmasks */
/* start/stop HC execution - do not write unless HC is halted*/
#define CMD_RUN XHCI_CMD_RUN
/* Reset HC - resets internal HC state machine and all registers (except
 * PCI config regs).  HC does NOT drive a USB reset on the downstream ports.
 * The xHCI driver must reinitialize the xHC after setting this bit.
 */
#define CMD_RESET_USB BIT(1)
/* Event Interrupt Enable - a '1' allows interrupts from the host controller */
#define CMD_EIE XHCI_CMD_EIE
/* Host System Error Interrupt Enable - get out-of-band signal for HC errors */
#define CMD_HSEIE XHCI_CMD_HSEIE
/* bits 4:6 are reserved (and should be preserved on writes). */
/* light reset (port status stays unchanged) - reset completed when this is 0 */
#define CMD_LRESET BIT(7)
/* host controller save/restore state. */
#define CMD_CSS BIT(8)
#define CMD_CRS BIT(9)
/* Enable Wrap Event - '1' means xHC generates an event when MFINDEX wraps. */
#define CMD_EWE XHCI_CMD_EWE
/* MFINDEX power management - '1' means xHC can stop MFINDEX counter if all root
 * hubs are in U3 (selective suspend), disconnect, disabled, or powered-off.
 * '0' means the xHC can power it off if all ports are in the disconnect,
 * disabled, or powered-off state.
 */
#define CMD_PM_INDEX BIT(11)
/* Configure Endpoint Command Max Exit Latency Too Large Enable - xHCI 1.1+ (requires CMC in HCCPARAMS2) */
#define CMD_CME BIT(13)
/* bits 14:31 are reserved (and should be preserved on writes). */

/* USBSTS - USB status - status bitmasks */
/* HC not running - set to 1 when run/stop bit is cleared. */
#define STS_HALT BIT(0)
/* serious error, e.g. PCI parity error.  The HC will clear the run/stop bit. */
#define STS_FATAL BIT(2)
/* event interrupt - clear this prior to clearing any IP flags in IR set*/
#define STS_EINT BIT(3)
/* port change detect */
#define STS_PORT BIT(4)
/* bits 5:7 reserved and zeroed */
/* save state status - '1' means xHC is saving state */
#define STS_SAVE BIT(8)
/* restore state status - '1' means xHC is restoring state */
#define STS_RESTORE BIT(9)
/* true: save or restore error */
#define STS_SRE BIT(10)
/* true: Controller Not Ready to accept doorbell or op reg writes after reset */
#define STS_CNR XHCI_STS_CNR
/* true: internal Host Controller Error - SW needs to reset and reinitialize */
#define STS_HCE BIT(12)
/* bits 13:31 reserved and should be preserved */

/*
 * DNCTRL - Device Notification Control Register - dev_notification bitmasks
 * Generate a device notification event when the HC sees a transaction with a
 * notification type that matches a bit set in this bit field.
 */
#define DEV_NOTE_MASK (0xffff)
#define ENABLE_DEV_NOTE(x) (1 << (x))
/* Most of the device notification types should only be used for debug.
 * SW does need to pay attention to function wake notifications.
 */
#define DEV_NOTE_FWAKE ENABLE_DEV_NOTE(1)

/* CRCR - Command Ring Control Register - cmd_ring bitmasks */
/* bit 0 is the command ring cycle state */
/* stop ring operation after completion of the currently executing command */
#define CMD_RING_PAUSE BIT(1)
/* stop ring immediately - abort the currently executing command */
#define CMD_RING_ABORT BIT(2)
/* true: command ring is running */
#define CMD_RING_RUNNING BIT(3)
/* bits 4:5 reserved and should be preserved */
#define CMD_RING_RSVD_BITS (3 << 4)
#define CMD_RING_ADDR_MASK (CMD_RING_PAUSE | CMD_RING_ABORT | CMD_RING_RUNNING | CMD_RING_RSVD_BITS)

/* CONFIG - Configure Register - config_reg bitmasks */
/* bits 0:7 - maximum number of device slots enabled (NumSlotsEn) */
#define MAX_DEVS(p) ((p) & 0xff)
/* bits 8:31 - reserved and should be preserved */

/* PORTSC - Port Status and Control Register - port_status_base bitmasks */
/* true: device connected */
#define PORT_CONNECT BIT(0)
/* true: port enabled */
#define PORT_PE BIT(1)
/* bit 2 reserved and zeroed */
/* true: port has an over-current condition */
#define PORT_OC BIT(3)
/* true: port reset signaling asserted */
#define PORT_RESET BIT(4)
/* Port Link State - bits 5:8
 * A read gives the current link PM state of the port,
 * a write with Link State Write Strobe set sets the link state.
 */
#define PORT_PLS_MASK (0xfu << 5)
#define XDEV_U0 (0x0u << 5)
#define XDEV_U1 (0x1u << 5)
#define XDEV_U2 (0x2u << 5)
#define XDEV_U3 (0x3u << 5)
#define XDEV_DISABLED (0x4u << 5)
#define XDEV_RXDETECT (0x5u << 5)
#define XDEV_INACTIVE (0x6u << 5)
#define XDEV_POLLING (0x7u << 5)
#define XDEV_RECOVERY (0x8u << 5)
#define XDEV_HOTRESET (0x9u << 5)
#define XDEV_COMPLIANCE (0xau << 5)
#define XDEV_TESTMODE (0xbu << 5)
#define XDEV_RESUME (0xfu << 5)
/* true: port has power (see HCC_PPC) */
#define PORT_POWER BIT(9)
/* bits 10:13 indicate device speed:
 * 0 - undefined speed - port hasn't be initialized by a reset yet
 * 1 - full speed
 * 2 - low speed
 * 3 - high speed
 * 4 - super speed
 * 5-15 reserved
 */
#define DEV_SPEED_MASK (0xf << 10)
#define XDEV_FS (0x1 << 10)
#define XDEV_LS (0x2 << 10)
#define XDEV_HS (0x3 << 10)
#define XDEV_SS (0x4 << 10)
#define DEV_UNDEFSPEED(p) (((p) & DEV_SPEED_MASK) == (0x0 << 10))
#define DEV_FULLSPEED(p) (((p) & DEV_SPEED_MASK) == XDEV_FS)
#define DEV_LOWSPEED(p) (((p) & DEV_SPEED_MASK) == XDEV_LS)
#define DEV_HIGHSPEED(p) (((p) & DEV_SPEED_MASK) == XDEV_HS)
#define DEV_SUPERSPEED(p) (((p) & DEV_SPEED_MASK) == XDEV_SS)
/* Bits 20:23 in the Slot Context are the speed for the device */
#define SLOT_SPEED_FS (XDEV_FS << 10)
#define SLOT_SPEED_LS (XDEV_LS << 10)
#define SLOT_SPEED_HS (XDEV_HS << 10)
#define SLOT_SPEED_SS (XDEV_SS << 10)
/* Port Indicator Control */
#define PORT_LED_OFF (0 << 14)
#define PORT_LED_AMBER BIT(14)
#define PORT_LED_GREEN (2 << 14)
#define PORT_LED_MASK (3 << 14)
/* Port Link State Write Strobe - set this when changing link state */
#define PORT_LINK_STROBE BIT(16)
/* true: connect status change */
#define PORT_CSC BIT(17)
/* true: port enable change */
#define PORT_PEC BIT(18)
/* true: warm reset for a USB 3.0 device is done.  A "hot" reset puts the port
 * into an enabled state, and the device into the default state.  A "warm" reset
 * also resets the link, forcing the device through the link training sequence.
 * SW can also look at the Port Reset register to see when warm reset is done.
 */
#define PORT_WRC BIT(19)
/* true: over-current change */
#define PORT_OCC BIT(20)
/* true: reset change - 1 to 0 transition of PORT_RESET */
#define PORT_RC BIT(21)
/* port link status change - set on some port link state transitions:
 *  Transition				Reason
 *  --------------------------------------------------------------------------
 *  - U3 to Resume		Wakeup signaling from a device
 *  - Resume to Recovery to U0	USB 3.0 device resume
 *  - Resume to U0		USB 2.0 device resume
 *  - U3 to Recovery to U0	Software resume of USB 3.0 device complete
 *  - U3 to U0			Software resume of USB 2.0 device complete
 *  - U2 to U0			L1 resume of USB 2.1 device complete
 *  - U0 to U0 (???)		L1 entry rejection by USB 2.1 device
 *  - U0 to disabled		L1 entry error with USB 2.1 device
 *  - Any state to inactive	Error on USB 3.0 port
 */
#define PORT_PLC BIT(22)
/* port configure error change - port failed to configure its link partner */
#define PORT_CEC BIT(23)
/* bit 24 reserved */
/* wake on connect (enable) */
#define PORT_WKCONN_E BIT(25)
/* wake on disconnect (enable) */
#define PORT_WKDISC_E BIT(26)
/* wake on over-current (enable) */
#define PORT_WKOC_E BIT(27)
/* bits 28:29 reserved */
/* true: device is removable - for USB 3.0 roothub emulation */
#define PORT_DEV_REMOVE BIT(30)
/* Initiate a warm port reset - complete when PORT_WRC is '1' */
#define PORT_WR BIT(31)

/* We mark duplicate entries with -1 */
#define DUPLICATE_ENTRY ((u8)(-1))

/* Port Power Management Status and Control - port_power_base bitmasks */
/* Inactivity timer value for transitions into U1, in microseconds.
 * Timeout can be up to 127us.  0xFF means an infinite timeout.
 */
#define PORT_U1_TIMEOUT(p) ((p) & 0xff)
/* Inactivity timer value for transitions into U2 */
#define PORT_U2_TIMEOUT(p) (u32)(((p) & 0xff) << 8)
#define PORT_FLA BIT(16)
/* Bits 24:31 for port testing */

/* USB2 Protocol PORTSPMSC */
#define PORT_L1S_MASK 7
#define PORT_L1S_SUCCESS 1
#define PORT_RWE BIT(3)
#define PORT_HIRD(p) (((p) & 0xf) << 4)
#define PORT_HIRD_MASK (0xf << 4)
#define PORT_L1DS(p) (((p) & 0xff) << 8)
#define PORT_L1DS_MASK (0xff << 8)
#define PORT_HLE BIT(16)

/* USB2 Protocol PORTHLPMC (Port Hardware LPM Control) - xHCI 1.1 section 5.4.11.2 */
#define PORT_HIRDM(p) ((p) & 3)				   /* HIRD Mode: 0=HIRD, 1=BESL */
#define PORT_L1_TIMEOUT(p) (((p) & 0xff) << 2) /* L1 inactivity timeout, 256us units */
#define PORT_BESLD(p) (((p) & 0xf) << 10)	   /* deep BESL value */
/**
* struct xhci_intr_reg - Interrupt Register Set
* @irq_pending:	IMAN - Interrupt Management Register.  Used to enable
*			interrupts and check for pending interrupts.
* @irq_control:	IMOD - Interrupt Moderation Register.
*			Used to throttle interrupts.
* @erst_size:		Number of segments in the
			Event Ring Segment Table (ERST).
* @erst_base:		ERST base address.
* @erst_dequeue:	Event ring dequeue pointer.
*
* Each interrupter (defined by a MSI-X vector) has an event ring and an Event
* Ring Segment Table (ERST) associated with it.
* The event ring is comprised of  multiple segments of the same size.
* The HC places events on the ring and  "updates the Cycle bit in the TRBs to
* indicate to software the current  position of the Enqueue Pointer."
* The HCD (Linux) processes those events and  updates the dequeue pointer.
*/
struct xhci_intr_reg
{
	volatile __le32 irq_pending;
	volatile __le32 irq_control;
	volatile __le32 erst_size;
	volatile __le32 rsvd;
	volatile __le64 erst_base;
	volatile __le64 erst_dequeue;
};

/* irq_pending bitmasks */
#define ER_IRQ_PENDING(p) ((p) & 0x1)
/* bits 2:31 need to be preserved */
/* THIS IS BUGGY - FIXME - IP IS WRITE 1 TO CLEAR */
#define ER_IRQ_CLEAR(p) ((p) & 0xfffffffe)
#define ER_IRQ_ENABLE(p) ((ER_IRQ_CLEAR(p)) | 0x2)
#define ER_IRQ_DISABLE(p) ((ER_IRQ_CLEAR(p)) & ~(0x2U))

/* irq_control bitmasks */
/* Minimum interval between interrupts (in 250ns intervals).  The interval
 * between interrupts will be longer if there are no events on the event ring.
 * Default is 4000 (1 ms).
 */
#define ER_IRQ_INTERVAL_MASK (0xffffU)
/* Counter used to count down the time to the next interrupt - HW use only */
#define ER_IRQ_COUNTER_MASK (0xffffU << 16)

/* erst_size bitmasks */
/* Preserve bits 16:31 of erst_size */
#define ERST_SIZE_MASK (0xffffU << 16)

/* erst_dequeue bitmasks */
/* Dequeue ERST Segment Index (DESI) - Segment number (or alias)
 * where the current dequeue pointer lies.  This is an optional HW hint.
 */
#define ERST_DESI_MASK (0x7U)
/* Event Handler Busy (EHB) - is the event ring scheduled to be serviced by
 * a work queue (or delayed service routine)?
 */
#define ERST_EHB BIT(3)
#define ERST_PTR_MASK (0xfU)

/**
 * struct xhci_run_regs
 * @microframe_index:	MFINDEX - current microframe number
 *
 * Section 5.5 Host Controller Runtime Registers:
 * "Software should read and write these registers using only Dword (32 bit)
 * or larger accesses"
 */
struct xhci_run_regs
{
	__le32 microframe_index;
	__le32 rsvd[7];
	struct xhci_intr_reg ir_set[128];
};

/**
 * struct doorbell_array
 *
 * Bits  0 -  7: Endpoint target
 * Bits  8 - 15: RsvdZ
 * Bits 16 - 31: Stream ID
 *
 * Section 5.6
 */
struct xhci_doorbell_array
{
	volatile __le32 doorbell[256];
};

#define DB_VALUE(ep, stream) ((((ep) + 1) & 0xff) | ((stream) << 16))
#define DB_VALUE_HOST 0x00000000
/*
 * Registers should always be accessed with double word or quad word accesses.
 * Some xHCI implementations may support 64-bit address pointers.  Registers
 * with 64-bit address pointers should be written to with dword accesses by
 * writing the low dword first (ptr[0]), then the high dword (ptr[1]) second.
 * xHCI implementations that do not support 64-bit address pointers will ignore
 * the high dword, and write order is irrelevant.
 */
static inline u64 xhci_readq(__le64 volatile *regs)
{
	u32 *ptr = (u32 *)regs;
	u64 val_lo = mmio_read32(ptr);
	u64 val_hi = mmio_read32(ptr + 1);
	return val_lo + (val_hi << 32);
}

static inline void xhci_writeq(__le64 volatile *regs, const u64 val)
{
	u32 *ptr = (u32 *)regs;
	u32 val_lo = u64_lo32(val);
	/* FIXME */
	u32 val_hi = u64_hi32(val);
	mmio_write32(val_lo, ptr);
	mmio_write32(val_hi, ptr + 1);
}
/*************************************************************
	EXTENDED CAPABILITY DEFINITIONS
*************************************************************/

/* Extended capability register fields */
#define XHCI_EXT_CAPS_ID(p) (((p) >> 0) & 0xff)
#define XHCI_EXT_CAPS_NEXT(p) (((p) >> 8) & 0xff) // relative offset, in dwords, from this dword, to the next extended capability
#define XHCI_EXT_CAPS_VAL(p) ((p) >> 16)

/* Extended capability IDs - ID 0 reserved */
#define XHCI_EXT_CAPS_LEGACY 1
#define XHCI_EXT_CAPS_PROTOCOL 2
#define XHCI_EXT_CAPS_PM 3
#define XHCI_EXT_CAPS_VIRT 4
#define XHCI_EXT_CAPS_MSI 5
#define XHCI_EXT_CAPS_LOCAL_MEMORY 6
/* IDs 7-9 reserved */
#define XHCI_EXT_CAPS_DEBUG 10
#define XHCI_EXT_CAPS_MSIX 17

/* USB Legacy Support Capability - section 7.1.1 */
/* Add this offset, plus the value of xECP in HCCPARAMS to the base address */
#define XHCI_LEGACY_SUPPORT_OFFSET (0x00)
/* USB Legacy Support Capability - section 7.1.1 */
#define XHCI_HC_BIOS_OWNED BIT(16)
#define XHCI_HC_OS_OWNED BIT(24)
/* USB Legacy Support Control and Status Register  - section 7.1.2 */
/* Add this offset, plus the value of xECP in HCCPARAMS to the base address */
#define XHCI_LEGACY_CONTROL_OFFSET (0x04)
/* bits 1:2, 5:12, and 17:19 need to be preserved; bits 21:28 should be zero */
#define XHCI_LEGACY_DISABLE_SMI ((0x3 << 1) + (0xff << 5) + (0x7 << 17))

/* Supported Protocol Capability - section 7.2 */
struct xhci_protocol_caps
{
	u8 major_revision;
	u8 minor_revision;
	u8 port_offset;
	u8 port_count;
	u8 protocol_speed_id_count;
	u8 protocol_slot_type;

	u8 max_hub_depth;
	BOOL usb3_lsecc;		  /* Link Soft Error Count Capability */
	BOOL usb2_integrated_hub; /* Integrated Hub Implemented */
	BOOL usb2_hs_only;		  /* High-Speed Only Capability */
	BOOL usb2_hw_lpm;		  /* Hardware LPM Capability */
	BOOL usb2_besl_lpm;		  /* BESL LPM Capability */
};

/* Offset +00h */
#define XHCI_PROTOCOL_CAP_MINOR_REV(p) (((p) >> 16) & 0xff)
#define XHCI_PROTOCOL_CAP_MAJOR_REV(p) ((u8)(((p) >> 24) & 0xff))

/* Offset +08h */
#define XHCI_PROTOCOL_CAP_PORT_OFFSET(p) (((p) >> 0) & 0xff)
#define XHCI_PROTOCOL_CAP_PORT_COUNT(p) (((p) >> 8) & 0xff)
#define XHCI_PROTOCOL_CAP_USB3_LSECC(p) (((p) >> 24) & 0x1) // Link Soft Error Count Capability
#define XHCI_PROTOCOL_CAP_USB3_MHD(p) (((p) >> 25) & 0x7)	// Maximum Hub Depth
#define XHCI_PROTOCOL_CAP_USB2_HSO(p) (((p) >> 17) & 0x1)	// High-Speed Only Capability
#define XHCI_PROTOCOL_CAP_USB2_IHI(p) (((p) >> 18) & 0x1)	// Integrated Hub Implemented
#define XHCI_PROTOCOL_CAP_USB2_HLC(p) (((p) >> 19) & 0x1)	// Hardware LPM Capability
#define XHCI_PROTOCOL_CAP_USB2_BLC(p) (((p) >> 20) & 0x1)	// BESL LPM Capability
#define XHCI_PROTOCOL_CAP_USB2_MHD(p) (((p) >> 25) & 0x7)	// Maximum Hub Depth
#define XHCI_PROTOCOL_CAP_SPEED_ID_COUNT(p) ((u8)(((p) >> 28) & 0xf))

/* Offset +0Ch */
#define XHCI_PROTOCOL_CAP_SLOT_TYPE(p) ((u8)(((p) >> 0) & 0xf))

/* Capability Register */
/* bits 7:0 - how long is the Capabilities register */
#define XHCI_HC_LENGTH(p) (((p) >> 00) & 0x00ff)

/* USB 2.0 xHCI 0.96 L1C capability - section 7.2.2.1.3.2 */
#define XHCI_L1C BIT(16)

/* USB 2.0 xHCI 1.0 hardware LMP capability - section 7.2.2.1.3.2 */
#define XHCI_HLC BIT(19)

/* End of extended capability definitions */

/* command register values to disable interrupts and halt the HC */
/* start/stop HC execution - do not write unless HC is halted*/
#define XHCI_CMD_RUN BIT(0)
/* Event Interrupt Enable - get irq when EINT bit is set in USBSTS register */
#define XHCI_CMD_EIE BIT(2)
/* Host System Error Interrupt Enable - get irq when HSEIE bit set in USBSTS */
#define XHCI_CMD_HSEIE BIT(3)
/* Enable Wrap Event - '1' means xHC generates an event when MFINDEX wraps. */
#define XHCI_CMD_EWE BIT(10)

#define XHCI_IRQS (XHCI_CMD_EIE | XHCI_CMD_HSEIE | XHCI_CMD_EWE)

/* true: Controller Not Ready to accept doorbell or op reg writes after reset */
#define XHCI_STS_CNR BIT(11)

/*************************************************************
	TYPED REGISTER ACCESSORS
*************************************************************/

/* Strip RW1C/RW1S bits from a PORTSC value so a read-modify-write neither acks
 * pending change bits nor triggers a reset; keeps RO status and RWS controls. */
static inline u32 xhci_port_state_to_neutral(u32 state)
{
	return (state & XHCI_PORT_RO) | (state & XHCI_PORT_RWS);
}

/* PORTSC of a 1-based root-hub port */
static inline u32 xhci_port_read(struct xhci_hcor *hcor, u32 port)
{
	return mmio_read32(&hcor->portregs[port - 1].or_portsc);
}

/* Write PORTSC of a 1-based port: neutralized current value OR set_bits. */
static inline void xhci_port_write_neutral(struct xhci_hcor *hcor, u32 port, u32 set_bits)
{
	volatile u32 *portsc = &hcor->portregs[port - 1].or_portsc;
	mmio_write32(xhci_port_state_to_neutral(mmio_read32(portsc)) | set_bits, portsc);
}

/* Direct a 1-based port's link to an XDEV_* state (sets the write strobe). */
static inline void xhci_port_set_link_state(struct xhci_hcor *hcor, u32 port, u32 xdev_state)
{
	volatile u32 *portsc = &hcor->portregs[port - 1].or_portsc;
	u32 reg = xhci_port_state_to_neutral(mmio_read32(portsc)) & ~PORT_PLS_MASK;
	mmio_write32(reg | xdev_state | PORT_LINK_STROBE, portsc);
}

/* Ring a doorbell: DB_VALUE(ep_index, stream) for a device slot,
 * DB_VALUE_HOST on slot 0 for the command ring. */
static inline void xhci_db_ring(struct xhci_doorbell_array *dba, u32 slot, u32 value)
{
	mmio_write32(value, &dba->doorbell[slot]);
}

/* Acknowledge handled events: advance the interrupter's ERST dequeue pointer
 * and clear the Event Handler Busy flag. */
static inline void xhci_intr_ack_events(struct xhci_intr_reg *ir_set, u64 dequeue)
{
	xhci_writeq(&ir_set->erst_dequeue, dequeue | ERST_EHB);
}

#endif /* XHCI_REGS_H_ */
