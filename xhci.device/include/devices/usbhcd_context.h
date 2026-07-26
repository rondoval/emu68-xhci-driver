#ifndef DEVICES_USBHCD_CONTEXT_H
#define DEVICES_USBHCD_CONTEXT_H
/*
**  usbhcd_context.h — the context HCD ABI (lifecycle ops + transfers)
**
**  The xHCI-native half of the two-ABI split: explicit device/endpoint
**  lifecycle operations and handle-keyed transfers for hardware-managed
**  controllers. This ABI has its own commands and its own request structs;
**  the only surface it shares with the legacy per-transfer wire format
**  (usbhardware.h V1+V2, which stays frozen for classic HCDs) — the
**  UHIOERR_ error value pool, the UHA_Capabilities capability tag and bits,
**  and the iso buffer-block flag pool — lives in the shared header
**  usbhcd_common.h, included below. A driver opts in with the UHCF_CONTEXT
**  capability bit and advertises the individual commands it implements
**  through NSCMD_DEVICEQUERY (NewStyle Device).
**
**  Request framing:
**   - LIFECYCLE OPS are synchronous commands on a plain struct IOStdReq
**     (exactly like NSCMD_DEVICEQUERY): io_Command = the NSCMD below,
**     io_Data -> the Uhcd* op struct, io_Length = its size, io_Error
**     returns a UHIOERR_ code. OUT fields are filled by the HCD.
**     Ops are serialized by the HCD's unit task; the create-device op
**     performs the whole Enable Slot + Address Device sequence internally,
**     so no software-visible default-address phase exists on this ABI
**     (the stack-wide address-0 serialization applies to legacy HCDs only).
**   - TRANSFERS are direct calls into the HCD ("The transfer path" below):
**     NSCMD_USB_ATTACH exchanges the stack's completion hook for the HCD's
**     submit/abort entries, and the lifecycle ops return opaque per-endpoint
**     tokens that key every submit. No IORequest travels for a transfer;
**     completion arrives through the hook. Everything durable (topology,
**     speed, max packet, intervals, SS companion facts) lives in the
**     device/endpoint contexts established by the lifecycle ops.
*/

#ifndef EXEC_IO_H
#include <exec/io.h>
#endif
#ifndef UTILITY_HOOKS_H
#include <utility/hooks.h>
#endif
#ifndef DEVICES_USBHCD_COMMON_H
#include <devices/usbhcd_common.h>   /* shared error/capability/iso-flag pool */
#endif

#if defined(__GNUC__)
# pragma pack(2)
#endif

/* ------------------------------------------------------------------------ */
/* Commands — a block in the third-party command area. The NSD standard
 * keeps 0x4000-0x7FFF and 0xC000-0xFFFF for the OS; third parties get
 * 0x0000-0x3FFF and 0x8000-0xBFFF. Fleet allocations: nvme passthrough
 * 0x8020..0x8024, this block here, netdev 0x8900..0x891f.
 * NSCMD_DEVICEQUERY enumerates exactly the commands a driver implements; an
 * unimplemented one is rejected with IOERR_NOCMD.
 *
 * Each op below is marked [M] mandatory or [O] optional:
 *   [M] a UHCF_CONTEXT driver MUST implement it — the stack rejects the
 *       context backend and falls back to the legacy ABI otherwise (the gate
 *       is UHCD_MANDATORY_CMD_MASK below).
 *   [O] optional: the stack probes for it via NSD and degrades gracefully when
 *       absent (e.g. no DECONFIGURE -> replace-config only; no ALLOC_STREAMS ->
 *       single-ring UAS; no SET_LINK_POWER -> no LPM). Iso submits are gated
 *       by the UHCF_ISO capability. */
#define NSCMD_USBHCD_BASE               0x8800

/* lifecycle ops (struct IOStdReq framing) */
#define NSCMD_USB_CREATE_DEVICE         (NSCMD_USBHCD_BASE + 0x00)  /* [M] */
#define NSCMD_USB_DESTROY_DEVICE        (NSCMD_USBHCD_BASE + 0x01)  /* [M] */
#define NSCMD_USB_UPDATE_EP0            (NSCMD_USBHCD_BASE + 0x02)  /* [M] */
#define NSCMD_USB_CONFIGURE_ENDPOINTS   (NSCMD_USBHCD_BASE + 0x03)  /* [M] */
#define NSCMD_USB_DECONFIGURE           (NSCMD_USBHCD_BASE + 0x04)  /* [O] */
#define NSCMD_USB_UPDATE_HUB            (NSCMD_USBHCD_BASE + 0x05)  /* [M] */
#define NSCMD_USB_RESET_DEVICE          (NSCMD_USBHCD_BASE + 0x06)  /* [O] */
#define NSCMD_USB_SET_SUSPEND           (NSCMD_USBHCD_BASE + 0x07)  /* [O] */
#define NSCMD_USB_SET_LINK_POWER        (NSCMD_USBHCD_BASE + 0x08)  /* [O] */
#define NSCMD_USB_ALLOC_STREAMS         (NSCMD_USBHCD_BASE + 0x09)  /* [O] */
#define NSCMD_USB_FREE_STREAMS          (NSCMD_USBHCD_BASE + 0x0a)  /* [O] */
/* the transfer-path attach handshake ("The transfer path" below) and the
 * clock-driven iso hooks (struct UhcdIsoHooks / USBIsoHooks below) */
#define NSCMD_USB_ATTACH                (NSCMD_USBHCD_BASE + 0x0b)  /* [M] */
#define NSCMD_USB_REGISTER_HOOKS        (NSCMD_USBHCD_BASE + 0x0c)  /* [O] */
#define NSCMD_USB_UNREGISTER_HOOKS      (NSCMD_USBHCD_BASE + 0x0d)  /* [O] */
#define NSCMD_USB_START_STREAM          (NSCMD_USBHCD_BASE + 0x0e)  /* [O] */
#define NSCMD_USB_STOP_STREAM           (NSCMD_USBHCD_BASE + 0x0f)  /* [O] */

/* the whole context command block occupies NSCMD_USBHCD_BASE + 0x00..0x1f */
#define UHCD_IS_CTXCMD(cmd)     ((((UWORD)(cmd)) & 0xFFE0) == NSCMD_USBHCD_BASE)

/* UHCD_CTXCMD_BIT(cmd): the command's bit in a context-command mask (one bit
 * per NSCMD_USB_* op).
 * UHCD_MANDATORY_CMD_MASK is the set every UHCF_CONTEXT driver MUST advertise
 * in its NSD list; the stack keeps such a driver on the legacy backend unless
 * (nsd_mask & UHCD_MANDATORY_CMD_MASK) == UHCD_MANDATORY_CMD_MASK. */
#define UHCD_CTXCMD_BIT(cmd)    (1UL << (((UWORD)(cmd)) - NSCMD_USBHCD_BASE))
#define UHCD_MANDATORY_CMD_MASK ( \
    UHCD_CTXCMD_BIT(NSCMD_USB_CREATE_DEVICE)       | \
    UHCD_CTXCMD_BIT(NSCMD_USB_DESTROY_DEVICE)      | \
    UHCD_CTXCMD_BIT(NSCMD_USB_UPDATE_EP0)          | \
    UHCD_CTXCMD_BIT(NSCMD_USB_CONFIGURE_ENDPOINTS) | \
    UHCD_CTXCMD_BIT(NSCMD_USB_UPDATE_HUB)          | \
    UHCD_CTXCMD_BIT(NSCMD_USB_ATTACH) )

/* ------------------------------------------------------------------------ */
/* Device handles. The HCD allocates handles at NSCMD_USB_CREATE_DEVICE and
 * the stack treats them as opaque. Handle values >= UHCD_HANDLE_RESERVED are
 * reserved for emulated devices; the HCD never allocates them for real slots
 * and ignores them where a real device is required (e.g. cdo_TTHubHandle —
 * root ports have no external TT, the xHC translates itself).
 *
 * The root hub(s) are emulated by the HCD — they have no hardware slot — so
 * their create performs no hardware work, and EVERY implemented lifecycle op
 * on a root-hub handle (destroy, update-EP0, configure/deconfigure,
 * update-hub) is a successful no-op: the stack drives the root devices
 * through the same code path as any other device — including the endpoint
 * tokens their create/configure ops return, whose submits the HCD routes to
 * the root-hub emulation.
 *
 * A controller with both USB2-protocol and USB3-protocol root ports exposes
 * TWO protocol-pure root hubs (report the count via UHA_NumRootHubs in the
 * device query). CREATE_DEVICE with
 * cdo_ParentHandle == 0 selects by cdo_Speed:
 *   - cdo_Speed >= UHCD_SPEED_SUPER and USB3 ports exist
 *       -> UHCD_HANDLE_ROOTHUB       (the SuperSpeed root hub)
 *   - otherwise, if USB2 ports exist
 *       -> UHCD_HANDLE_ROOTHUB_USB2  (the USB 2.0 root hub)
 *   - USB3-only controller -> UHCD_HANDLE_ROOTHUB
 * Devices on root ports are created with the corresponding root-hub handle
 * as cdo_ParentHandle; cdo_HubPort is the port number ON THAT root hub
 * (1..bNbrPorts of its hub descriptor), not a controller-global index.
 *
 * 0 is never a valid device handle (it means "root hub" in cdo_ParentHandle
 * only). */
#define UHCD_HANDLE_RESERVED    0xFFFFFFF0UL
#define UHCD_HANDLE_ROOTHUB_USB2 0xFFFFFFFDUL
#define UHCD_HANDLE_ROOTHUB     0xFFFFFFFEUL

/* ------------------------------------------------------------------------ */
/* The wire setup packet of a control transfer (16-bit fields LE), passed to
 * the control submit entry ("The transfer path" below). */

struct UhcdSetupData
{
    UBYTE   usd_RequestType;
    UBYTE   usd_Request;
    UWORD   usd_Value;
    UWORD   usd_Index;
    UWORD   usd_Length;
};

/* transfer flags (the submit entries' flags argument). The bit positions
 * equal the matching UHFB_ bits of the legacy iouh_Flags word (usbhardware.h),
 * so a stack translates with a single AND mask. */
#define UHCD_XFB_NOSHORTPKT     2           /* suppress the terminating short packet on OUT */
#define UHCD_XFB_ALLOWRUNT      4           /* short read is success, not UHIOERR_RUNTPACKET */
#define UHCD_XFF_NOSHORTPKT     (1 << UHCD_XFB_NOSHORTPKT)
#define UHCD_XFF_ALLOWRUNT      (1 << UHCD_XFB_ALLOWRUNT)

/* layout freeze helper (pack(2)) */
#define UHCD_ABI_ASSERT2(cond, line) typedef char uhcd_abi_assert_##line[(cond) ? 1 : -1]
#define UHCD_ABI_ASSERT1(cond, line) UHCD_ABI_ASSERT2(cond, line)
#define UHCD_ABI_ASSERT(cond)        UHCD_ABI_ASSERT1(cond, __LINE__)
#ifndef UHCD_OFFSETOF
# if defined(__GNUC__)
#  define UHCD_OFFSETOF(type, member) __builtin_offsetof(type, member)
# else
#  define UHCD_OFFSETOF(type, member) ((ULONG) &((type *)0)->member)
# endif
#endif

/* ------------------------------------------------------------------------ */
/* Values for UhcdCreateDevice.cdo_Speed */
#define UHCD_SPEED_LOW          0
#define UHCD_SPEED_FULL         1
#define UHCD_SPEED_HIGH         2
#define UHCD_SPEED_SUPER        3
#define UHCD_SPEED_SUPERPLUS    4

/* Endpoint types (== USB bmAttributes transfer type) */
#define UHCD_EPTYPE_CONTROL     0
#define UHCD_EPTYPE_ISO         1
#define UHCD_EPTYPE_BULK        2
#define UHCD_EPTYPE_INTERRUPT   3

/* ------------------------------------------------------------------------ */
/* Lifecycle op parameter blocks (io_Data of a struct IOStdReq). The device
 * handle is the opaque token returned by NSCMD_USB_CREATE_DEVICE; the stack
 * never interprets it.
 * Handle 0 in cdo_ParentHandle denotes "this create is a root hub". */

struct UhcdCreateDevice         /* NSCMD_USB_CREATE_DEVICE */
{
    ULONG   cdo_ParentHandle;   /* 0 = this is a root hub (see handle rules above) */
    UWORD   cdo_HubPort;        /* 1-based port on the parent hub */
    UWORD   cdo_Speed;          /* UHCD_SPEED_* */
    ULONG   cdo_TTHubHandle;    /* LS/FS behind a HS hub: the TT hub; 0 or a reserved handle = none */
    UWORD   cdo_TTPort;         /* TT port number */
    UWORD   cdo_TTThinkTime;    /* encoded think time */
    UWORD   cdo_Ep0MaxPktHint;  /* 0 = HCD guesses from speed */
    UWORD   cdo_Pad;
    ULONG   cdo_DeviceHandle;   /* OUT: the allocated handle */
    APTR    cdo_Ep0Token;       /* OUT: the device's EP0 submit token
                                   ("The transfer path" below) */
};

struct UhcdDestroyDevice        /* NSCMD_USB_DESTROY_DEVICE */
{
    ULONG   ddo_DeviceHandle;
};

struct UhcdUpdateEp0            /* NSCMD_USB_UPDATE_EP0 */
{
    ULONG   ueo_DeviceHandle;
    UWORD   ueo_Ep0MaxPkt;      /* validated bMaxPacketSize0 in BYTES.  The
                                   stack validates per speed — LS: 8, FS:
                                   8/16/32/64, HS: 64, SS+: always 512 (the SS
                                   descriptor byte is the exponent; never pass
                                   it through raw).  The HCD rejects anything
                                   else with UHIOERR_BADPARAMS rather than
                                   shrink EP0 into a babble trap. */
    UWORD   ueo_Pad;
};

struct UhcdEndpointDesc         /* one endpoint of a configure op */
{
    UBYTE   ed_Address;         /* bEndpointAddress: number | direction */
    UBYTE   ed_Type;            /* UHCD_EPTYPE_* */
    UWORD   ed_MaxPacket;       /* RAW wMaxPacketSize value: bits 10:0 = size,
                                   bits 12:11 = HS periodic extra transactions */
    UBYTE   ed_Interval;        /* encoded (as in the endpoint descriptor) */
    UBYTE   ed_MaxBurst;        /* SS companion; 0 otherwise */
    UBYTE   ed_Mult;            /* SS isoch */
    UBYTE   ed_IfClass;         /* bInterfaceClass of the owning interface — REQUIRED
                                   when known: controller quirks key on it (e.g. the
                                   VL805 SS-bulk-OUT mass-storage burst quirk is NOT
                                   applied when this is 0/unknown) */
    UWORD   ed_BytesPerInterval;/* SS companion */
    UWORD   ed_MaxStreams;      /* SS bulk; 0 = no streams */
    APTR    ed_Token;           /* OUT: the endpoint's submit token, written
                                   when the configure op succeeds */
};

/* How it works: in one xHCI Configure Endpoint the HCD builds endpoint
 * contexts and transfer rings for the ceo_Add[] set and drops the endpoints
 * named in ceo_DropAddresses[]. The stack fills ceo_Add straight from its
 * parsed config/interface/endpoint tree; on success the HCD writes each added
 * endpoint's submit token back into its ceo_Add[] entry (ed_Token — the block
 * is referenced, not copied, and must stay valid for the whole op). Dropping
 * an endpoint invalidates its token and retires its in-flight transfers.
 * A plain SET_CONFIGURATION populates only ceo_Add; a SET_INTERFACE populates
 * both add and drop. AFTER this op succeeds the stack issues the wire
 * SET_CONFIGURATION/SET_INTERFACE as a normal EP0 transfer, so the device
 * enters the new state. */
struct UhcdConfigureEndpoints   /* NSCMD_USB_CONFIGURE_ENDPOINTS */
{
    ULONG   ceo_DeviceHandle;
    UWORD   ceo_ConfigValue;    /* the configuration being selected */
    UWORD   ceo_NumAdd;
    struct UhcdEndpointDesc *ceo_Add;   /* endpoints to add */
    UWORD   ceo_NumDrop;
    UWORD   ceo_Pad;
    UBYTE  *ceo_DropAddresses;  /* endpoint addresses to drop (SET_INTERFACE) */
};

struct UhcdDeconfigure          /* NSCMD_USB_DECONFIGURE (SET_CONFIGURATION 0) */
{
    ULONG   dco_DeviceHandle;
};

struct UhcdUpdateHub            /* NSCMD_USB_UPDATE_HUB */
{
    ULONG   uho_DeviceHandle;
    UWORD   uho_NumPorts;
    UWORD   uho_TTThinkTime;
    UBYTE   uho_MultiTT;
    UBYTE   uho_HdrDecLat;      /* SS hubs: bHubHdrDecLat (0.1 µs units, as in
                                   the SS hub descriptor); 0 otherwise.  Feeds
                                   the HCD's U1/U2 exit-latency computation for
                                   devices downstream of this hub. */
    UWORD   uho_HubDelay;       /* SS hubs: wHubDelay (ns); 0 otherwise */
};

struct UhcdResetDevice          /* NSCMD_USB_RESET_DEVICE */
{
    ULONG   rdo_DeviceHandle;
};

/* Endpoint-ring quiesce around a port suspend (xHCI 4.15.1: all endpoints
 * stopped before the port is directed to U3).  The port transition itself is
 * NOT part of the op — the stack's hub class drives the link (external hub
 * port or root-hub view alike): suspend = SET_SUSPEND(1), then the port to
 * U3; resume = port to U0, then SET_SUSPEND(0).  Idempotent both ways; a
 * root-hub handle is a successful no-op. */
struct UhcdSetSuspend           /* NSCMD_USB_SET_SUSPEND */
{
    ULONG   sso_DeviceHandle;
    UWORD   sso_Suspend;        /* 1 = quiesce endpoint rings, 0 = restart them */
    UWORD   sso_Pad;
};

/* Link-power policy plus the pre-parsed BOS facts the HCD needs to program
 * it.  The stack owns the BOS read/parse and the go/no-go policy; the HCD owns
 * the controller-side computation (SEL/PEL/MEL, timeouts, HIRD/BESL) and the
 * controller-side mechanism (Evaluate Context for MEL, USB2 hardware-LPM
 * registers).  The HCD issues NO device/hub control transfers: it writes the
 * computed wire parameters into the slo_Out* fields and the stack issues
 * SET_SEL, SET_FEATURE(U1/U2/LTM_ENABLE) to the device and
 * SetPortFeature(U1/U2_TIMEOUT) to the parent hub itself.  The slo_U1/U2Timeout
 * and slo_MaxExitLatency IN fields are OVERRIDES: 0 = the HCD computes them.
 * Devices reject U1/U2_ENABLE until configured (USB 3.2 §9.4.9), so issue this
 * op only after the wire SET_CONFIGURATION completed.  The op replies only once
 * the MEL Evaluate Context has been latched: io_Error == 0 means the controller
 * state is in place and the slo_Out* fields are valid, so the stack should issue
 * the wire transfers UHCD_LPO_* asks for; io_Error != 0 means arm nothing. */
struct UhcdSetLinkPower         /* NSCMD_USB_SET_LINK_POWER */
{
    ULONG   slo_DeviceHandle;
    UWORD   slo_Flags;          /* UHCD_LPF_* capability facts from the BOS */
    UWORD   slo_U1Enable;       /* policy: allow U1 on the upstream link */
    UWORD   slo_U2Enable;       /* policy: allow U2 */
    UWORD   slo_U1DevExitLat;   /* BOS SS cap bU1DevExitLat (µs); 0 = U1 incapable */
    UWORD   slo_U2DevExitLat;   /* BOS SS cap wU2DevExitLat (µs); 0 = U2 incapable */
    UBYTE   slo_BeslBaseline;   /* BOS USB2-ext; valid if UHCD_LPF_BESL_BASELINE */
    UBYTE   slo_BeslDeep;       /* BOS USB2-ext; valid if UHCD_LPF_BESL_DEEP */
    UWORD   slo_U1Timeout;      /* IN override; 0 = HCD computes */
    UWORD   slo_U2Timeout;      /* IN override; 0 = HCD computes */
    ULONG   slo_MaxExitLatency; /* IN override MEL (µs); 0 = HCD computes */
    /* --- OUT: HCD-computed wire parameters, read back by the stack --- */
    UWORD   slo_OutU1Timeout;   /* hub-encoded U1 inactivity timeout; 0 = state off */
    UWORD   slo_OutU2Timeout;   /* hub-encoded U2 inactivity timeout; 0 = state off */
    UWORD   slo_OutU1Sel;       /* U1 System Exit Latency (µs) for the SET_SEL payload */
    UWORD   slo_OutU1Pel;       /* U1 Path Exit Latency (µs) */
    UWORD   slo_OutU2Sel;       /* U2 System Exit Latency (µs) */
    UWORD   slo_OutU2Pel;       /* U2 Path Exit Latency (µs) */
    UWORD   slo_OutFlags;       /* UHCD_LPO_* — which wire transfers the stack should issue */
};

/* slo_Flags — capability facts from the device's BOS descriptor */
#define UHCD_LPF_USB2_LPM       (1 << 0)    /* USB2-ext: LPM (L1) capable */
#define UHCD_LPF_BESL           (1 << 1)    /* USB2-ext: BESL/alt-HIRD supported */
#define UHCD_LPF_BESL_BASELINE  (1 << 2)    /* slo_BeslBaseline is valid */
#define UHCD_LPF_BESL_DEEP      (1 << 3)    /* slo_BeslDeep is valid */
#define UHCD_LPF_LTM            (1 << 4)    /* SS cap: LTM capable */

/* slo_OutFlags — the device/hub control transfers the stack issues after a
 * successful op (io_Error == 0).  Port timeouts are conveyed by the nonzero
 * slo_OutU1/U2Timeout fields, not a flag. */
#define UHCD_LPO_SET_SEL        (1 << 0)    /* send SET_SEL (SEL/PEL valid, ≥1 state enabled) */
#define UHCD_LPO_U1_INIT        (1 << 1)    /* send SET_FEATURE(U1_ENABLE) (device may initiate) */
#define UHCD_LPO_U2_INIT        (1 << 2)    /* send SET_FEATURE(U2_ENABLE) */
#define UHCD_LPO_LTM            (1 << 3)    /* send SET_FEATURE(LTM_ENABLE) */

UHCD_ABI_ASSERT(sizeof(struct UhcdSetLinkPower) == 38);

/* SS bulk streams (UAS).  ALLOC_STREAMS gives the endpoint one transfer ring
 * per stream id 1..sto_NumStreams (sto_NumStreams = the HIGHEST stream id the
 * stack will use; it must not exceed the ed_MaxStreams the endpoint was
 * configured with).  The endpoint must be configured, bulk, and idle (no
 * transfers in flight).  After success, every bulk submit on the endpoint
 * selects its ring by the stream_id argument (1..N; 0 is then invalid).
 * FREE_STREAMS returns the endpoint to its default single ring; it requires
 * an idle endpoint and is idempotent.  Without a successful alloc the
 * endpoint stays single-ring and the submit's stream_id rides along ignored —
 * a stack over a driver without these ops keeps the pre-streams behavior.
 * A driver only lists the ops in its NSD response when the controller
 * actually supports streams (xHCI: HCCPARAMS1.MaxPSASize > 0).  The emulated
 * root hubs have no bulk endpoints, so like the RT-ISO ops these REJECT
 * reserved handles with UHIOERR_BADPARAMS. */
struct UhcdStreams              /* NSCMD_USB_ALLOC_STREAMS / FREE_STREAMS */
{
    ULONG   sto_DeviceHandle;
    UBYTE   sto_EpAddress;      /* bEndpointAddress of the SS bulk endpoint */
    UBYTE   sto_Pad;
    UWORD   sto_NumStreams;     /* alloc: highest stream id; ignored on free */
};

/* ------------------------------------------------------------------------ */
/* The transfer path — every control/bulk/interrupt/iso transfer is a direct
 * call into the HCD; no IORequest travels.
 *
 * NSCMD_USB_ATTACH (IOStdReq framing, io_Data -> UhcdAttach) is issued once
 * per open, right after the NSD scan: the stack passes its completion hook
 * and the HCD returns its three transfer entries plus an opaque controller
 * context, passed back as the first argument of every entry (HCDs are
 * ROM-able and carry no writable data — the context is their only anchor).
 * Re-attach replaces the hook.  The entries use the plain C (stack-argument)
 * calling convention; they are callable from any task, never from
 * interrupts.
 *
 * Submits are keyed by the opaque endpoint tokens the lifecycle ops return:
 * CREATE_DEVICE yields the device's EP0 token (cdo_Ep0Token, root hubs
 * included), CONFIGURE_ENDPOINTS yields one token per added endpoint
 * (ed_Token).  A token is valid from delivery until its endpoint is dropped
 * or its device destroyed; a stale token is safe — the entries fail it with
 * UHIOERR_TIMEOUT (device-gone semantics).
 *
 *   LONG err = submit(hcd, ep_token, data, length, naktimeout_ms,
 *                     stream_id, flags, cookie);         bulk/interrupt/iso
 *   LONG err = ctrl_submit(hcd, ep0_token, setup, data, length,
 *                          naktimeout_ms, cookie);       control
 *
 * The endpoint's transfer type is known HCD-side from the token.  Control
 * direction comes from setup->usd_RequestType bit 7; *setup is copied before
 * ctrl_submit returns.  Iso submits are gated by the UHCF_ISO capability.
 * cookie is the caller's demux handle; naktimeout_ms 0 = none; stream_id
 * selects an allocated stream ring (0 = default ring, see UhcdStreams);
 * flags = UHCD_XFF_*.  A non-zero return is the synchronous UHIOERR_
 * failure and no completion follows; on 0 the transfer is in flight and
 * completion arrives EXACTLY ONCE via the attach hook:
 *
 *   CallHookPkt(ato_DoneHook, ato_UserData, &UhcdXferDone)
 *
 * from the HCD's completion context (its unit task).  The hook must be
 * non-blocking; it may re-enter submit().
 * abort(hcd, ep_token, cookie) requests an abort of a submitted transfer (a
 * wish, like AbortIO: the completion still arrives, possibly successful). */
struct UhcdAttach               /* NSCMD_USB_ATTACH */
{
    struct Hook *ato_DoneHook;  /* IN: transfer-completion hook */
    APTR    ato_UserData;       /* IN: hook object (a2) for done calls */
    APTR    ato_HcdContext;     /* OUT: first argument of every entry */
    APTR    ato_Submit;         /* OUT: UhcdSubmitFunc */
    APTR    ato_CtrlSubmit;     /* OUT: UhcdCtrlSubmitFunc */
    APTR    ato_Abort;          /* OUT: UhcdAbortFunc */
};

struct UhcdXferDone             /* the done-hook message (a1) */
{
    APTR    uxd_Cookie;         /* the submit's cookie */
    ULONG   uxd_Actual;         /* bytes transferred */
    UWORD   uxd_ExtError;       /* extended error code (0 for now) */
    UBYTE   uxd_Error;          /* UHIOERR_ result */
    UBYTE   uxd_Pad;
};

typedef LONG (*UhcdSubmitFunc)(APTR hcd, APTR ep_token, APTR data,
                               ULONG length, ULONG naktimeout_ms,
                               UWORD stream_id, UWORD flags, APTR cookie);
typedef LONG (*UhcdCtrlSubmitFunc)(APTR hcd, APTR ep0_token,
                                   const struct UhcdSetupData *setup,
                                   APTR data, ULONG length,
                                   ULONG naktimeout_ms, APTR cookie);
typedef LONG (*UhcdAbortFunc)(APTR hcd, APTR ep_token, APTR cookie);

/* ------------------------------------------------------------------------ */
/* Clock-driven iso hooks — isochronous endpoints only.
 *
 * NSCMD_USB_REGISTER_HOOKS installs a struct USBIsoHooks on an iso endpoint,
 * NSCMD_USB_UNREGISTER_HOOKS removes it (same block passed back),
 * NSCMD_USB_START/STOP_STREAM arm and disarm the continuous engine; STOP
 * replies only once the engine has drained (all in-flight iso TDs retired).
 * The block must stay valid from REGISTER to UNREGISTER.  While the stream
 * runs the HCD calls the hooks at frame cadence from its completion context:
 *
 *   CallHookPkt(hook, uih_Object, &buffer_request)
 *
 * where buffer_request is the classic 12-byte iso buffer block (Poseidon's
 * struct IOUsbHWBufferReq == the driver's struct USBBufferRequest — this
 * header deliberately names neither: {u8 *data; u32 length; u16 frame;
 * u16 flags}).  uih_Object is caller-chosen (Poseidon passes the classic
 * IOUsbHWRTIso block so existing class hooks run unchanged).  The hooks
 * must be non-blocking.  uih_ReleaseHook (may be NULL) fires once, with a
 * NULL message, when the stream dies without a client STOP — endpoint
 * failure or device teardown.  On the done direction the buffer block's
 * flags carry UHCD_UBF_XFER_ERROR when the interval's transfer failed on
 * the wire.  The emulated root hubs have no iso endpoints, so these ops
 * REJECT reserved handles with UHIOERR_BADPARAMS. */
struct USBIsoHooks
{
    struct Hook *uih_OutRequestHook;    /* OUT: fill the next span to send */
    struct Hook *uih_OutDoneHook;       /* OUT: span transmitted (recycle) */
    struct Hook *uih_InRequestHook;     /* IN:  provide a receive span */
    struct Hook *uih_InDoneHook;        /* IN:  span filled (consume) */
    struct Hook *uih_ReleaseHook;       /* stream died without STOP; may be NULL */
    ULONG        uih_MaxPrefetch;       /* OUT: max bytes pulled ahead (0 = HCD default) */
    UWORD        uih_Flags;             /* none defined yet */
    UWORD        uih_Pad;
    APTR         uih_Object;            /* hook object (a2) for every call */
};

/* The buffer-block flag UHCD_UBF_XFER_ERROR (set by the HCD on *_done calls
 * when the interval failed on the wire) lives in usbhcd_common.h. */

struct UhcdIsoHooks             /* NSCMD_USB_REGISTER/UNREGISTER_HOOKS, START/STOP_STREAM */
{
    ULONG   uio_DeviceHandle;
    UBYTE   uio_EpAddress;      /* bEndpointAddress (num | 0x80 = IN) */
    UBYTE   uio_Pad;
    UWORD   uio_Pad2;
    struct USBIsoHooks *uio_Hooks;
};

#if defined(__GNUC__)
# pragma pack()
#endif

#endif /* DEVICES_USBHCD_CONTEXT_H */
