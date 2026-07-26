#ifndef DEVICES_USBHCD_COMMON_H
#define DEVICES_USBHCD_COMMON_H
/*
**  usbhcd_common.h — values shared by the two USB HCD ABIs
**
**  The legacy per-transfer ABI (usbhardware.h) and the context lifecycle ABI
**  (usbhcd_context.h) share a narrow value surface: the io_Error value pool,
**  the UHA_Capabilities capability tag and its bit namespace, the
**  device-query command + tag pool (UHCMD_QUERYDEVICE / UHA_ identity tags,
**  still issued by the context stack), and the iso buffer-block flag pool.
**  This header is the single source of truth for those values so none of
**  them is ever defined twice.
**
**  VENDORED HEADER — the source of truth is
**  poseidon-backport/include/devices/usbhcd_common.h; the copy in
**  emu68-xhci-driver/xhci.device/include/devices/ must stay identical.
*/

#ifndef UTILITY_TAGITEM_H
#include <utility/tagitem.h>
#endif

#ifndef EXEC_IO_H
#include <exec/io.h>
#endif

/* ------------------------------------------------------------------------ */
/* Error value pool (io_Error / UHIOERR_).
 * Values 0..13 are the classic usbhardware.h pool; 14 is the context ABI's
 * addition. */
#define UHIOERR_NO_ERROR        0       /* No error occurred                        */
#define UHIOERR_USBOFFLINE      1       /* USB non-operational                      */
#define UHIOERR_NAK             2       /* NAK received                             */
#define UHIOERR_HOSTERROR       3       /* Unspecific host error                    */
#define UHIOERR_STALL           4       /* Endpoint stalled                         */
#define UHIOERR_PKTTOOLARGE     5       /* Packet too large to be transferred       */
#define UHIOERR_TIMEOUT         6       /* No acknowledge on packet (device gone)   */
#define UHIOERR_OVERFLOW        7       /* More data received than expected         */
#define UHIOERR_CRCERROR        8       /* Incoming packet corrupted                */
#define UHIOERR_RUNTPACKET      9       /* Less data received than requested        */
#define UHIOERR_NAKTIMEOUT      10      /* Timeout due to NAKs                      */
#define UHIOERR_BADPARAMS       11      /* Illegal parameters in request            */
#define UHIOERR_OUTOFMEMORY     12      /* Out of auxiliary memory for the driver   */
#define UHIOERR_BABBLE          13      /* Babble condition                         */
#define UHIOERR_NO_BANDWIDTH    14      /* Configure/alloc-streams rejected for
                                           periodic bandwidth; retry lighter        */

/* ------------------------------------------------------------------------ */
/* Device-query command + tag pool (UHCMD_ / UHA_).  The legacy
 * device-management commands and the UHCMD_QUERYDEVICE tag list carry the
 * SAME wire values in both ABIs: the context stack still issues
 * UHCMD_QUERYDEVICE (to read UHA_Capabilities and the identity strings) and
 * UHCMD_USBRESET before it selects the context backend.  They live here so a
 * context HCD can answer them without pulling in the legacy usbhardware.h /
 * hcd_api.h. */
#define UHCMD_QUERYDEVICE       (CMD_NONSTD + 0)
#define UHCMD_USBRESET          (CMD_NONSTD + 1)

/* UHA_State is LEGACY-ABI ONLY — the context stack never queries operational
 * state; its value bits (UHSB_/UHSF_) live in the legacy usbhardware.h. */
#define UHA_Dummy               (TAG_USER  + 0x4711)
#define UHA_State               (UHA_Dummy + 0x01) /* legacy ABI only */
#define UHA_Manufacturer        (UHA_Dummy + 0x10)
#define UHA_ProductName         (UHA_Dummy + 0x11)
#define UHA_Version             (UHA_Dummy + 0x12)
#define UHA_Revision            (UHA_Dummy + 0x13)
#define UHA_Description         (UHA_Dummy + 0x14)
#define UHA_Copyright           (UHA_Dummy + 0x15)
#define UHA_DriverVersion       (UHA_Dummy + 0x20)
#define UHA_Capabilities        (UHA_Dummy + 0x21) /* bit namespace (UHCB_/UHCF_) below */
#define UHA_PrepareEndpoint     (UHA_Dummy + 0x22) /* reserved; never queried */
#define UHA_DestroyEndpoint     (UHA_Dummy + 0x23) /* reserved; never queried */
#define UHA_NumRootHubs         (UHA_Dummy + 0x24)
/* Recommended DMA buffer alignment in bytes (a power of two). The HCD reports
 * the granularity a data buffer must meet to be DMA'd directly; buffers that
 * miss it are bounced through an aligned copy. 0/absent = no constraint. */
#define UHA_DMAAlignment        (UHA_Dummy + 0x25)

/* ------------------------------------------------------------------------ */
/* UHA_Capabilities bit namespace (UHCB_/UHCF_).
 * The legacy and context ABIs share this tag, so its bits form one merged
 * namespace: bits 0..4 and 31 belong to the legacy ABI, bit 5 (CONTEXT) to
 * the context ABI.
 *
 * NOTE: bit 4 is the CLASSIC UHCF_USB2OTG — the two namespaces share the tag,
 * so bit 4 must never be reused. */
#define UHCB_USB20              0       /* Host controller supports USB 2.0 Highspeed          */
#define UHCB_ISO                1       /* HCD supports ISO transfers                          */
#define UHCB_RT_ISO             2       /* HCD supports real-time ISO / iso hooks              */
#define UHCB_QUICKIO            3       /* BeginIO()/AbortIO() may be called from interrupts    */
#define UHCB_USB2OTG            4       /* Host controller supports USB2OTG device mode        */
#define UHCB_CONTEXT            5       /* HCD speaks the context lifecycle ABI                */
#define UHCB_USB30              31      /* Host controller supports USB 3.x SuperSpeed/+       */

#define UHCF_USB20              (1UL << UHCB_USB20)
#define UHCF_ISO                (1UL << UHCB_ISO)
#define UHCF_RT_ISO             (1UL << UHCB_RT_ISO)
#define UHCF_QUICKIO            (1UL << UHCB_QUICKIO)
#define UHCF_USB2OTG            (1UL << UHCB_USB2OTG)
#define UHCF_CONTEXT            (1UL << UHCB_CONTEXT)
#define UHCF_USB30              (1UL << UHCB_USB30)

/* ------------------------------------------------------------------------ */
/* Iso buffer-block flag pool (ubr_Flags of the 12-byte iso buffer block —
 * Poseidon's struct IOUsbHWBufferReq == the driver's struct USBBufferRequest).
 * Bit 0 (CONTBUFFER) is the classic scatter/gather flag; bit 14 (XFER_ERROR)
 * is set by the HCD on a *_done call when the interval failed on the wire. */
#define UBFB_CONTBUFFER         0       /* more buffer to copy (scatter/gather) */
#define UBFF_CONTBUFFER         (1 << UBFB_CONTBUFFER)
#define UHCD_UBB_XFER_ERROR     14      /* interval's transfer failed on the wire */
#define UHCD_UBF_XFER_ERROR     (1 << UHCD_UBB_XFER_ERROR)

#endif /* DEVICES_USBHCD_COMMON_H */
