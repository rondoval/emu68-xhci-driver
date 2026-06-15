/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __XHCI_ENDPOINT_H__
#define __XHCI_ENDPOINT_H__

#include <devices/hcd_api.h>

enum ep_state
{
    USB_DEV_EP_STATE_IDLE = 0,
    USB_DEV_EP_STATE_RECEIVING,
    USB_DEV_EP_STATE_ABORTING,
    USB_DEV_EP_STATE_RESETTING,
    USB_DEV_EP_STATE_FAILED,
    USB_DEV_EP_STATE_SUSPENDED, /* rings stopped for port U3; TDs stay queued for resume */
    USB_DEV_EP_STATE_RT_ISO_STOPPED,
    USB_DEV_EP_STATE_RT_ISO_RUNNING,
    USB_DEV_EP_STATE_RT_ISO_STOPPING
};

struct usb_device;
struct ep_context;
struct xhci_dma_span;
struct xhci_td_completion;

BOOL xhci_ep_create_context(struct usb_device *udev, u8 ep_index, u32 max_packet_size, u8 max_burst);
void xhci_ep_set_rt_interval(struct ep_context *ep_ctx, u8 interval);
void xhci_ep_destroy_contexts(struct usb_device *udev, s8 reply_code);
struct ep_context *xhci_ep_get_context_for_index(struct usb_device *udev, u8 ep_index);

void xhci_ep_set_max_packet_size(struct ep_context *ep_ctx, u32 max_packet_size);
u32 xhci_ep_get_max_packet_size(struct ep_context *ep_ctx);
u8 xhci_ep_get_max_burst(struct ep_context *ep_ctx);

void xhci_ep_set_failed(struct ep_context *ep_ctx);
void xhci_ep_set_idle(struct ep_context *ep_ctx);
void xhci_ep_set_receiving(struct ep_context *ep_ctx, struct USBIORequest *req, dma_addr_t *trb_addrs, u32 timeout_ms, u32 trb_count);
BOOL xhci_ep_set_receiving_rt(struct ep_context *ep_ctx, const struct xhci_dma_span *span,
                              u16 frame, u16 dir, BOOL staging, dma_addr_t *trb_addrs, u32 trb_count);
void xhci_ep_set_resetting(struct ep_context *ep_ctx);
void xhci_ep_set_aborting(struct ep_context *ep_ctx);

void xhci_ep_request_abort(struct ep_context *ep_ctx, struct USBIORequest *abort_req);
void xhci_ep_request_timeout_recovery(struct ep_context *ep_ctx);
void xhci_ep_request_stop(struct ep_context *ep_ctx);
void xhci_ep_process_stop(struct ep_context *ep_ctx, dma_addr_t *deq_ptr);
BOOL xhci_ep_request_suspend(struct ep_context *ep_ctx);
void xhci_ep_resume(struct ep_context *ep_ctx);

/* Stack/driver clear-halt deduplication (see xhci_udev_send_ctrl_first) */
void xhci_ep_mark_halt_synced(struct ep_context *ep_ctx);
BOOL xhci_ep_consume_halt_synced(struct ep_context *ep_ctx);

BOOL xhci_ep_is_expired(struct ep_context *ep_ctx);
enum ep_state xhci_ep_get_state(struct ep_context *ep_ctx);
u8 xhci_ep_get_ep_index(struct ep_context *ep_ctx);
u32 xhci_ep_get_active_trb_count(struct ep_context *ep_ctx);
struct xhci_ring *xhci_ep_get_ring(struct ep_context *ep_ctx);

BOOL xhci_ep_complete_by_trb(struct ep_context *ep_ctx, dma_addr_t trb_addr,
                             u32 residue, BOOL short_packet,
                             struct xhci_td_completion *out, BOOL *deferred);
BOOL xhci_ep_has_request(struct ep_context *ep_ctx, struct USBIORequest *io);
void xhci_ep_enqueue(struct ep_context *ep_ctx, struct USBIORequest *io);
void xhci_ep_flush(struct ep_context *ep_ctx, s8 reply_code);

/* RT ISO functions */
s8 xhci_ep_rt_iso_add_handler(struct ep_context *ep_ctx, struct USBIORequest *req);
s8 xhci_ep_rt_iso_rem_handler(struct ep_context *ep_ctx, struct USBIORequest *req);

s8 xhci_ep_rt_iso_start(struct ep_context *ep_ctx);
s8 xhci_ep_rt_iso_stop(struct ep_context *ep_ctx, struct USBIORequest *req);

void xhci_ep_rt_iso_in(struct ep_context *ep_ctx, APTR buffer, u32 length, u32 act_len, u16 rt_frame);
void xhci_ep_rt_iso_out(struct ep_context *ep_ctx, APTR buffer, u32 length, u32 act_len, u16 rt_frame);

void xhci_ep_schedule_rt_iso(struct ep_context *ep_ctx);

/* Free a staging IN buffer back to the endpoint's per-endpoint slab. */
void xhci_ep_free_rt_iso_buffer(struct ep_context *ep_ctx, APTR data_buffer);

#endif /* __XHCI_ENDPOINT_H__ */