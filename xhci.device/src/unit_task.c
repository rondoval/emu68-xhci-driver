// SPDX-License-Identifier: GPL-2.0-only
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/timer_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#include <proto/timer.h>
#endif

#include <dos/dos.h>

#include <device.h>
#include <minlist.h>
#include <debug.h>
#include <config.h>
#include <driver_task.h>
#include <drv_timer.h>

#include <xhci/xhci.h>
#include <xhci/xhci-events.h>
#include <xhci/xhci-commands.h>


static void UnitTask(struct XHCIUnit *unit, struct Task *parent)
{
    unit->irq_signal = -1;

    // Initialize the built in msg port, we'll receive commands here
    BYTE msg_sigbit = drv_unit_msgport_init(&unit->unit);
    if (msg_sigbit == -1)
    {
        Kprintf("[xhci] %s: Failed to allocate message signal\n", __func__);
        goto free_signals;
    }

    // Allocate signal for interrupt handler
    unit->irq_signal = AllocSignal(-1);
    if(unit->irq_signal == -1)
    {
        Kprintf("[xhci] %s: Failed to allocate event signal\n", __func__);
        goto free_signals;
    }

    /* Periodic tick: drives the command-ring and transfer-deadline timeout scans */
    struct drv_timer tick;
    if (!drv_timer_open(&tick))
    {
        Kprintf("[xhci] %s: Failed to open timer device\n", __func__);
        goto free_signals;
    }
    drv_timer_arm_ms(&tick, UNIT_TASK_POLL_DELAY_MS);

    /* The root-hub port waits' sleep timer: task-bound, so opened (and later
     * closed) here.  Failure is non-fatal — the waits degrade to hot polls. */
    if (!drv_timer_open(&unit->xhci_ctrl->sleep_timer))
        Kprintf("[xhci] %s: no sleep timer; port waits will spin\n", __func__);

    unit->task = FindTask(NULL);
    /* Signal parent that Unit task is up and running now */
    Signal(parent, SIGBREAKF_CTRL_F);

    KprintfT("[xhci] %s: Entering main unit task loop\n", __func__);

    ULONG sigset;
    ULONG waitMask = (1UL << unit->unit.unit_MsgPort.mp_SigBit) |
                     drv_timer_sigmask(&tick) |
                     (1UL << unit->irq_signal) |
                     SIGBREAKF_CTRL_C;

    do
    {
        sigset = Wait(waitMask);

        /* Every work block runs under the transfer-plane lock: the direct
         * submit/abort entries touch the same rings, TD lists and pools
         * from other tasks' contexts. */
        if(sigset & (1UL << unit->irq_signal))
        {
            struct xhci_ctrl *ctrl = unit->xhci_ctrl;
#ifdef PROFILE
            if (ctrl->irq_t0)
            {
                perf_add(&ctrl->perf, XP_IRQ_TO_TASK, ctrl->irq_t0);
                ctrl->irq_t0 = 0;
            }
#endif
            lock_prof_obtain(&ctrl->lockProf, &ctrl->xfer_lock);
            PERF_T0(drain_t0);
            xhci_process_event_trb(ctrl);
            PERF_ADD(&ctrl->perf, XP_EVT_DRAIN, drain_t0);
            lock_prof_release(&ctrl->lockProf, &ctrl->xfer_lock);
            xhci_int_rearm(unit);
        }

        // IO queue got a new message
        if (sigset & (1UL << unit->unit.unit_MsgPort.mp_SigBit))
        {
            struct IORequest *io;
            // Drain command queue and process it
            while ((io = (struct IORequest *)GetMsg(&unit->unit.unit_MsgPort)))
            {
                lock_prof_obtain(&unit->xhci_ctrl->lockProf, &unit->xhci_ctrl->xfer_lock);
                ProcessCommand(io);
                lock_prof_release(&unit->xhci_ctrl->lockProf, &unit->xhci_ctrl->xfer_lock);
            }
        }


        // Timer expired, time to check timeouts
        if (sigset & drv_timer_sigmask(&tick))
        {
            drv_timer_consume(&tick);

            lock_prof_obtain(&unit->xhci_ctrl->lockProf, &unit->xhci_ctrl->xfer_lock);
            xhci_process_command_timeouts(unit->xhci_ctrl);
            xhci_process_event_timeouts(unit->xhci_ctrl);
            lock_prof_release(&unit->xhci_ctrl->lockProf, &unit->xhci_ctrl->xfer_lock);

#ifdef PROFILE
            if (++unit->xhci_ctrl->profTicks >= XHCI_PROF_REPORT_TICKS)
            {
                unit->xhci_ctrl->profTicks = 0;
                perf_report(&unit->xhci_ctrl->perf);
                lock_prof_report(&unit->xhci_ctrl->lockProf);
            }
#endif

            drv_timer_arm_ms(&tick, UNIT_TASK_POLL_DELAY_MS);
        }

        if (sigset & SIGBREAKF_CTRL_C)
        {
            KprintfT("[xhci] %s: Received SIGBREAKF_CTRL_C, stopping xhci task\n", __func__);
            drv_timer_cancel(&tick);
        }
    } while ((sigset & SIGBREAKF_CTRL_C) == 0);

    drv_timer_close(&unit->xhci_ctrl->sleep_timer);
    drv_timer_close(&tick);
free_signals:
    /* Reachable before every signal exists — freeing an unallocated -1 (or
     * worse, the cleared struct's bit 0, which belongs to Exec) must not
     * happen. */
    if (unit->irq_signal != -1)
        FreeSignal(unit->irq_signal);
    if (msg_sigbit != -1)
        FreeSignal(msg_sigbit);

    /* drv_task_exit clears the liveness slot first (drv_task_join polls it),
     * then reports CTRL_F for a task that ran / CTRL_C for one that never got
     * to its loop. */
    drv_task_exit(&unit->task, parent, unit->task != NULL);
}

s32 UnitTaskStart(struct XHCIUnit *unit)
{
    return drv_task_spawn(unit, UnitTask, "XHCI USB driver",
                          STACK_SIZE, UNIT_TASK_PRIORITY) == 0
               ? UHIOERR_NO_ERROR
               : UHIOERR_HOSTERROR;
}

void UnitTaskStop(struct XHCIUnit *unit)
{
    drv_task_join(&unit->task);
}
