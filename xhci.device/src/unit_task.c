// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/timer_protos.h>
#else
#include <proto/exec.h>
#include <proto/timer.h>
#endif

#include <dos/dos.h>

#include <devices/usbhardware.h>

#include <compat.h>
#include <device.h>
#include <minlist.h>
#include <debug.h>

struct Device *TimerBase = NULL;

static inline BOOL ProcessReceive(struct XHCIUnit *unit)
{
    // TODO implement
    (void)unit;
    return FALSE;
}

static void UnitTask(struct XHCIUnit *unit, struct Task *parent)
{
    // Initialize the built in msg port, we'll receive commands here
    _NewMinList((struct MinList *)&unit->unit.unit_MsgPort.mp_MsgList);
    unit->unit.unit_MsgPort.mp_SigTask = FindTask(NULL);
    unit->unit.unit_MsgPort.mp_SigBit = AllocSignal(-1);
    unit->unit.unit_MsgPort.mp_Flags = PA_SIGNAL;
    unit->unit.unit_MsgPort.mp_Node.ln_Type = NT_MSGPORT;

    // Create a timer, we'll use it to poll the PHY
    struct MsgPort *microHZTimerPort = CreateMsgPort();
    struct MsgPort *vblankTimerPort = CreateMsgPort();
    struct timerequest *packetTimerReq = CreateIORequest(microHZTimerPort, sizeof(struct timerequest));
    struct timerequest *statsTimerReq = CreateIORequest(vblankTimerPort, sizeof(struct timerequest));
    if (microHZTimerPort == NULL || vblankTimerPort == NULL || packetTimerReq == NULL || statsTimerReq == NULL)
    {
        Kprintf("[xhci] %s: Failed to create timer msg port or request\n", __func__);
        DeleteMsgPort(microHZTimerPort);
        DeleteMsgPort(vblankTimerPort);
        DeleteIORequest((struct IORequest *)packetTimerReq);
        DeleteIORequest((struct IORequest *)statsTimerReq);
        Signal(parent, SIGBREAKF_CTRL_C);
        return;
    }

    UBYTE ret = OpenDevice((CONST_STRPTR)TIMERNAME, UNIT_MICROHZ, (struct IORequest *)packetTimerReq, LIB_MIN_VERSION);
    UBYTE ret2 = OpenDevice((CONST_STRPTR)TIMERNAME, UNIT_VBLANK, (struct IORequest *)statsTimerReq, LIB_MIN_VERSION);
    if (ret || ret2)
    {
        Kprintf("[xhci] %s: Failed to open timer device ret=%ld, %ld\n", __func__, ret, ret2);
        DeleteMsgPort(microHZTimerPort);
        DeleteMsgPort(vblankTimerPort);
        DeleteIORequest((struct IORequest *)packetTimerReq);
        DeleteIORequest((struct IORequest *)statsTimerReq);
        Signal(parent, SIGBREAKF_CTRL_C);
        return;
    }

    /* used to reset stats on S2_ONLINE */
    TimerBase = packetTimerReq->tr_node.io_Device;

    // ULONG backoff_idx = xhciConfig.poll_delay_len - 1; /* Start conservative until first activity */
    // ULONG delay = xhciConfig.poll_delay_us[backoff_idx];
    //TODO make configurable
    ULONG delay = 500;

    // Set a timer... we need to pull on RX
    packetTimerReq->tr_node.io_Command = TR_ADDREQUEST;
    packetTimerReq->tr_time.tv_secs = 0;
    packetTimerReq->tr_time.tv_micro = delay;
    SendIO(&packetTimerReq->tr_node);

    statsTimerReq->tr_node.io_Command = TR_ADDREQUEST;
    statsTimerReq->tr_time.tv_secs = 15;
    statsTimerReq->tr_time.tv_micro = 0;
    SendIO(&statsTimerReq->tr_node);

    unit->task = FindTask(NULL);
    /* Signal parent that Unit task is up and running now */
    Signal(parent, SIGBREAKF_CTRL_F);

    ULONG sigset;
    BOOL activity = FALSE;
    ULONG waitMask = (1UL << unit->unit.unit_MsgPort.mp_SigBit) |
                     (1UL << microHZTimerPort->mp_SigBit) |
                     (1UL << vblankTimerPort->mp_SigBit) |
                     SIGBREAKF_CTRL_C;

    do
    {
        sigset = Wait(waitMask);

        // IO queue got a new message
        if (sigset & (1UL << unit->unit.unit_MsgPort.mp_SigBit))
        {
            activity = TRUE;
            struct IOUsbHWReq *io;
            // Drain command queue and process it
            while ((io = (struct IOUsbHWReq *)GetMsg(&unit->unit.unit_MsgPort)))
            {
                ProcessCommand(io);
            }
        }

        activity |= ProcessReceive(unit);

        // Timer expired, query PHY for link state
        if (sigset & (1UL << microHZTimerPort->mp_SigBit))
        {
            if (CheckIO(&packetTimerReq->tr_node))
            {
                WaitIO(&packetTimerReq->tr_node);
            }

            /* TODO Periodic TX reclaim */
            // if (unit->state == STATE_ONLINE)
            //     bcmxhci_tx_reclaim(unit);

            // TODO pool PHY for state

            // if (activity /*|| unit->tx_watchdog_fast_ticks*/)
            // {
            //     backoff_idx = 0;
            //     // if (unit->tx_watchdog_fast_ticks)
            //     //     --unit->tx_watchdog_fast_ticks;
            // }
            // else
            // {
            //     if (backoff_idx + 1 < xhciConfig.poll_delay_len)
            //         backoff_idx++;
            // }
            activity = FALSE; /* reset activity */
            // delay = xhciConfig.poll_delay_us[backoff_idx];
            delay = 500; //TODO make configurable

            /* TX watchdog soft cap: ensure we never sleep beyond this while descriptors outstanding */
            // if (unit->tx_ring.free_bds < TX_DESCS && delay > xhciConfig.tx_reclaim_soft_us)
            //     delay = xhciConfig.tx_reclaim_soft_us;

            /* Re-arm timer */
            packetTimerReq->tr_node.io_Command = TR_ADDREQUEST;
            packetTimerReq->tr_time.tv_secs = 0;
            packetTimerReq->tr_time.tv_micro = delay;
            SendIO(&packetTimerReq->tr_node);
        }

        if (sigset & (1UL << vblankTimerPort->mp_SigBit))
        {
            if(CheckIO(&statsTimerReq->tr_node))
            {
                WaitIO(&statsTimerReq->tr_node);
            }
            //TODO is this needed

            statsTimerReq->tr_node.io_Command = TR_ADDREQUEST;
            statsTimerReq->tr_time.tv_secs = 15;
            statsTimerReq->tr_time.tv_micro = 0;
            SendIO(&statsTimerReq->tr_node);
        }
        if (sigset & SIGBREAKF_CTRL_C)
        {
            Kprintf("[xhci] %s: Received SIGBREAKF_CTRL_C, stopping xhci task\n", __func__);
            AbortIO(&packetTimerReq->tr_node);
            WaitIO(&packetTimerReq->tr_node);
            AbortIO(&statsTimerReq->tr_node);
            WaitIO(&statsTimerReq->tr_node);
        }
    } while ((sigset & SIGBREAKF_CTRL_C) == 0);

    FreeSignal(unit->unit.unit_MsgPort.mp_SigBit);
    CloseDevice(&packetTimerReq->tr_node);
    DeleteIORequest(&packetTimerReq->tr_node);
    DeleteIORequest(&statsTimerReq->tr_node);
    DeleteMsgPort(microHZTimerPort);
    DeleteMsgPort(vblankTimerPort);
    unit->task = NULL;
}

int UnitTaskStart(struct XHCIUnit *unit)
{
    Kprintf("[xhci] %s: xhci task starting\n", __func__);

    // Get all memory we need for the receiver task
    struct MemList *ml = AllocMem(sizeof(struct MemList) + sizeof(struct MemEntry), MEMF_PUBLIC | MEMF_CLEAR);
    struct Task *task = AllocMem(sizeof(struct Task), MEMF_PUBLIC | MEMF_CLEAR);
    ULONG *stack = AllocMem(/* TODO xhciConfig.unit_stack_bytes*/65535, MEMF_PUBLIC | MEMF_CLEAR);
    if (!ml || !task || !stack)
    {
        Kprintf("[xhci] %s: Failed to allocate memory for xhci task\n", __func__);
        if (ml)
            FreeMem(ml, sizeof(struct MemList) + sizeof(struct MemEntry));
        if (task)
            FreeMem(task, sizeof(struct Task));
        if (stack)
            FreeMem(stack, /* TODO xhciConfig.unit_stack_bytes*/65535);
        return UHIOERR_HOSTERROR;
    }

    // Prepare mem list, put task and its stack there
    ml->ml_NumEntries = 2;
    ml->ml_ME[0].me_Un.meu_Addr = task;
    ml->ml_ME[0].me_Length = sizeof(struct Task);

    ml->ml_ME[1].me_Un.meu_Addr = &stack[0];
    ml->ml_ME[1].me_Length = /* TODO xhciConfig.unit_stack_bytes*/65535;

    // Set up stack
    task->tc_SPLower = &stack[0];
    task->tc_SPUpper = &stack[65535 /* TODO */ / sizeof(ULONG)];

    // Push ThisTask and Unit on the stack
    stack = (ULONG *)task->tc_SPUpper;
    *--stack = (ULONG)FindTask(NULL);
    *--stack = (ULONG)unit;
    task->tc_SPReg = stack;

    task->tc_Node.ln_Name = "xhci rx/tx";
    task->tc_Node.ln_Type = NT_TASK;
    task->tc_Node.ln_Pri = 0/*TODO xhciConfig.unit_task_priority*/;

    _NewMinList((struct MinList *)&task->tc_MemEntry);
    AddHead(&task->tc_MemEntry, &ml->ml_Node);

    APTR result = AddTask(task, UnitTask, NULL);
    if (result == NULL)
    {
        Kprintf("[xhci] %s: Failed to add xhci task\n", __func__);
        FreeMem(ml, sizeof(struct MemList) + sizeof(struct MemEntry));
        FreeMem(task, sizeof(struct Task));
        FreeMem(&stack[0], /* TODO xhciConfig.unit_stack_bytes*/65535);
        return UHIOERR_HOSTERROR;
    }

    Wait(SIGBREAKF_CTRL_F);
    Kprintf("[xhci] %s: xhci task started\n", __func__);
    return UHIOERR_NO_ERROR;
}

void UnitTaskStop(struct XHCIUnit *unit)
{
    Kprintf("[xhci] %s: xhci task stopping\n", __func__);

    struct MsgPort *timerPort = CreateMsgPort();
    struct timerequest *timerReq = CreateIORequest(timerPort, sizeof(struct timerequest));

    if (timerPort != NULL && timerReq != NULL)
    {
        BYTE result = OpenDevice((CONST_STRPTR) "timer.device", UNIT_VBLANK, (struct IORequest *)timerReq, LIB_MIN_VERSION);
        if (result != NULL)
        {
            Kprintf("[xhci] %s: Failed to open timer device: %ld\n", __func__, result);
            // We'll continue anyway
        }
    }

    Signal(unit->task, SIGBREAKF_CTRL_C);
    do
    {
        timerReq->tr_node.io_Command = TR_ADDREQUEST;
        timerReq->tr_time.tv_secs = 0;
        timerReq->tr_time.tv_micro = 250000;
        DoIO(&timerReq->tr_node);
    } while (unit->task != NULL);

    CloseDevice(&timerReq->tr_node);
    DeleteIORequest(&timerReq->tr_node);
    DeleteMsgPort(timerPort);

    Kprintf("[xhci] %s: xhci task stopped\n", __func__);
}