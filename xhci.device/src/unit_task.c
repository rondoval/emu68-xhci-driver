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
#include <xhci/xhci-events.h>

#include <compat.h>
#include <device.h>
#include <minlist.h>
#include <debug.h>
#include <config.h>

static void UnitTask(struct XHCIUnit *unit, struct Task *parent)
{
    // Initialize the built in msg port, we'll receive commands here
    _NewMinList((struct MinList *)&unit->unit.unit_MsgPort.mp_MsgList);
    unit->unit.unit_MsgPort.mp_SigTask = FindTask(NULL);
    unit->unit.unit_MsgPort.mp_SigBit = AllocSignal(-1);
    unit->unit.unit_MsgPort.mp_Flags = PA_SIGNAL;
    unit->unit.unit_MsgPort.mp_Node.ln_Type = NT_MSGPORT;

    // Allocate singal for interrupt handler
    unit->irq_signal = AllocSignal(-1);
    if(unit->irq_signal == -1)
    {
        Kprintf("[xhci] %s: Failed to allocate event signal\n", __func__);
        goto free_signals;
    }

    // Create a watchdog timer
    struct MsgPort *microHZTimerPort = CreateMsgPort();
    struct timerequest *packetTimerReq = CreateIORequest(microHZTimerPort, sizeof(struct timerequest));
    if (microHZTimerPort == NULL || packetTimerReq == NULL)
    {
        Kprintf("[xhci] %s: Failed to create timer msg port or request\n", __func__);
        goto free_ports;
    }

    UBYTE ret = OpenDevice((CONST_STRPTR)TIMERNAME, UNIT_MICROHZ, (struct IORequest *)packetTimerReq, LIB_MIN_VERSION);
    if (ret)
    {
        Kprintf("[xhci] %s: Failed to open timer device ret=%ld\n", __func__, ret);
        goto free_ports;
    }

    const ULONG delay = UNIT_TASK_POLL_DELAY_MS * 1000;

    // Set a timer... we need to pull on RX
    packetTimerReq->tr_node.io_Command = TR_ADDREQUEST;
    packetTimerReq->tr_time.tv_secs = 0;
    packetTimerReq->tr_time.tv_micro = delay;
    SendIO(&packetTimerReq->tr_node);

    unit->task = FindTask(NULL);
    /* Signal parent that Unit task is up and running now */
    Signal(parent, SIGBREAKF_CTRL_F);

    Kprintf("[genet] %s: Entering main unit task loo\n", __func__);

    ULONG sigset;
    ULONG waitMask = (1UL << unit->unit.unit_MsgPort.mp_SigBit) |
                     (1UL << microHZTimerPort->mp_SigBit) |
                     (1UL << unit->irq_signal) |
                     SIGBREAKF_CTRL_C;

    do
    {
        sigset = Wait(waitMask);

        // IO queue got a new message
        if (sigset & (1UL << unit->unit.unit_MsgPort.mp_SigBit))
        {
            struct IOUsbHWReq *io;
            // Drain command queue and process it
            while ((io = (struct IOUsbHWReq *)GetMsg(&unit->unit.unit_MsgPort)))
            {
                ProcessCommand(io);
            }
        }

        if(sigset & (1UL << unit->irq_signal))
        {
            xhci_intx_handle(unit);
        }

        // Timer expired, time to check timeouts
        if (sigset & (1UL << microHZTimerPort->mp_SigBit))
        {
            if (CheckIO(&packetTimerReq->tr_node))
            {
                WaitIO(&packetTimerReq->tr_node);
            }

            xhci_process_event_timeouts(unit->xhci_ctrl);

            //TODO enable irq, just in case

            /* Re-arm timer */
            packetTimerReq->tr_node.io_Command = TR_ADDREQUEST;
            packetTimerReq->tr_time.tv_secs = 0;
            packetTimerReq->tr_time.tv_micro = delay;
            SendIO(&packetTimerReq->tr_node);
        }

        if (sigset & SIGBREAKF_CTRL_C)
        {
            Kprintf("[xhci] %s: Received SIGBREAKF_CTRL_C, stopping xhci task\n", __func__);
            AbortIO(&packetTimerReq->tr_node);
            WaitIO(&packetTimerReq->tr_node);
        }
    } while ((sigset & SIGBREAKF_CTRL_C) == 0);

    CloseDevice(&packetTimerReq->tr_node);
free_ports:
    DeleteIORequest((struct IORequest *)packetTimerReq);
    DeleteIORequest(&packetTimerReq->tr_node);
    DeleteMsgPort(microHZTimerPort);
free_signals:
    FreeSignal(unit->irq_signal);
    FreeSignal(unit->unit.unit_MsgPort.mp_SigBit);

    Signal(parent, SIGBREAKF_CTRL_C);
    unit->task = NULL;
}

int UnitTaskStart(struct XHCIUnit *unit)
{
    Kprintf("[xhci] %s: xhci task starting\n", __func__);

    // Get all memory we need for the receiver task
    struct MemList *ml = AllocMem(sizeof(struct MemList) + sizeof(struct MemEntry), MEMF_PUBLIC | MEMF_CLEAR);
    struct Task *task = AllocMem(sizeof(struct Task), MEMF_PUBLIC | MEMF_CLEAR);
    ULONG *stack = AllocMem(STACK_SIZE, MEMF_PUBLIC | MEMF_CLEAR);
    if (!ml || !task || !stack)
    {
        Kprintf("[xhci] %s: Failed to allocate memory for xhci task\n", __func__);
        if (ml)
            FreeMem(ml, sizeof(struct MemList) + sizeof(struct MemEntry));
        if (task)
            FreeMem(task, sizeof(struct Task));
        if (stack)
            FreeMem(stack, STACK_SIZE);
        return UHIOERR_HOSTERROR;
    }

    // Prepare mem list, put task and its stack there
    ml->ml_NumEntries = 2;
    ml->ml_ME[0].me_Un.meu_Addr = task;
    ml->ml_ME[0].me_Length = sizeof(struct Task);

    ml->ml_ME[1].me_Un.meu_Addr = &stack[0];
    ml->ml_ME[1].me_Length = STACK_SIZE;

    // Set up stack
    task->tc_SPLower = &stack[0];
    task->tc_SPUpper = &stack[STACK_SIZE / sizeof(ULONG)];

    // Push ThisTask and Unit on the stack
    stack = (ULONG *)task->tc_SPUpper;
    *--stack = (ULONG)FindTask(NULL);
    *--stack = (ULONG)unit;
    task->tc_SPReg = stack;

    task->tc_Node.ln_Name = "xhci rx/tx";
    task->tc_Node.ln_Type = NT_TASK;
    task->tc_Node.ln_Pri = UNIT_TASK_PRIORITY;

    _NewMinList((struct MinList *)&task->tc_MemEntry);
    AddHead(&task->tc_MemEntry, &ml->ml_Node);

    SetSignal(0UL, SIGBREAKF_CTRL_F);

    APTR result = AddTask(task, UnitTask, NULL);
    if (result == NULL)
    {
        Kprintf("[xhci] %s: Failed to add xhci task\n", __func__);
        FreeMem(ml, sizeof(struct MemList) + sizeof(struct MemEntry));
        FreeMem(task, sizeof(struct Task));
        FreeMem(&stack[0], STACK_SIZE);
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

    SetSignal(0UL, SIGBREAKF_CTRL_F);

    CloseDevice(&timerReq->tr_node);
    DeleteIORequest(&timerReq->tr_node);
    DeleteMsgPort(timerPort);

    Kprintf("[xhci] %s: xhci task stopped\n", __func__);
}