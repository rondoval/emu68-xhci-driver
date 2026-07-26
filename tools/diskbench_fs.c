/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * diskbench_fs.c — filesystem engine: packet I/O on <DRIVE>diskbench.tmp.
 *
 * These are the application-visible numbers. There is no O_DIRECT on Amiga:
 * handler and cache behaviour (FFS/PFS/SFS/fat95/NTFS buffering, write-behind)
 * is deliberately part of the measured stack.
 *
 * Queue depth is real here. dos.library Read()/Write()/Seek() are nothing but
 * synchronous ACTION_READ/ACTION_WRITE/ACTION_SEEK packets, so this engine
 * sends those packets itself and keeps QD of them outstanding. Each slot needs
 * its own file handle: a handler tracks one seek position per open file
 * object, so slots sharing a handle would fight over it. What the depth buys
 * depends on the filesystem — a single-threaded handler still serializes the
 * packets, but it no longer idles for a task switch between them, and handlers
 * with their own read-ahead or write-behind (PFS, SFS) can overlap further.
 *
 * DOS packets cannot be aborted, so ^C stops resubmission and drains.
 *
 * Sequential slots each own a contiguous slice of the file and walk it, so a
 * point costs exactly one packet per op (like the old synchronous path) and QD1
 * numbers stay comparable. Random slots seek anywhere in the file, paying the
 * SEEK+data packet pair that an application also pays.
 *
 * The file is materialized once up-front with real writes (no sparse tricks),
 * so the timed write passes measure steady-state overwrite, not allocation —
 * slots never extend it. Fragmentation is whatever the volume gives us: noted,
 * not controlled (dev tool).
 */

#include <stdio.h>
#include <string.h>

#include <exec/types.h>
#include <exec/memory.h>
#include <dos/dos.h>
#include <dos/dosextens.h>

#include <proto/exec.h>
#include <proto/dos.h>

#include "diskbench.h"

#define DB_CREATE_CHUNK 1048576UL

struct FsSlot
{
    struct StandardPacket sl_Sp; /* first member: GetMsg() casts back to the slot */
    BPTR sl_File;
    LONG sl_Arg1;   /* the handler's file object behind sl_File */
    UBYTE *sl_Buf;
    db_u64 sl_Submit;
    db_u64 sl_Pos;  /* next offset this slot transfers */
    db_u64 sl_Base; /* the slot's slice of the file (sequential) */
    db_u64 sl_End;
    BOOL sl_Seek;   /* an ACTION_SEEK is outstanding; the data op follows */
};

static struct MsgPort *fs_port;    /* replies land here */
static struct MsgPort *fs_handler; /* the filesystem's packet port */
static struct FsSlot fs_slots[DB_MAX_QD];
static ULONG fs_nslots;

static BOOL db_break(void)
{
    return (BOOL)((SetSignal(0, SIGBREAKF_CTRL_C) & SIGBREAKF_CTRL_C) != 0);
}

/* create or validate <drive:>diskbench.tmp at cfg->size */
static BOOL db_fs_prepare(const struct BenchCfg *cfg, const char *path)
{
    BPTR fh = Open((CONST_STRPTR)path, MODE_OLDFILE);
    if (fh != 0)
    {
        /* Seek() returns the OLD position, so the second call yields the size */
        if (Seek(fh, 0, OFFSET_END) >= 0)
        {
            LONG fsize = Seek(fh, 0, OFFSET_BEGINNING);
            if (fsize >= 0 && (db_u64)fsize == cfg->size)
            {
                printf("fs: reusing %s\n", path);
                Close(fh);
                return TRUE;
            }
        }
        Close(fh);
    }

    APTR base;
    ULONG basesize;
    UBYTE *buf = db_alloc_aligned(DB_CREATE_CHUNK, MEMF_PUBLIC, &base, &basesize);
    if (buf == NULL)
    {
        printf("diskbench: no memory for the %lu byte create buffer\n",
               (unsigned long)DB_CREATE_CHUNK);
        return FALSE;
    }
    /* deterministic non-zero pattern: nothing here should compress or go sparse */
    ULONG rng = cfg->seed;
    for (ULONG i = 0; i < DB_CREATE_CHUNK / 4; i++)
        ((ULONG *)(void *)buf)[i] = db_lcg_next(&rng);

    char sizestr[12];
    db_fmt_size((ULONG)cfg->size, sizestr);
    printf("fs: creating %s (%s)...", path, sizestr);
    fflush(stdout);

    BOOL ok = FALSE;
    fh = Open((CONST_STRPTR)path, MODE_NEWFILE);
    if (fh == 0)
    {
        printf("\n");
        PrintFault(IoErr(), (CONST_STRPTR)"diskbench: create");
        goto out;
    }
    for (db_u64 done = 0; done < cfg->size; done += DB_CREATE_CHUNK)
    {
        LONG chunk = (LONG)DB_CREATE_CHUNK;
        if (cfg->size - done < DB_CREATE_CHUNK)
            chunk = (LONG)(cfg->size - done);
        if (db_break())
        {
            printf(" aborted\n");
            goto out_kill;
        }
        if (Write(fh, buf, chunk) != chunk)
        {
            printf("\n");
            PrintFault(IoErr(), (CONST_STRPTR)"diskbench: write");
            goto out_kill;
        }
    }
    printf(" done\n");
    Close(fh);
    ok = TRUE;
    goto out;

out_kill:
    Close(fh);
    DeleteFile((CONST_STRPTR)path);
out:
    FreeMem(base, basesize);
    return ok;
}

static void db_fs_close(void)
{
    for (ULONG i = 0; i < fs_nslots; i++)
    {
        if (fs_slots[i].sl_File != 0)
        {
            Close(fs_slots[i].sl_File);
            fs_slots[i].sl_File = 0;
        }
    }
    fs_nslots = 0;
    fs_handler = NULL;
    if (fs_port != NULL)
    {
        DeleteMsgPort(fs_port);
        fs_port = NULL;
    }
}

/* one handle per slot: a handler keeps one seek position per open file object */
static LONG db_fs_open(const char *path, ULONG n)
{
    fs_port = CreateMsgPort();
    if (fs_port == NULL)
        return RETURN_ERROR;

    for (ULONG i = 0; i < n; i++)
    {
        struct FsSlot *s = &fs_slots[i];
        s->sl_File = Open((CONST_STRPTR)path, MODE_OLDFILE);
        if (s->sl_File == 0)
        {
            if (i == 0)
            {
                PrintFault(IoErr(), (CONST_STRPTR)"diskbench: fs open");
                db_fs_close();
                return RETURN_ERROR;
            }
            /* a handler that caps concurrent opens caps the sweep, not the run */
            printf("fs: only %lu concurrent handles on this filesystem\n",
                   (unsigned long)i);
            break;
        }
        struct FileHandle *fh = BADDR(s->sl_File);
        s->sl_Arg1 = fh->fh_Arg1;
        if (fs_handler == NULL)
            fs_handler = fh->fh_Type;

        memset(&s->sl_Sp, 0, sizeof(s->sl_Sp));
        s->sl_Sp.sp_Msg.mn_Node.ln_Name = (APTR)&s->sl_Sp.sp_Pkt;
        s->sl_Sp.sp_Msg.mn_Node.ln_Type = NT_MESSAGE;
        s->sl_Sp.sp_Msg.mn_Length = sizeof(struct StandardPacket);
        s->sl_Sp.sp_Msg.mn_ReplyPort = fs_port;
        s->sl_Sp.sp_Pkt.dp_Link = &s->sl_Sp.sp_Msg;
        fs_nslots = i + 1;
    }
    return RETURN_OK;
}

static void db_fs_send(struct FsSlot *s, LONG action, LONG arg2, LONG arg3)
{
    struct DosPacket *dp = &s->sl_Sp.sp_Pkt;
    dp->dp_Type = action;
    dp->dp_Arg1 = s->sl_Arg1;
    dp->dp_Arg2 = arg2;
    dp->dp_Arg3 = arg3;
    dp->dp_Res1 = 0;
    dp->dp_Res2 = 0;
    /* the reply leaves the handler's port in dp_Port: restore ours every time */
    dp->dp_Port = fs_port;
    PutMsg(fs_handler, &s->sl_Sp.sp_Msg);
}

/* start one op; random pays a SEEK first, sequential only on wrap */
static void db_fs_start(struct FsSlot *s, LONG act, ULONG bs, BOOL rnd,
                        ULONG *rng, ULONG nblk)
{
    s->sl_Submit = db_now();
    if (rnd)
        s->sl_Pos = (db_u64)db_lcg_range(rng, nblk) * bs;
    else if (s->sl_Pos + bs <= s->sl_End)
    {
        db_fs_send(s, act, (LONG)s->sl_Buf, (LONG)bs);
        return;
    }
    else
        s->sl_Pos = s->sl_Base;

    s->sl_Seek = TRUE;
    db_fs_send(s, ACTION_SEEK, (LONG)s->sl_Pos, OFFSET_BEGINNING);
}

/* one measured point; kind: 0=seqrd 1=seqwr 2=rndrd 3=rndwr */
static LONG db_fs_point(const struct BenchCfg *cfg, ULONG kind, ULONG bs, ULONG qd)
{
    static const char *const names[] = {"seqrd", "seqwr", "rndrd", "rndwr"};
    const BOOL rnd = (BOOL)(kind >= 2);
    const LONG act = ((kind & 1) != 0) ? ACTION_WRITE : ACTION_READ;
    const ULONG nblk = (ULONG)(cfg->size / bs);

    if (qd > fs_nslots)
    {
        printf("fs: skip qd=%lu: only %lu file handles\n",
               (unsigned long)qd, (unsigned long)fs_nslots);
        return RETURN_OK;
    }
    if (nblk < qd)
    {
        printf("fs: skip bs=%lu qd=%lu: file too small\n",
               (unsigned long)bs, (unsigned long)qd);
        return RETURN_OK;
    }
    if ((db_u64)qd * bs > 0x20000000ULL) /* keep the buffer set inside a LONG */
    {
        printf("fs: skip bs=%lu qd=%lu: buffer set above 512M\n",
               (unsigned long)bs, (unsigned long)qd);
        return RETURN_OK;
    }

    APTR base;
    ULONG basesize;
    UBYTE *buf = db_alloc_aligned(qd * bs + 4,
                                  MEMF_PUBLIC | (cfg->chip ? MEMF_CHIP : MEMF_FAST),
                                  &base, &basesize);
    if (buf == NULL)
    {
        printf("fs: skip bs=%lu qd=%lu: no memory for %lu byte buffer\n",
               (unsigned long)bs, (unsigned long)qd, (unsigned long)(qd * bs));
        return RETURN_OK;
    }
    if (cfg->misalign)
        buf++;
    memset(buf, 0xA5, qd * bs);

    struct BenchStat st;
    db_stat_reset(&st);

    /* sequential: one contiguous slice each, so only wraps cost a seek */
    db_u64 chunk = (db_u64)(nblk / qd) * bs;
    for (ULONG i = 0; i < qd; i++)
    {
        struct FsSlot *s = &fs_slots[i];
        s->sl_Buf = buf + i * bs;
        s->sl_Base = rnd ? 0 : (db_u64)i * chunk;
        s->sl_End = rnd ? cfg->size : s->sl_Base + chunk;
        /* start out of range so the first op seeks the handle down to sl_Base:
         * it is still wherever the previous point left it */
        s->sl_Pos = s->sl_End;
        s->sl_Seek = FALSE;
    }

    LONG rc = RETURN_OK;
    ULONG rng = cfg->seed;
    db_u64 warm_ticks = (db_u64)cfg->secs * db_efreq / 4;
    if (warm_ticks < db_efreq / 2)
        warm_ticks = db_efreq / 2;
    db_u64 warm_end = db_now() + warm_ticks;
    db_u64 start = 0;
    db_u64 deadline = ~0ULL;
    db_u64 last = 0;
    LONG err = 0;
    BOOL recording = FALSE;
    BOOL stop = FALSE;
    ULONG inflight = 0;

    for (ULONG i = 0; i < qd; i++)
    {
        db_fs_start(&fs_slots[i], act, bs, rnd, &rng, nblk);
        inflight++;
    }

    while (inflight > 0)
    {
        ULONG sig = Wait((1UL << fs_port->mp_SigBit) | SIGBREAKF_CTRL_C);
        if ((sig & SIGBREAKF_CTRL_C) != 0 && !stop)
        {
            printf("fs: ^C\n"); /* DOS packets cannot be aborted: drain */
            stop = TRUE;
            rc = RETURN_WARN;
        }

        struct FsSlot *s;
        while ((s = (struct FsSlot *)GetMsg(fs_port)) != NULL)
        {
            struct DosPacket *dp = &s->sl_Sp.sp_Pkt;
            db_u64 now = db_now();

            if (s->sl_Seek)
            {
                s->sl_Seek = FALSE;
                if (dp->dp_Res1 >= 0)
                {
                    /* the data half is still the same op: clock keeps running */
                    db_fs_send(s, act, (LONG)s->sl_Buf, (LONG)bs);
                    continue;
                }
            }
            else if (dp->dp_Res1 == (LONG)bs)
            {
                if (!rnd)
                    s->sl_Pos += bs;
                if (recording)
                {
                    st.bytes += bs;
                    st.ops++;
                    db_stat_lat(&st, now - s->sl_Submit);
                    last = now;
                }
                else if (now >= warm_end)
                {
                    db_stat_reset(&st);
                    start = now;
                    last = now;
                    deadline = now + (db_u64)cfg->secs * db_efreq;
                    recording = TRUE;
                }

                if (!stop && now < deadline)
                {
                    db_fs_start(s, act, bs, rnd, &rng, nblk);
                    continue;
                }
                inflight--;
                continue;
            }

            /* seek or transfer failed: stop feeding and drain the rest */
            if (err == 0)
                err = dp->dp_Res2;
            stop = TRUE;
            rc = RETURN_ERROR;
            inflight--;
        }
    }

    if (rc == RETURN_ERROR)
    {
        PrintFault(err, (CONST_STRPTR)"diskbench: fs I/O");
    }
    else if (rc == RETURN_OK)
    {
        st.ticks = (last > start) ? last - start : 0;
        struct BenchPoint pt = {"fs", names[kind], bs, qd, NULL};
        db_report(&pt, &st);
    }

    FreeMem(base, basesize);
    return rc;
}

LONG db_run_fs(struct BenchCfg *cfg)
{
    char path[DB_NAME_LEN + 16];
    sprintf(path, "%sdiskbench.tmp", cfg->drive);

    if (!db_fs_prepare(cfg, path))
        return RETURN_ERROR;

    ULONG maxqd = 1;
    for (ULONG i = 0; i < cfg->n_qd; i++)
    {
        if (cfg->qd[i] > maxqd)
            maxqd = cfg->qd[i];
    }

    LONG rc = db_fs_open(path, maxqd);
    if (rc != RETURN_OK)
        goto out;

    if ((cfg->tests & DB_T_FSSEQ) != 0)
    {
        for (ULONG k = 0; k <= 1 && rc == RETURN_OK; k++) /* seqrd, then seqwr */
        {
            for (ULONG i = 0; i < cfg->n_bs && rc == RETURN_OK; i++)
            {
                for (ULONG q = 0; q < cfg->n_qd && rc == RETURN_OK; q++)
                    rc = db_fs_point(cfg, k, cfg->bs[i], cfg->qd[q]);
            }
        }
    }
    if ((cfg->tests & DB_T_FSIOPS) != 0)
    {
        for (ULONG k = 2; k <= 3 && rc == RETURN_OK; k++) /* rndrd, then rndwr */
        {
            for (ULONG q = 0; q < cfg->n_qd && rc == RETURN_OK; q++)
                rc = db_fs_point(cfg, k, DB_IOPS_BS, cfg->qd[q]);
        }
    }

    db_fs_close();
out:
    if (!cfg->keepfile)
        DeleteFile((CONST_STRPTR)path);
    return rc;
}
