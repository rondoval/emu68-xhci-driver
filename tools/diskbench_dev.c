/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * diskbench_dev.c — device engine: read-only async NSCMD_TD_READ64 directly
 * at the exec device under the volume (usbscsi.device for UAS drives).
 *
 * One IORequest = one UAS tag in massstorage, and a single request larger
 * than maxtrans chunks serially on that one tag — so queue depth on the
 * wire exists only while N requests are outstanding at the unit at once.
 * This engine keeps exactly QD requests in flight with completion-driven
 * resubmission. Never writes.
 *
 * Offsets and lengths stay multiples of the sector size: misaligned
 * requests are not tag-eligible in massstorage and would silently degrade
 * into serialized barrier commands.
 */

#include <stdio.h>
#include <string.h>

#include <exec/types.h>
#include <exec/memory.h>
#include <exec/io.h>
#include <devices/trackdisk.h>
#include <devices/newstyle.h>
#include <dos/dos.h>
#include <dos/dosextens.h>
#include <dos/filehandler.h>

#include <proto/exec.h>
#include <proto/dos.h>

#include "diskbench.h"

struct DevSlot
{
    struct IOExtTD ds_Req; /* first member: GetMsg() casts back to the slot */
    db_u64 ds_Submit;
    UBYTE *ds_Buf;
    BOOL ds_Busy;
};

static struct MsgPort *dev_port;
static struct IOExtTD *dev_anchor; /* the OpenDevice()d request; DoIO only */
static struct DevSlot dev_slots[DB_MAX_QD];
static UWORD dev_cmd; /* NSCMD_TD_READ64 or TD_READ64 */
static ULONG dev_secsize;

/*
 * DRIVE -> exec device/unit/partition range, via the DosList. GetDeviceProc's
 * dvp_DevNode is "DON'T TOUCH OR USE!" per the NDK, so we walk the list:
 * device entry by name, else volume entry matched back to its device entry
 * through dol_Task. Non-disk handlers keep junk in dol_Startup, hence the
 * sanity gates before dereferencing anything.
 */

static BOOL db_bstr_copy(BSTR b, char *dst, ULONG dstlen)
{
    const UBYTE *s = BADDR(b);
    if (s == NULL || TypeOfMem((APTR)s) == 0)
        return FALSE;
    ULONG len = s[0];
    if (len == 0 || len >= dstlen)
        return FALSE;
    memcpy(dst, s + 1, len);
    dst[len] = '\0';
    return TRUE;
}

LONG db_resolve_target(struct BenchCfg *cfg)
{
    char name[DB_NAME_LEN];
    strcpy(name, cfg->drive);
    name[strlen(name) - 1] = '\0'; /* strip the ':' */

    LONG rc = RETURN_ERROR;
    struct DosList *dl = LockDosList(LDF_DEVICES | LDF_VOLUMES | LDF_READ);

    struct DosList *entry = FindDosEntry(dl, (CONST_STRPTR)name, LDF_DEVICES);
    if (entry == NULL)
    {
        struct DosList *vol = FindDosEntry(dl, (CONST_STRPTR)name, LDF_VOLUMES);
        if (vol != NULL && vol->dol_Task != NULL)
        {
            struct DosList *d = dl;
            while ((d = NextDosEntry(d, LDF_DEVICES)) != NULL)
            {
                if (d->dol_Task == vol->dol_Task)
                {
                    entry = d;
                    break;
                }
            }
        }
    }

    if (entry == NULL)
    {
        printf("diskbench: cannot find a mounted DOS device for %s\n", cfg->drive);
        goto out;
    }

    ULONG su = entry->dol_misc.dol_handler.dol_Startup;
    struct FileSysStartupMsg *fssm = BADDR((BPTR)su);
    if (su == 0 || TypeOfMem(fssm) == 0)
        goto bad_fssm;

    char devname[DB_NAME_LEN];
    if (!db_bstr_copy(fssm->fssm_Device, devname, sizeof(devname)))
        goto bad_fssm;

    struct DosEnvec *de = BADDR(fssm->fssm_Environ);
    if (de == NULL || TypeOfMem(de) == 0 ||
        de->de_TableSize < 11 || de->de_TableSize > 25 ||
        de->de_SizeBlock < 64 || de->de_SizeBlock > 4096 ||
        de->de_Surfaces == 0 || de->de_Surfaces > 16384 ||
        de->de_BlocksPerTrack == 0 || de->de_BlocksPerTrack > 16384)
        goto bad_fssm;

    if (de->de_SectorPerBlock > 1)
        printf("diskbench: warning: de_SectorPerBlock=%lu, range may be off\n",
               (unsigned long)de->de_SectorPerBlock);

    if (!cfg->have_device)
    {
        strcpy(cfg->device, devname);
        cfg->unit = fssm->fssm_Unit;
        cfg->have_device = TRUE;
    }
    else if (strcmp(cfg->device, devname) != 0 || cfg->unit != fssm->fssm_Unit)
    {
        /* DEVICE=/UNIT= override a resolved drive: the DosEnvec range
         * belongs to another device — fall back to whole-disk */
        cfg->whole_disk = TRUE;
        rc = RETURN_OK;
        goto out;
    }

    db_u64 bpc = (db_u64)de->de_SizeBlock * 4 * de->de_Surfaces * de->de_BlocksPerTrack;
    cfg->part_start = de->de_LowCyl * bpc;
    cfg->part_end = ((db_u64)de->de_HighCyl + 1) * bpc;
    cfg->whole_disk = FALSE;
    rc = RETURN_OK;
    goto out;

bad_fssm:
    if (cfg->have_device)
    {
        cfg->whole_disk = TRUE; /* DEVICE= given: geometry will set the range */
        rc = RETURN_OK;
    }
    else
    {
        printf("diskbench: %s has no usable FileSysStartupMsg; use DEVICE=/UNIT=\n",
               cfg->drive);
    }
out:
    UnLockDosList(LDF_DEVICES | LDF_VOLUMES | LDF_READ);
    return rc;
}

static void db_dev_submit(struct DevSlot *s, ULONG bs, db_u64 off)
{
    struct IOStdReq *io = &s->ds_Req.iotd_Req;
    io->io_Command = dev_cmd;
    io->io_Data = s->ds_Buf;
    io->io_Length = bs;
    io->io_Offset = (ULONG)off;
    /* io_HighOffset IS io_Actual, and the device overwrites io_Actual with
     * the byte count on completion — it must be re-set on EVERY submit */
    io->io_Actual = (ULONG)(off >> 32);
    s->ds_Submit = db_now();
    s->ds_Busy = TRUE;
    SendIO((struct IORequest *)io);
}

static void db_dev_abort_all(void)
{
    for (ULONG i = 0; i < DB_MAX_QD; i++)
    {
        if (dev_slots[i].ds_Busy)
            AbortIO((struct IORequest *)&dev_slots[i].ds_Req);
    }
    for (ULONG i = 0; i < DB_MAX_QD; i++)
    {
        if (dev_slots[i].ds_Busy)
        {
            WaitIO((struct IORequest *)&dev_slots[i].ds_Req);
            dev_slots[i].ds_Busy = FALSE;
        }
    }
}

/* one measured point at a fixed (bs, qd); rnd selects random offsets */
static LONG db_dev_point(const struct BenchCfg *cfg, ULONG bs, ULONG qd, BOOL rnd)
{
    struct BenchStat st;
    db_stat_reset(&st);

    db_u64 span = cfg->part_end - cfg->part_start;
    db_u64 workset = rnd ? ((cfg->size < span) ? cfg->size : span) : span;
    ULONG nblk = (ULONG)(workset / bs);
    if (nblk < qd)
    {
        printf("dev: skip bs=%lu qd=%lu: range too small\n",
               (unsigned long)bs, (unsigned long)qd);
        return RETURN_OK;
    }
    if ((db_u64)qd * bs > 0x20000000ULL) /* keep the buffer set inside a ULONG */
    {
        printf("dev: skip bs=%lu qd=%lu: buffer set above 512M\n",
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
        printf("dev: skip bs=%lu qd=%lu: no memory for %lu byte buffer\n",
               (unsigned long)bs, (unsigned long)qd, (unsigned long)(qd * bs));
        return RETURN_OK;
    }
    if (cfg->misalign)
        buf++;
    memset(buf, 0xA5, qd * bs);

    ULONG rng = cfg->seed;
    db_u64 cursor = cfg->part_start;
    db_u64 warm_ticks = (db_u64)cfg->secs * db_efreq / 4;
    if (warm_ticks < db_efreq / 2)
        warm_ticks = db_efreq / 2;
    db_u64 warm_end = db_now() + warm_ticks;
    db_u64 start = 0;
    db_u64 deadline = ~0ULL;
    db_u64 last = 0;
    db_u64 errs = 0;
    BYTE first_err = 0;
    BOOL recording = FALSE;
    LONG rc = RETURN_OK;
    ULONG inflight = 0;

    for (ULONG i = 0; i < qd; i++)
    {
        struct DevSlot *s = &dev_slots[i];
        s->ds_Buf = buf + i * bs;
        db_u64 off;
        if (rnd)
            off = cfg->part_start + (db_u64)db_lcg_range(&rng, nblk) * bs;
        else
        {
            off = cursor;
            cursor += bs;
            if (cursor + bs > cfg->part_end)
                cursor = cfg->part_start;
        }
        db_dev_submit(s, bs, off);
        inflight++;
    }

    while (inflight > 0)
    {
        ULONG sig = Wait((1UL << dev_port->mp_SigBit) | SIGBREAKF_CTRL_C);
        if ((sig & SIGBREAKF_CTRL_C) != 0)
        {
            printf("dev: ^C\n");
            db_dev_abort_all();
            rc = RETURN_WARN;
            goto out;
        }

        struct DevSlot *s;
        while ((s = (struct DevSlot *)GetMsg(dev_port)) != NULL)
        {
            s->ds_Busy = FALSE;
            db_u64 now = db_now();
            struct IOStdReq *io = &s->ds_Req.iotd_Req;

            if (io->io_Error != 0)
            {
                if (errs == 0)
                    first_err = io->io_Error;
                errs++;
            }
            if (recording)
            {
                if (io->io_Error == 0)
                {
                    st.bytes += io->io_Actual;
                    st.ops++;
                }
                db_stat_lat(&st, now - s->ds_Submit);
                last = now;
            }
            else if (now >= warm_end)
            {
                db_stat_reset(&st);
                start = now;
                deadline = now + (db_u64)cfg->secs * db_efreq;
                recording = TRUE;
                last = now;
            }

            if (now < deadline && errs <= DB_ERR_BUDGET)
            {
                db_u64 off;
                if (rnd)
                    off = cfg->part_start + (db_u64)db_lcg_range(&rng, nblk) * bs;
                else
                {
                    off = cursor;
                    cursor += bs;
                    if (cursor + bs > cfg->part_end)
                        cursor = cfg->part_start;
                }
                db_dev_submit(s, bs, off);
            }
            else
            {
                inflight--;
            }
        }
    }

    st.ticks = (last > start) ? last - start : 0;
    st.errors = errs;
    if (errs > DB_ERR_BUDGET)
        printf("dev: bs=%lu qd=%lu aborted, io_Error %ld (%lu errors)\n",
               (unsigned long)bs, (unsigned long)qd, (long)first_err,
               (unsigned long)errs);

    const char *note = NULL;
    if (!rnd && bs > 2097152)
        note = "(bs>maxtrans default 2M: chunks serialize on one tag)";
    struct BenchPoint pt = {"dev", rnd ? "rndrd" : "seqrd", bs, qd, note};
    db_report(&pt, &st);

out:
    FreeMem(base, basesize);
    return rc;
}

static LONG db_dev_open(const struct BenchCfg *cfg)
{
    dev_port = CreateMsgPort();
    dev_anchor = (struct IOExtTD *)CreateIORequest(dev_port, sizeof(struct IOExtTD));
    if (dev_anchor == NULL)
    {
        if (dev_port != NULL)
            DeleteMsgPort(dev_port);
        dev_port = NULL;
        return RETURN_ERROR;
    }
    if (OpenDevice((CONST_STRPTR)cfg->device, cfg->unit,
                   (struct IORequest *)dev_anchor, 0) != 0)
    {
        printf("diskbench: cannot open %s unit %lu\n",
               cfg->device, (unsigned long)cfg->unit);
        DeleteIORequest((struct IORequest *)dev_anchor);
        DeleteMsgPort(dev_port);
        dev_anchor = NULL;
        dev_port = NULL;
        return RETURN_ERROR;
    }
    return RETURN_OK;
}

static void db_dev_close(void)
{
    if (dev_anchor != NULL)
    {
        CloseDevice((struct IORequest *)dev_anchor);
        DeleteIORequest((struct IORequest *)dev_anchor);
        DeleteMsgPort(dev_port);
        dev_anchor = NULL;
        dev_port = NULL;
    }
}

LONG db_run_dev(struct BenchCfg *cfg)
{
    if (cfg->drive[0] != '\0')
    {
        LONG r = db_resolve_target(cfg);
        if (r != RETURN_OK)
            return r;
    }

    if (db_dev_open(cfg) != RETURN_OK)
        return RETURN_ERROR;

    LONG rc = RETURN_ERROR;
    struct IOStdReq *aio = &dev_anchor->iotd_Req;

    /* one-time queries — these are tag-drain barriers in massstorage, so
     * they must never sit inside a timed loop */
    struct NSDeviceQueryResult nsdqr;
    memset(&nsdqr, 0, sizeof(nsdqr));
    aio->io_Command = NSCMD_DEVICEQUERY;
    aio->io_Data = &nsdqr;
    aio->io_Length = sizeof(nsdqr);
    dev_cmd = TD_READ64;
    if (DoIO((struct IORequest *)aio) == 0 && nsdqr.nsdqr_SupportedCommands != NULL)
    {
        if (nsdqr.nsdqr_DeviceType != NSDEVTYPE_TRACKDISK)
            printf("diskbench: warning: %s is not a trackdisk-type device\n", cfg->device);
        const UWORD *c = nsdqr.nsdqr_SupportedCommands;
        while (*c != 0)
        {
            if (*c == NSCMD_TD_READ64)
            {
                dev_cmd = NSCMD_TD_READ64;
                break;
            }
            c++;
        }
    }
    if (dev_cmd == TD_READ64)
        printf("dev: no NSCMD_TD_READ64, using TD_READ64\n");

    struct DriveGeometry dg;
    memset(&dg, 0, sizeof(dg));
    aio->io_Command = TD_GETGEOMETRY;
    aio->io_Data = &dg;
    aio->io_Length = sizeof(dg);
    if (DoIO((struct IORequest *)aio) == 0 && dg.dg_SectorSize != 0)
    {
        dev_secsize = dg.dg_SectorSize;
    }
    else
    {
        dev_secsize = 512;
        printf("dev: TD_GETGEOMETRY failed, assuming 512 byte sectors\n");
    }

    db_u64 disk_end = (db_u64)dg.dg_TotalSectors * dg.dg_SectorSize;
    if (cfg->whole_disk)
    {
        if (disk_end == 0)
        {
            printf("diskbench: no partition range and no geometry — cannot size the disk\n");
            goto out;
        }
        cfg->part_start = 0;
        cfg->part_end = disk_end;
    }
    else if (disk_end != 0 && cfg->part_end > disk_end)
    {
        printf("dev: warning: DosEnvec range exceeds disk, clamping\n");
        cfg->part_end = disk_end;
    }
    if (cfg->part_end <= cfg->part_start)
    {
        printf("diskbench: empty device range\n");
        goto out;
    }

    {
        char lo[21], hi[21];
        db_u64_dec(cfg->part_start, lo);
        db_u64_dec(cfg->part_end, hi);
        printf("dev: %s unit %lu, bytes %s..%s (%lu MB), sector %lu, %s\n",
               cfg->device, (unsigned long)cfg->unit, lo, hi,
               (unsigned long)((cfg->part_end - cfg->part_start) >> 20),
               (unsigned long)dev_secsize,
               (dev_cmd == NSCMD_TD_READ64) ? "NSCMD_TD_READ64" : "TD_READ64");
    }

    /* the slots inherit io_Device/io_Unit from the opened anchor */
    for (ULONG i = 0; i < DB_MAX_QD; i++)
    {
        CopyMem(dev_anchor, &dev_slots[i].ds_Req, sizeof(struct IOExtTD));
        dev_slots[i].ds_Req.iotd_Req.io_Message.mn_ReplyPort = dev_port;
        dev_slots[i].ds_Req.iotd_Req.io_Message.mn_Length = sizeof(struct IOExtTD);
        dev_slots[i].ds_Busy = FALSE;
    }

    rc = RETURN_OK;
    if ((cfg->tests & DB_T_DEVSEQ) != 0)
    {
        for (ULONG i = 0; i < cfg->n_bs && rc == RETURN_OK; i++)
        {
            if ((cfg->bs[i] % dev_secsize) != 0)
            {
                printf("dev: skip bs=%lu: not a multiple of the %lu byte sector\n",
                       (unsigned long)cfg->bs[i], (unsigned long)dev_secsize);
                continue;
            }
            for (ULONG q = 0; q < cfg->n_qd && rc == RETURN_OK; q++)
                rc = db_dev_point(cfg, cfg->bs[i], cfg->qd[q], FALSE);
        }
    }
    if ((cfg->tests & DB_T_DEVIOPS) != 0 && rc == RETURN_OK)
    {
        ULONG bs = DB_IOPS_BS;
        if ((bs % dev_secsize) != 0)
            bs = dev_secsize; /* 4Kn or exotic sectors: use one sector */
        for (ULONG q = 0; q < cfg->n_qd && rc == RETURN_OK; q++)
            rc = db_dev_point(cfg, bs, cfg->qd[q], TRUE);
    }

out:
    db_dev_close();
    return rc;
}
