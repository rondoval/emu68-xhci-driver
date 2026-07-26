/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * diskbench — USB mass-storage benchmark for xhci.device/UAS tuning.
 *
 * Hybrid: fs engine (a file in the volume root, app-visible numbers, QD as
 * outstanding DOS packets) + read-only dev engine (async NSCMD_TD_READ64 at
 * the underlying exec device, QD as outstanding IORequests). See diskbench.h
 * for the why.
 *
 * The wire queue depth is capped by the massstorage 'UAS queue depth'
 * slider, which is latched at bind — change it in Trident and replug the
 * drive between sweep configurations; QD= here only sets how many requests
 * this tool keeps outstanding.
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include <exec/types.h>
#include <dos/dos.h>
#include <dos/rdargs.h>

#include <proto/exec.h>
#include <proto/dos.h>

#include "diskbench.h"

#ifdef VERSTAG
static const char verstag[] __attribute__((used)) = VERSTAG;
#endif

#define TEMPLATE "DRIVE,DEVICE/K,UNIT/N,TEST/K,BS/K,QD/K,SIZE/K,SECS/N,SEED/N," \
                 "MISALIGN/S,CHIP/S,KEEPFILE/S,RAW/S,HELP/S"

enum
{
    ARG_DRIVE,
    ARG_DEVICE,
    ARG_UNIT,
    ARG_TEST,
    ARG_BS,
    ARG_QD,
    ARG_SIZE,
    ARG_SECS,
    ARG_SEED,
    ARG_MISALIGN,
    ARG_CHIP,
    ARG_KEEPFILE,
    ARG_RAW,
    ARG_HELP,
    ARG_COUNT
};

static const ULONG def_bs[] = {4096, 16384, 65536, 262144, 524288,
                               1048576, 2097152, 4194304, 8388608};
static const ULONG def_qd[] = {1, 2, 4, 8, 16};

static BOOL db_strieq(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0')
    {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b))
            return FALSE;
        a++;
        b++;
    }
    return (BOOL)(*a == *b);
}

static BOOL db_parse_test(const char *s, ULONG *tests)
{
    if (db_strieq(s, "FSSEQ"))
        *tests = DB_T_FSSEQ;
    else if (db_strieq(s, "FSIOPS"))
        *tests = DB_T_FSIOPS;
    else if (db_strieq(s, "DEVSEQ"))
        *tests = DB_T_DEVSEQ;
    else if (db_strieq(s, "DEVIOPS"))
        *tests = DB_T_DEVIOPS;
    else if (db_strieq(s, "FS"))
        *tests = DB_T_FS;
    else if (db_strieq(s, "DEV"))
        *tests = DB_T_DEV;
    else if (db_strieq(s, "ALL"))
        *tests = DB_T_ALL;
    else
        return FALSE;
    return TRUE;
}

/* plain ASCII: the Amiga shell is ISO-8859-1, not UTF-8 */
static void db_usage(void)
{
    printf(
        "usage: diskbench <drive:> [DEVICE=<name>] [UNIT=<n>] [TEST=<what>]\n"
        "                 [BS=<list>] [QD=<list>] [SIZE=<n>] [SECS=<n>] [SEED=<n>]\n"
        "                 [MISALIGN] [CHIP] [KEEPFILE] [RAW] [HELP]\n"
        "\n"
        "Two engines, both sweeping block size and queue depth:\n"
        "  fs   read/write packets on <drive:>diskbench.tmp - the numbers an\n"
        "       application sees, filesystem cache and all. QD is how many packets\n"
        "       stay outstanding (one file handle each); how much of that reaches the\n"
        "       wire is up to the handler. A single-threaded one (FFS) still does them\n"
        "       one at a time, but stops idling between them; PFS and SFS can overlap\n"
        "       more. Writes, inside that one file.\n"
        "  dev  NSCMD_TD_READ64 straight at the exec device under the volume, QD\n"
        "       requests in flight - no filesystem in the way, so QD lands on the\n"
        "       massstorage UAS tag engine. Never writes.\n"
        "\n");
    printf(
        "Target\n"
        "  <drive:>  Volume or device root to test, e.g. USB0:. The fs engine needs\n"
        "            it; for the dev engine it also resolves the exec device, unit and\n"
        "            partition range, so dev I/O stays inside that partition.\n"
        "  DEVICE=, UNIT=\n"
        "            Exec device and unit, e.g. usbscsi.device 0. dev engine only.\n"
        "            Without a <drive:> the whole disk is the range: still read-only,\n"
        "            but it reads outside any partition.\n"
        "  TEST=     FSSEQ    fs sequential read, then write, every BS x QD\n"
        "            FSIOPS   fs random read, then write, at 4K, per QD entry\n"
        "            DEVSEQ   dev sequential read, every BS x QD\n"
        "            DEVIOPS  dev random read at 4K, per QD entry\n"
        "            FS | DEV | ALL select the matching groups.\n"
        "            Default: ALL with a <drive:>, DEV with only DEVICE=.\n"
        "\n"
        "Both engines\n"
        "  BS=       Block sizes to sweep, comma separated, K/M suffixes, max 16.\n"
        "            Default 4K,16K,64K,256K,512K,1M,2M,4M,8M. The IOPS tests ignore\n"
        "            it and always use 4K. Blocks past the device maxtrans (2M by\n"
        "            default) split into chunks that serialize on one tag - the dev\n"
        "            report flags those points.\n"
        "  QD=       Queue depths to sweep, max 16 entries, each clamped to 16\n"
        "            (massstorage NCM_MAXTAGS). Default 1,2,4,8,16. This is only what\n"
        "            diskbench keeps outstanding; the wire depth is capped by the\n"
        "            massstorage 'UAS queue depth' slider, which latches at bind, so\n"
        "            change it in Trident and replug the drive between sweeps.\n"
        "  SIZE=     fs test file size, and the dev engine's random-read working set.\n"
        "            1M..1G, default 256M. Pick something past the drive's own cache.\n"
        "  SECS=     Measured seconds per point, 1..600, default 3. Each point also\n"
        "            runs an unrecorded warm-up of SECS/4 (at least half a second).\n"
        "  SEED=     Seed for the random offsets, so a rerun replays them. Default\n"
        "            0x1234567.\n"
        "  MISALIGN  Offset every buffer by one byte, taking the driver's bounce-\n"
        "            buffer path instead of zero-copy DMA.\n"
        "  CHIP      Allocate buffers in Chip RAM. PiStorm cannot DMA there, so this\n"
        "            measures the bounce path plus Chip bus contention.\n"
        "  RAW       Print one machine-parseable 'RESULT key=value ...' line per point\n"
        "            instead of the human-readable line.\n"
        "  HELP      This text.\n");
    printf(
        "\n"
        "fs engine only\n"
        "  KEEPFILE  Leave <drive:>diskbench.tmp behind; a later run with the same\n"
        "            SIZE reuses it and skips the create pass.\n"
        "\n"
        "Report line:\n"
        "  fs seqrd bs=64K qd=1: 31.5 MB/s  504 IOPS  lat avg/min/max 1980/1710/9100 us\n"
        "  Rates cover the measured window only; latency is per request as diskbench\n"
        "  sees it, so at QD>1 it includes queueing. Ctrl-C stops the run (the fs\n"
        "  engine first drains: DOS packets cannot be aborted).\n"
        "\n"
        "A default sweep is a few hundred points - trim it with TEST=, BS= and QD=.\n"
        "\n"
        "Examples:\n"
        "  diskbench USB0:\n"
        "  diskbench USB0: TEST=DEVSEQ BS=64K,1M QD=1,4,16 SECS=10\n"
        "  diskbench USB0: TEST=FS BS=1M QD=1,4 SIZE=512M KEEPFILE\n"
        "  diskbench DEVICE=usbscsi.device UNIT=0 TEST=DEV RAW\n");
}

int main(void)
{
    struct BenchCfg cfg;
    LONG args[ARG_COUNT];
    int rc = RETURN_ERROR;

    memset(&cfg, 0, sizeof(cfg));
    memset(args, 0, sizeof(args));

    struct RDArgs *rda = ReadArgs((CONST_STRPTR)TEMPLATE, args, NULL);
    if (rda == NULL)
    {
        PrintFault(IoErr(), (CONST_STRPTR)"diskbench");
        printf("usage: diskbench <drive:> [options] - 'diskbench HELP' for the full list\n");
        return RETURN_ERROR;
    }

    /* neither engine can do anything without a target: help is the answer */
    if (args[ARG_HELP] != 0 || (args[ARG_DRIVE] == 0 && args[ARG_DEVICE] == 0))
    {
        db_usage();
        rc = (args[ARG_HELP] != 0) ? RETURN_OK : RETURN_ERROR;
        goto out_args;
    }

    /* defaults */
    memcpy(cfg.bs, def_bs, sizeof(def_bs));
    cfg.n_bs = sizeof(def_bs) / sizeof(def_bs[0]);
    memcpy(cfg.qd, def_qd, sizeof(def_qd));
    cfg.n_qd = sizeof(def_qd) / sizeof(def_qd[0]);
    cfg.size = 256UL * 1024 * 1024;
    cfg.secs = 3;
    cfg.seed = 0x1234567;

    /* DRIVE: volume root, normalized to a trailing ':' */
    if (args[ARG_DRIVE] != 0)
    {
        const char *d = (const char *)args[ARG_DRIVE];
        size_t len = strlen(d);
        const char *colon = strchr(d, ':');
        if (len == 0 || len >= DB_NAME_LEN - 1 || strchr(d, '/') != NULL ||
            (colon != NULL && colon != d + len - 1))
        {
            printf("diskbench: DRIVE must be a volume or device root like USB0:\n");
            goto out_args;
        }
        strcpy(cfg.drive, d);
        if (colon == NULL)
            strcat(cfg.drive, ":");
    }

    if (args[ARG_DEVICE] != 0)
    {
        const char *dev = (const char *)args[ARG_DEVICE];
        if (strlen(dev) >= DB_NAME_LEN)
        {
            printf("diskbench: DEVICE name too long\n");
            goto out_args;
        }
        strcpy(cfg.device, dev);
        cfg.have_device = TRUE;
        cfg.whole_disk = TRUE; /* no DosEnvec range unless DRIVE resolves one */
    }
    if (args[ARG_UNIT] != 0)
        cfg.unit = (ULONG)*(LONG *)args[ARG_UNIT];

    cfg.tests = (cfg.drive[0] != '\0') ? DB_T_ALL : DB_T_DEV;
    if (args[ARG_TEST] != 0 && !db_parse_test((const char *)args[ARG_TEST], &cfg.tests))
    {
        printf("diskbench: bad TEST (FSSEQ|FSIOPS|DEVSEQ|DEVIOPS|FS|DEV|ALL)\n");
        goto out_args;
    }

    if (args[ARG_BS] != 0 &&
        !db_parse_list((const char *)args[ARG_BS], cfg.bs, DB_MAX_LIST, &cfg.n_bs))
    {
        printf("diskbench: bad BS list (e.g. BS=4K,64K,1M; max %d entries)\n", DB_MAX_LIST);
        goto out_args;
    }
    if (args[ARG_QD] != 0 &&
        !db_parse_list((const char *)args[ARG_QD], cfg.qd, DB_MAX_LIST, &cfg.n_qd))
    {
        printf("diskbench: bad QD list (e.g. QD=1,2,4,8)\n");
        goto out_args;
    }
    for (ULONG i = 0; i < cfg.n_qd; i++)
    {
        if (cfg.qd[i] > DB_MAX_QD)
        {
            printf("diskbench: QD %lu clamped to %d (massstorage NCM_MAXTAGS)\n",
                   (unsigned long)cfg.qd[i], DB_MAX_QD);
            cfg.qd[i] = DB_MAX_QD;
        }
    }

    if (args[ARG_SIZE] != 0 && !db_parse_size((const char *)args[ARG_SIZE], &cfg.size))
    {
        printf("diskbench: bad SIZE (e.g. SIZE=256M)\n");
        goto out_args;
    }
    if (cfg.size < 1048576ULL || cfg.size > 0x40000000ULL)
    {
        /* Seek() is a signed LONG and FAT32 caps files at 4G: keep 1M..1G */
        printf("diskbench: SIZE must be between 1M and 1G\n");
        goto out_args;
    }
    for (ULONG i = 0; i < cfg.n_bs; i++)
    {
        if ((db_u64)cfg.bs[i] > cfg.size)
        {
            printf("diskbench: BS %lu exceeds SIZE\n", (unsigned long)cfg.bs[i]);
            goto out_args;
        }
    }

    if (args[ARG_SECS] != 0)
    {
        LONG s = *(LONG *)args[ARG_SECS];
        if (s < 1 || s > 600)
        {
            printf("diskbench: SECS must be 1..600\n");
            goto out_args;
        }
        cfg.secs = (ULONG)s;
    }
    if (args[ARG_SEED] != 0)
        cfg.seed = (ULONG)*(LONG *)args[ARG_SEED];
    cfg.misalign = (BOOL)(args[ARG_MISALIGN] != 0);
    cfg.chip = (BOOL)(args[ARG_CHIP] != 0);
    cfg.keepfile = (BOOL)(args[ARG_KEEPFILE] != 0);
    db_raw = (BOOL)(args[ARG_RAW] != 0);

    if ((cfg.tests & DB_T_FS) != 0 && cfg.drive[0] == '\0')
    {
        printf("diskbench: filesystem tests need a DRIVE\n");
        goto out_args;
    }

    if (!db_timer_open())
    {
        printf("diskbench: no timer.device\n");
        goto out_args;
    }

#ifdef VERSTAG
    printf("%s\n", verstag + 6); /* skip "$VER: " */
#endif
    {
        char sizestr[12];
        db_fmt_size((ULONG)cfg.size, sizestr);
        printf("  buffers: %s%s  size=%s secs=%lu seed=0x%lx  eclock=%lu Hz\n",
               cfg.chip ? "chip" : "fast",
               cfg.misalign ? " misaligned(+1)" : " 4K-aligned",
               sizestr, (unsigned long)cfg.secs, (unsigned long)cfg.seed,
               (unsigned long)db_efreq);
        if ((cfg.tests & DB_T_DEV) != 0)
            printf("  note: wire QD is capped by the 'UAS queue depth' slider (latched at bind)\n");
    }

    rc = RETURN_OK;
    if ((cfg.tests & DB_T_FS) != 0)
    {
        LONG r = db_run_fs(&cfg);
        if (r > rc)
            rc = (int)r;
    }
    if ((cfg.tests & DB_T_DEV) != 0 && rc < RETURN_WARN)
    {
        LONG r = db_run_dev(&cfg);
        if (r > rc)
            rc = (int)r;
    }

    db_timer_close();
out_args:
    FreeArgs(rda);
    return rc;
}
