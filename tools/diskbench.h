/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * diskbench — USB mass-storage benchmark for xhci.device/UAS tuning.
 *
 * Two engines share this header, both sweeping block size and queue depth:
 *  - fs  (diskbench_fs.c):  ACTION_READ/WRITE packets on a file in the volume
 *    root — the application-visible numbers, filesystem included. QD is how
 *    many packets stay outstanding (one open file handle each); how much of
 *    that reaches the wire is up to the handler.
 *  - dev (diskbench_dev.c): read-only async NSCMD_TD_READ64 directly at the
 *    underlying exec device, N requests in flight — no filesystem in the way,
 *    so QD lands on the massstorage UAS tag engine (one IORequest = one tag).
 */

#ifndef DISKBENCH_H
#define DISKBENCH_H

#include <exec/types.h>
#include <dos/dos.h>

typedef unsigned long long db_u64; /* NDK 3.2 has no UQUAD */

#define DB_MAX_LIST 16       /* max entries in the BS= / QD= sweeps */
#define DB_MAX_QD 16         /* massstorage NCM_MAXTAGS */
#define DB_IOPS_BS 4096      /* classic random-I/O block size */
#define DB_ALIGN 4096        /* buffer alignment for the zero-copy DMA path */
#define DB_NAME_LEN 64
#define DB_ERR_BUDGET 8      /* device I/O errors tolerated per point */

/* cfg_tests bitmask */
#define DB_T_FSSEQ (1UL << 0)
#define DB_T_FSIOPS (1UL << 1)
#define DB_T_DEVSEQ (1UL << 2)
#define DB_T_DEVIOPS (1UL << 3)
#define DB_T_FS (DB_T_FSSEQ | DB_T_FSIOPS)
#define DB_T_DEV (DB_T_DEVSEQ | DB_T_DEVIOPS)
#define DB_T_ALL (DB_T_FS | DB_T_DEV)

struct BenchCfg
{
    char drive[DB_NAME_LEN];    /* volume root incl. ':' ("USB0:"), or "" */
    char device[DB_NAME_LEN];   /* exec device ("usbscsi.device") */
    ULONG unit;
    BOOL have_device;           /* device/unit filled in (arg or resolve) */
    BOOL whole_disk;            /* no DosEnvec range: use TD_GETGEOMETRY */
    db_u64 part_start;          /* device-engine byte range on the disk */
    db_u64 part_end;
    ULONG bs[DB_MAX_LIST];
    ULONG n_bs;
    ULONG qd[DB_MAX_LIST];
    ULONG n_qd;
    db_u64 size;                /* fs file size / dev random working set */
    ULONG secs;                 /* time budget per point */
    ULONG seed;
    ULONG tests;                /* DB_T_* mask */
    BOOL misalign;              /* +1 byte buffer offset: bounce-buffer path */
    BOOL chip;                  /* MEMF_CHIP buffers: bounce-buffer path */
    BOOL keepfile;
};

struct BenchStat
{
    db_u64 bytes;
    db_u64 ops;
    db_u64 errors;
    db_u64 lat_sum; /* EClock ticks; min/max/sum only, no per-op storage */
    db_u64 lat_min;
    db_u64 lat_max;
    db_u64 ticks;   /* measured elapsed EClock ticks */
};

struct BenchPoint
{
    const char *engine; /* "fs" | "dev" */
    const char *test;   /* "seqrd" | "seqwr" | "rndrd" | "rndwr" */
    ULONG bs;
    ULONG qd;
    const char *note;   /* extra annotation or NULL */
};

/* diskbench_util.c */
extern ULONG db_efreq; /* EClock ticks per second (valid after db_timer_open) */
extern BOOL db_raw;    /* RAW: one machine-parseable line per point instead */

BOOL db_timer_open(void);
void db_timer_close(void);
db_u64 db_now(void);
void db_u64_dec(db_u64 v, char *buf); /* buf: at least 21 chars */
void db_stat_reset(struct BenchStat *st);
void db_report(const struct BenchPoint *pt, const struct BenchStat *st);
ULONG db_lcg_next(ULONG *state);
ULONG db_lcg_range(ULONG *state, ULONG n); /* [0, n), high-bits reduction */
APTR db_alloc_aligned(ULONG size, ULONG memf, APTR *base, ULONG *basesize);
BOOL db_parse_size(const char *s, db_u64 *out);        /* K/M/G suffixes */
BOOL db_parse_list(const char *s, ULONG *out, ULONG max, ULONG *count);
void db_fmt_size(ULONG v, char *buf); /* "64K", "2M", "512" (buf >= 12) */

static inline void db_stat_lat(struct BenchStat *st, db_u64 lat)
{
    st->lat_sum += lat;
    if (lat < st->lat_min)
        st->lat_min = lat;
    if (lat > st->lat_max)
        st->lat_max = lat;
}

/* diskbench_fs.c — returns 0 on success, DOS RC on failure */
LONG db_run_fs(struct BenchCfg *cfg);

/* diskbench_dev.c */
LONG db_resolve_target(struct BenchCfg *cfg); /* DosList walk: DRIVE -> device/unit/range */
LONG db_run_dev(struct BenchCfg *cfg);

#endif /* DISKBENCH_H */
