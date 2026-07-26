/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * diskbench_util.c — EClock timing, integer-only rate reporting, parsers,
 * aligned allocation, LCG. Timing follows the sockbench idiom: timer.device
 * UNIT_MICROHZ opened once, ReadEClock as a 64-bit tick counter. All rate
 * math is integer (libnix printf floats/%llu are not a given).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <exec/types.h>
#include <exec/memory.h>
#include <devices/timer.h>

#include <proto/exec.h>
#define TIMER_BASE_NAME TimerBase
#include <proto/timer.h>

#include "diskbench.h"

struct Device *TimerBase;
ULONG db_efreq;
BOOL db_raw;

static struct MsgPort *db_timer_port;
static struct timerequest *db_timer_req;

BOOL db_timer_open(void)
{
    db_timer_port = CreateMsgPort();
    db_timer_req = (struct timerequest *)CreateIORequest(db_timer_port, sizeof(struct timerequest));
    if (db_timer_req == NULL ||
        OpenDevice((CONST_STRPTR)TIMERNAME, UNIT_MICROHZ, &db_timer_req->tr_node, 0) != 0)
    {
        if (db_timer_req != NULL)
            DeleteIORequest(&db_timer_req->tr_node);
        if (db_timer_port != NULL)
            DeleteMsgPort(db_timer_port);
        db_timer_req = NULL;
        db_timer_port = NULL;
        return FALSE;
    }
    TimerBase = db_timer_req->tr_node.io_Device;

    struct EClockVal ev;
    db_efreq = ReadEClock(&ev);
    return TRUE;
}

void db_timer_close(void)
{
    if (db_timer_req != NULL)
    {
        CloseDevice(&db_timer_req->tr_node);
        DeleteIORequest(&db_timer_req->tr_node);
        DeleteMsgPort(db_timer_port);
        db_timer_req = NULL;
        db_timer_port = NULL;
    }
}

db_u64 db_now(void)
{
    struct EClockVal ev;
    ReadEClock(&ev);
    return ((db_u64)ev.ev_hi << 32) | ev.ev_lo;
}

/* decimal without printf %llu */
void db_u64_dec(db_u64 v, char *buf)
{
    char tmp[21];
    char *p = tmp + 20;
    *p = '\0';
    do
    {
        *--p = (char)('0' + (char)(v % 10));
        v /= 10;
    } while (v != 0);
    strcpy(buf, p);
}

void db_stat_reset(struct BenchStat *st)
{
    memset(st, 0, sizeof(*st));
    st->lat_min = ~0ULL;
}

void db_fmt_size(ULONG v, char *buf)
{
    if (v != 0 && (v & (1024 * 1024 - 1)) == 0)
        sprintf(buf, "%luM", (unsigned long)(v >> 20));
    else if (v != 0 && (v & (1024 - 1)) == 0)
        sprintf(buf, "%luK", (unsigned long)(v >> 10));
    else
        sprintf(buf, "%lu", (unsigned long)v);
}

void db_report(const struct BenchPoint *pt, const struct BenchStat *st)
{
    db_u64 ticks = st->ticks;
    db_u64 mb_x10 = 0;
    db_u64 iops = 0;
    db_u64 avg_us = 0;
    db_u64 min_us = 0;
    db_u64 max_us = 0;

    if (ticks != 0 && db_efreq != 0)
    {
        /* MB/s ×10: bytes*10*freq stays well under 2^63 for any real run */
        mb_x10 = st->bytes * 10ULL * db_efreq / ticks / 1048576ULL;
        iops = st->ops * db_efreq / ticks;
    }
    if (st->ops != 0 && db_efreq != 0)
    {
        avg_us = st->lat_sum * 1000000ULL / db_efreq / st->ops;
        min_us = st->lat_min * 1000000ULL / db_efreq;
        max_us = st->lat_max * 1000000ULL / db_efreq;
    }

    if (db_raw)
    {
        char bytestr[21];
        db_u64_dec(st->bytes, bytestr);
        db_u64 ticks_ms = (db_efreq != 0) ? ticks * 1000ULL / db_efreq : 0;
        printf("RESULT engine=%s test=%s bs=%lu qd=%lu bytes=%s ticks_ms=%lu "
               "mbps_x10=%lu iops=%lu lat_avg_us=%lu lat_min_us=%lu lat_max_us=%lu err=%lu\n",
               pt->engine, pt->test, (unsigned long)pt->bs, (unsigned long)pt->qd,
               bytestr, (unsigned long)ticks_ms,
               (unsigned long)mb_x10, (unsigned long)iops,
               (unsigned long)avg_us, (unsigned long)min_us, (unsigned long)max_us,
               (unsigned long)st->errors);
        return;
    }

    char bstr[12];
    db_fmt_size(pt->bs, bstr);
    printf("%-3s %5s bs=%s qd=%lu: %lu.%lu MB/s  %lu IOPS  lat avg/min/max %lu/%lu/%lu us%s%s%s\n",
           pt->engine, pt->test, bstr, (unsigned long)pt->qd,
           (unsigned long)(mb_x10 / 10), (unsigned long)(mb_x10 % 10),
           (unsigned long)iops,
           (unsigned long)avg_us, (unsigned long)min_us, (unsigned long)max_us,
           (st->errors != 0) ? "  ERRORS" : "",
           (pt->note != NULL) ? "  " : "", (pt->note != NULL) ? pt->note : "");
}

ULONG db_lcg_next(ULONG *state)
{
    *state = *state * 1664525UL + 1013904223UL;
    return *state;
}

/* [0, n) via high-bits multiply — the LCG's low bits have short periods */
ULONG db_lcg_range(ULONG *state, ULONG n)
{
    return (ULONG)(((db_u64)db_lcg_next(state) * n) >> 32);
}

/* AllocMem gives no alignment guarantee: over-allocate and round up */
APTR db_alloc_aligned(ULONG size, ULONG memf, APTR *base, ULONG *basesize)
{
    ULONG total = size + DB_ALIGN;
    APTR mem = AllocMem(total, memf);
    if (mem == NULL)
        return NULL;
    *base = mem;
    *basesize = total;
    return (APTR)(((ULONG)mem + DB_ALIGN - 1) & ~(ULONG)(DB_ALIGN - 1));
}

BOOL db_parse_size(const char *s, db_u64 *out)
{
    char *end;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s)
        return FALSE;
    db_u64 r = v;
    if (*end == 'k' || *end == 'K')
    {
        r <<= 10;
        end++;
    }
    else if (*end == 'm' || *end == 'M')
    {
        r <<= 20;
        end++;
    }
    else if (*end == 'g' || *end == 'G')
    {
        r <<= 30;
        end++;
    }
    if (*end != '\0')
        return FALSE;
    *out = r;
    return TRUE;
}

/* comma-separated K/M-suffixed values, each 1..2G */
BOOL db_parse_list(const char *s, ULONG *out, ULONG max, ULONG *count)
{
    ULONG n = 0;
    char item[24];
    while (*s != '\0')
    {
        const char *comma = strchr(s, ',');
        size_t len = (comma != NULL) ? (size_t)(comma - s) : strlen(s);
        if (len == 0 || len >= sizeof(item) || n >= max)
            return FALSE;
        memcpy(item, s, len);
        item[len] = '\0';

        db_u64 v;
        if (!db_parse_size(item, &v) || v == 0 || v > 0x80000000ULL)
            return FALSE;
        out[n++] = (ULONG)v;

        s += len;
        if (*s == ',')
            s++;
    }
    if (n == 0)
        return FALSE;
    *count = n;
    return TRUE;
}
