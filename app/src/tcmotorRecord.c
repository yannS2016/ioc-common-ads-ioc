/*
 * tcmotorRecord.c
 *
 * Record support for the TwinCAT Motor (tcmotor) record type.
 *
 * Provides motor record field naming convention (.VAL, .RBV, .DMOV etc.)
 * for TwinCAT ADS-based motors with no internal motion logic.
 *
 * Each output field has:
 *   OUT_* — outlink to forward operator writes to the PLC PV
 *   RBK_* — inlink (CP) from the corresponding PLC _RBV record.
 *            Reading the _RBV confirms the PLC acknowledged the write.
 *            If the PLC clamps or rejects the value, the EPICS field
 *            reflects what the PLC actually accepted.
 *
 * Roundtrip for VAL:
 *   caput TST:M1.VAL 150
 *     -> OUT_VAL puts to fPosition
 *     -> PLC acknowledges, updates fPosition_RBV
 *     -> RBK_VAL CP fires, reads fPosition_RBV into VAL
 *     -> TST:M1.VAL now shows PLC-confirmed value
 *
 * Derived from the aSub record implementation pattern (aSubRecord.c).
 * Targets EPICS Base 7 -- get_value/valueDes API was removed in Base 7.
 *
 * Author: PCDS / SLAC
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "alarm.h"
#include "dbDefs.h"
#include "dbEvent.h"
#include "dbAccess.h"
#include "dbFldTypes.h"
#include "errMdef.h"
#include "errlog.h"
#include "recSup.h"
#include "recGbl.h"
#include "cantProceed.h"
#include "epicsExport.h"

#define GEN_SIZE_OFFSET
#include "tcmotorRecord.h"
#undef GEN_SIZE_OFFSET

/* -------------------------------------------------------------------------
 * Forward declarations
 * ------------------------------------------------------------------------- */
static long init_record(struct dbCommon *pcommon, int pass);
static long process(struct dbCommon *pcommon);
static long cvt_dbaddr(DBADDR *paddr);
static long get_array_info(DBADDR *paddr, long *no_elements, long *offset);
static long get_units(DBADDR *paddr, char *units);
static long get_precision(const DBADDR *paddr, long *precision);
static long get_graphic_double(DBADDR *paddr, struct dbr_grDouble *pgd);
static long get_control_double(DBADDR *paddr, struct dbr_ctrlDouble *pcd);
static long get_alarm_double(DBADDR *paddr, struct dbr_alDouble *pad);

/* -------------------------------------------------------------------------
 * Record support entry table (EPICS Base 7)
 * get_value (slot 6) was removed in Base 7 -- use NULL.
 * ------------------------------------------------------------------------- */
struct rset tcmotorRSET = {
    RSETNUMBER,
    NULL,           /* report */
    NULL,           /* initialize */
    init_record,
    process,
    NULL,           /* special */
    NULL,           /* get_value -- removed in Base 7 */
    cvt_dbaddr,
    get_array_info,
    NULL,           /* put_array_info */
    get_units,
    get_precision,
    get_graphic_double,
    get_control_double,
    get_alarm_double
};
epicsExportAddress(rset, tcmotorRSET);

/* -------------------------------------------------------------------------
 * Helper: read one input link into a double, return 0 on success.
 * Leaves *pval unchanged if the link is CONSTANT (not configured).
 * ------------------------------------------------------------------------- */
static long read_input(tcmotorRecord *prec, DBLINK *plink, double *pval)
{
    long status;
    double val = 0.0;

    if (plink->type == CONSTANT)
        return 0;

    status = dbGetLink(plink, DBR_DOUBLE, &val, NULL, NULL);
    if (!status)
        *pval = val;
    else
        recGblSetSevr(prec, LINK_ALARM, INVALID_ALARM);

    return status;
}

/* -------------------------------------------------------------------------
 * Helper: write one output link from a double value, return 0 on success
 * ------------------------------------------------------------------------- */
static long write_output(tcmotorRecord *prec, DBLINK *plink, double val)
{
    long status;

    if (plink->type == CONSTANT)
        return 0;

    status = dbPutLink(plink, DBR_DOUBLE, &val, 1);
    if (status)
        recGblSetSevr(prec, LINK_ALARM, INVALID_ALARM);

    return status;
}

/* -------------------------------------------------------------------------
 * Helper: write one output link from a short value, return 0 on success
 * ------------------------------------------------------------------------- */
static long write_output_short(tcmotorRecord *prec, DBLINK *plink, short val)
{
    long status;

    if (plink->type == CONSTANT)
        return 0;

    status = dbPutLink(plink, DBR_SHORT, &val, 1);
    if (status)
        recGblSetSevr(prec, LINK_ALARM, INVALID_ALARM);

    return status;
}

/* -------------------------------------------------------------------------
 * init_record
 *
 * Pass 0: nothing to do -- links are not yet resolved
 * Pass 1: read all input and readback links for initial values,
 *         seed last-sent output cache to suppress spurious ADS writes
 *         on the first process cycle.
 * ------------------------------------------------------------------------- */
static long init_record(struct dbCommon *pcommon, int pass)
{
    tcmotorRecord *prec = (tcmotorRecord *)pcommon;
    double v;

    if (pass == 0)
        return 0;

    /* Initial read of all pure input fields */
    read_input(prec, &prec->inp_rbv, &prec->rbv);

    v = prec->dmov; read_input(prec, &prec->inp_dmov, &v); prec->dmov = (short)v;
    v = prec->movn; read_input(prec, &prec->inp_movn, &v); prec->movn = (short)v;
    v = prec->hls;  read_input(prec, &prec->inp_hls,  &v); prec->hls  = (short)v;
    v = prec->lls;  read_input(prec, &prec->inp_lls,  &v); prec->lls  = (short)v;
    v = prec->athm; read_input(prec, &prec->inp_athm, &v); prec->athm = (short)v;
    v = prec->tdir; read_input(prec, &prec->inp_tdir, &v); prec->tdir = (short)v;

    /* Initial read of output readbacks (fPosition_RBV, fVelocity_RBV etc.)
     * so VAL, VELO etc. reflect the PLC-confirmed value from the start. */
    read_input(prec, &prec->rbk_val,  &prec->val);
    v = prec->stop; read_input(prec, &prec->rbk_stop, &v); prec->stop = (short)v;
    v = prec->homf; read_input(prec, &prec->rbk_homf, &v); prec->homf = (short)v;
    v = prec->homr; read_input(prec, &prec->rbk_homr, &v); prec->homr = (short)v;
    read_input(prec, &prec->rbk_velo, &prec->velo);
    read_input(prec, &prec->rbk_accl, &prec->accl);
    v = prec->cnen; read_input(prec, &prec->rbk_cnen, &v); prec->cnen = (short)v;

    /* Initialise monitor tracking */
    prec->lval = prec->val;
    prec->lrbv = prec->rbv;

    /* Seed last-sent output cache from the PLC-confirmed readback values.
     * Prevents the first CP-triggered process from forwarding all outputs
     * as "changed" when they are already in sync with the PLC. */
    prec->lovl = prec->val;
    prec->lsto = prec->stop;
    prec->lhof = prec->homf;
    prec->lhor = prec->homr;
    prec->lvel = prec->velo;
    prec->lacc = prec->accl;
    prec->lcne = prec->cnen;

    return 0;
}

/* -------------------------------------------------------------------------
 * process
 *
 * Called when triggered by any CP link update (input or output readback)
 * or a CA write to an output field (pp(TRUE) in dbd).
 *
 * Reads all input and readback links, then conditionally forwards output
 * fields to their linked PLC PVs only when the value has changed.
 * ------------------------------------------------------------------------- */
static long process(struct dbCommon *pcommon)
{
    tcmotorRecord *prec = (tcmotorRecord *)pcommon;
    unsigned short monitor_mask;
    double v;

    prec->pact = TRUE;

    /* ------------------------------------------------------------------
     * Read pure input links (status from PLC)
     * ------------------------------------------------------------------ */
    read_input(prec, &prec->inp_rbv, &prec->rbv);

    v = prec->dmov; read_input(prec, &prec->inp_dmov, &v); prec->dmov = (short)v;
    v = prec->movn; read_input(prec, &prec->inp_movn, &v); prec->movn = (short)v;
    v = prec->hls;  read_input(prec, &prec->inp_hls,  &v); prec->hls  = (short)v;
    v = prec->lls;  read_input(prec, &prec->inp_lls,  &v); prec->lls  = (short)v;
    v = prec->athm; read_input(prec, &prec->inp_athm, &v); prec->athm = (short)v;
    v = prec->tdir; read_input(prec, &prec->inp_tdir, &v); prec->tdir = (short)v;

    /* ------------------------------------------------------------------
     * Forward output fields to PLC PVs only when value has changed.
     * Track whether any output was forwarded this cycle — if so, skip
     * reading the RBK links to avoid immediately overwriting the
     * operator's write with the stale PLC-confirmed value.
     * The CP link on RBK_* will trigger a new process cycle once the
     * PLC updates its _RBV, at which point the confirmed value is read.
     * ------------------------------------------------------------------ */
    int forwarded = 0;

    if (prec->val  != prec->lovl) { write_output      (prec, &prec->out_val,  prec->val);  prec->lovl = prec->val;  forwarded = 1; }
    if (prec->stop != prec->lsto) { write_output_short(prec, &prec->out_stop, prec->stop); prec->lsto = prec->stop; forwarded = 1; }
    if (prec->homf != prec->lhof) { write_output_short(prec, &prec->out_homf, prec->homf); prec->lhof = prec->homf; forwarded = 1; }
    if (prec->homr != prec->lhor) { write_output_short(prec, &prec->out_homr, prec->homr); prec->lhor = prec->homr; forwarded = 1; }
    if (prec->velo != prec->lvel) { write_output      (prec, &prec->out_velo, prec->velo); prec->lvel = prec->velo; forwarded = 1; }
    if (prec->accl != prec->lacc) { write_output      (prec, &prec->out_accl, prec->accl); prec->lacc = prec->accl; forwarded = 1; }
    if (prec->cnen != prec->lcne) { write_output_short(prec, &prec->out_cnen, prec->cnen); prec->lcne = prec->cnen; forwarded = 1; }

    /* ------------------------------------------------------------------
     * Read output readback links (PLC-confirmed values) only when no
     * output was forwarded this cycle. If we forwarded, the RBK CP link
     * will fire again once the PLC updates its _RBV — we wait for that
     * rather than overwriting the operator's write with a stale value.
     * ------------------------------------------------------------------ */
    if (!forwarded) {
        read_input(prec, &prec->rbk_val,  &prec->val);  prec->lovl = prec->val;
        v = prec->stop; read_input(prec, &prec->rbk_stop, &v); prec->stop = (short)v; prec->lsto = prec->stop;
        v = prec->homf; read_input(prec, &prec->rbk_homf, &v); prec->homf = (short)v; prec->lhof = prec->homf;
        v = prec->homr; read_input(prec, &prec->rbk_homr, &v); prec->homr = (short)v; prec->lhor = prec->homr;
        read_input(prec, &prec->rbk_velo, &prec->velo);  prec->lvel = prec->velo;
        read_input(prec, &prec->rbk_accl, &prec->accl);  prec->lacc = prec->accl;
        v = prec->cnen; read_input(prec, &prec->rbk_cnen, &v); prec->cnen = (short)v; prec->lcne = prec->cnen;
    }

    /* ------------------------------------------------------------------
     * Timestamps and alarms
     * ------------------------------------------------------------------ */
    recGblGetTimeStamp(prec);

    /* ------------------------------------------------------------------
     * Post monitors
     * ------------------------------------------------------------------ */
    monitor_mask = recGblResetAlarms(prec);

    if (prec->lrbv != prec->rbv) {
        monitor_mask |= DBE_VALUE | DBE_LOG;
        prec->lrbv = prec->rbv;
    }
    if (prec->lval != prec->val) {
        monitor_mask |= DBE_VALUE | DBE_LOG;
        prec->lval = prec->val;
    }

    if (monitor_mask)
        db_post_events(prec, &prec->val, monitor_mask);

    db_post_events(prec, &prec->rbv,  DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->dmov, DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->movn, DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->hls,  DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->lls,  DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->athm, DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->tdir, DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->stop, DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->velo, DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->accl, DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->cnen, DBE_VALUE | DBE_LOG);

    recGblFwdLink(prec);
    prec->pact = FALSE;
    return 0;
}

/* -------------------------------------------------------------------------
 * cvt_dbaddr - called when a field is accessed by dbNameToAddr
 * ------------------------------------------------------------------------- */
static long cvt_dbaddr(DBADDR *paddr)
{
    paddr->no_elements = 1;
    return 0;
}

static long get_array_info(DBADDR *paddr, long *no_elements, long *offset)
{
    *no_elements = 1;
    *offset = 0;
    return 0;
}

/* -------------------------------------------------------------------------
 * Metadata routines -- driven by local fields EGU, PREC, HOPR/LOPR, DRVH/DRVL
 * ------------------------------------------------------------------------- */
static long get_units(DBADDR *paddr, char *units)
{
    tcmotorRecord *prec = (tcmotorRecord *)paddr->precord;
    strncpy(units, prec->egu, DB_UNITS_SIZE);
    return 0;
}

static long get_precision(const DBADDR *paddr, long *precision)
{
    tcmotorRecord *prec = (tcmotorRecord *)paddr->precord;
    *precision = prec->prec;
    recGblGetPrec(paddr, precision);
    return 0;
}

static long get_graphic_double(DBADDR *paddr, struct dbr_grDouble *pgd)
{
    tcmotorRecord *prec = (tcmotorRecord *)paddr->precord;
    pgd->upper_disp_limit = prec->hopr;
    pgd->lower_disp_limit = prec->lopr;
    return 0;
}

static long get_control_double(DBADDR *paddr, struct dbr_ctrlDouble *pcd)
{
    tcmotorRecord *prec = (tcmotorRecord *)paddr->precord;
    pcd->upper_ctrl_limit = prec->drvh;
    pcd->lower_ctrl_limit = prec->drvl;
    return 0;
}

static long get_alarm_double(DBADDR *paddr, struct dbr_alDouble *pad)
{
    pad->upper_alarm_limit   = 0.0;
    pad->upper_warning_limit = 0.0;
    pad->lower_warning_limit = 0.0;
    pad->lower_alarm_limit   = 0.0;
    return 0;
}