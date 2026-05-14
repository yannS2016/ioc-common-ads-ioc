/*
 * tcmotorRecord.c
 *
 * Record support for the TwinCAT Motor (tcmotor) record type.
 *
 * Provides motor record field naming convention (.VAL, .RBV, .DMOV etc.)
 * for TwinCAT ADS-based motors with no internal motion logic.
 *
 * Computed fields (derived each cycle, not directly linked):
 *
 *   TDIR  — computed from INP_NDIR and INP_PDIR booleans:
 *             1 if bPositiveDirection, 0 if bNegativeDirection,
 *             unchanged when motor is stopped (neither active).
 *
 *   MSTA  — motor status word computed from all status bools following
 *             motor record bit convention (see bit layout in dbd).
 *
 *   SPMG  — Stop/Pause/Move/Go state machine owned by tcmotor,
 *             replacing FB_MotionSPMG in the PLC:
 *             0=Stop/1=Pause -> assert bHalt
 *             2=Move         -> clear bHalt, trigger bMoveCmd,
 *                               auto-revert to Pause(1) when DMOV goes high
 *             3=Go           -> clear bHalt, normal operation
 *
 *   HOMF/HOMR special handler:
 *             HOMF -> eHomeMode=LOW_LIMIT(1)  + bHomeCmd=1
 *             HOMR -> eHomeMode=HIGH_LIMIT(2) + bHomeCmd=1
 *             Both share OUT_HOMF (bHomeCmd) and OUT_HMOD (eHomeMode).
 *
 * Output forwarding:
 *   All output forwarding happens in special() on CA writes.
 *   process() handles status inputs, derived fields, and DMOV edge detection.
 *   When RBK_* is read, last-sent cache is updated to prevent re-forwarding
 *   of PLC-clamped values.
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
#include "dbCommon.h"
#include "dbFldTypes.h"
#include "errMdef.h"
#include "errlog.h"
#include "recSup.h"
#include "recGbl.h"
#include "cantProceed.h"
#include "callback.h"
#include "epicsExport.h"

#define GEN_SIZE_OFFSET
#include "tcmotorRecord.h"
#undef GEN_SIZE_OFFSET

/* -------------------------------------------------------------------------
 * Delayed initialization callback — reads output field initial values
 * from PLC _RBV records after IDLY seconds (configurable per-instance).
 * Single shot — no retry. Set IDLY in the db template to tune timing.
 * ------------------------------------------------------------------------- */
static void init_output_fields(tcmotorRecord *prec)
{
    epicsInt32 cnen_val = 0;
    double rbk_val  = 0, rbk_velo = 0, rbk_accs = 0;
    double rbk_hlm  = 0, rbk_llm  = 0, rbk_bdst = 0;

    dbGetLink(&prec->rbk_val,  DBR_DOUBLE, &rbk_val,  NULL, NULL);
    dbGetLink(&prec->rbk_velo, DBR_DOUBLE, &rbk_velo, NULL, NULL);
    dbGetLink(&prec->rbk_accs, DBR_DOUBLE, &rbk_accs, NULL, NULL);
    dbGetLink(&prec->rbk_cnen, DBR_ENUM,   &cnen_val, NULL, NULL);
    dbGetLink(&prec->rbk_hlm,  DBR_DOUBLE, &rbk_hlm,  NULL, NULL);
    dbGetLink(&prec->rbk_llm,  DBR_DOUBLE, &rbk_llm,  NULL, NULL);
    dbGetLink(&prec->rbk_bdst, DBR_DOUBLE, &rbk_bdst, NULL, NULL);

    /* VBAS and VMAX — base and max velocity from PLC */
    double rbk_vbas = 0, rbk_vmax = 0;
    dbGetLink(&prec->rbk_vbas, DBR_DOUBLE, &rbk_vbas, NULL, NULL);
    dbGetLink(&prec->rbk_vmax, DBR_DOUBLE, &rbk_vmax, NULL, NULL);

    dbScanLock((dbCommon *)prec);

    prec->val  = rbk_val;   prec->lovl = rbk_val;
    prec->velo = rbk_velo;  prec->lvel = rbk_velo;
    prec->accs = rbk_accs;  prec->lacs = rbk_accs;
    prec->cnen = (short)cnen_val; prec->lcne = prec->cnen;
    prec->hlm  = rbk_hlm;   prec->lhlm = rbk_hlm;
    prec->llm  = rbk_llm;   prec->lllm = rbk_llm;
    prec->bdst = rbk_bdst;  prec->lbds = rbk_bdst;
    prec->vbas = rbk_vbas;  prec->lvbs = rbk_vbas;
    prec->vmax = rbk_vmax;  prec->lvmx = rbk_vmax;

    errlogPrintf("tcmotor %s init: val=%.3f velo=%.3f vbas=%.3f vmax=%.3f "
                 "accs=%.3f cnen=%d hlm=%.3f llm=%.3f bdst=%.3f\n",
                 prec->name, prec->val, prec->velo, prec->vbas, prec->vmax,
                 prec->accs, prec->cnen, prec->hlm, prec->llm, prec->bdst);

    recGblGetTimeStamp(prec);
    db_post_events(prec, &prec->val,  DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->velo, DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->vbas, DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->vmax, DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->accs, DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->cnen, DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->hlm,  DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->llm,  DBE_VALUE | DBE_LOG);
    db_post_events(prec, &prec->bdst, DBE_VALUE | DBE_LOG);
    prec->lval = prec->val;

    dbScanUnlock((dbCommon *)prec);
}

static void init_callback(CALLBACK *pcb)
{
    tcmotorRecord *prec;
    callbackGetUser(prec, pcb);
    init_output_fields(prec);
}



/* -------------------------------------------------------------------------
 * SPMG state values — generated from menu(tcmotorSPMG) in tcmotorRecord.dbd
 * via DBDINC. The header defines tcmotorSPMGEnum with:
 *   tcmotorSPMG_Stop=0, tcmotorSPMG_Pause=1, tcmotorSPMG_Move=2, tcmotorSPMG_Go=3
 * Use short aliases for readability throughout this file.
 * ------------------------------------------------------------------------- */
#define SPMG_STOP  tcmotorSPMG_Stop
#define SPMG_PAUSE tcmotorSPMG_Pause
#define SPMG_MOVE  tcmotorSPMG_Move
#define SPMG_GO    tcmotorSPMG_Go

/* -------------------------------------------------------------------------
 * eHomeMode menu indices for the mbbo record generated by pytmc.
 * DBR_SHORT writes to mbbo VAL as a menu index (0-based position in the
 * menu list), NOT the raw TwinCAT enum value.
 *
 * Menu order confirmed by caput testing on eHomeMode:
 *   0 = NONE
 *   1 = AUTOZERO
 *   2 = LOW_LIMIT   <- HOMF
 *   3 = HIGH_LIMIT  <- HOMR
 *   4 = HOME_INDEX
 *   5 = HOME_VIA_LOW
 *   6 = HOME_VIA_HIGH
 *   7 = ABSOLUTE_SET
 *   8 = CURRENT_POSITION_METHOD
 * ------------------------------------------------------------------------- */
#define HOME_LOW_LIMIT  2   /* HOMF: menu index 2 = LOW_LIMIT  */
#define HOME_HIGH_LIMIT 3   /* HOMR: menu index 3 = HIGH_LIMIT */

/* -------------------------------------------------------------------------
 * MSTA bit masks (motor record convention)
 * ------------------------------------------------------------------------- */
#define MSTA_DIRECTION  0x0001  /* 0=positive, 1=negative */
#define MSTA_DONE       0x0002
#define MSTA_PLUS_LS    0x0004  /* high limit switch */
#define MSTA_HOME_SWITCH 0x0008
#define MSTA_MINUS_LS   0x0020  /* low limit switch */
#define MSTA_HOMED      0x0040
#define MSTA_PRESENT    0x0080  /* always set */
#define MSTA_PROBLEM    0x0100  /* error */
#define MSTA_MOVING     0x0200
#define MSTA_COMM_ERR   0x0800
#define MSTA_AT_HOME    0x1000
#define MSTA_HOMING     0x8000

/* -------------------------------------------------------------------------
 * Forward declarations
 * ------------------------------------------------------------------------- */
static long init_record(struct dbCommon *pcommon, int pass);
static long process(struct dbCommon *pcommon);
static long special(DBADDR *paddr, int after);
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
    special,
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
 * Helper: write a double value to an output link
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
 * Helper: write a short value to an output link
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
 * compute_lvio
 * Limit violation: 1 if RBV outside soft limits, 0 otherwise.
 * Only checks when limits are non-zero (unconfigured links leave HLM/LLM=0).
 * ------------------------------------------------------------------------- */
static void compute_lvio(tcmotorRecord *prec)
{
    if (prec->hlm != 0.0 || prec->llm != 0.0)
        prec->lvio = (prec->rbv > prec->hlm || prec->rbv < prec->llm) ? 1 : 0;
    else
        prec->lvio = 0;
}

/* -------------------------------------------------------------------------
 * compute_tdir
 * Derive TDIR from NDIR/PDIR bools.
 * 1 = positive direction, 0 = negative direction.
 * Left unchanged when motor is stopped (neither flag active).
 * ------------------------------------------------------------------------- */
static void compute_tdir(tcmotorRecord *prec)
{
    if (prec->pdir)
        prec->tdir = 1;
    else if (prec->ndir)
        prec->tdir = 0;
    /* else: motor stopped, leave TDIR unchanged (last known direction) */
}

/* -------------------------------------------------------------------------
 * compute_msta
 * Build the MSTA motor status word from individual status bools.
 * Bit layout follows motor record convention.
 * ------------------------------------------------------------------------- */
static void compute_msta(tcmotorRecord *prec)
{
    epicsUInt32 msta = MSTA_PRESENT;  /* bit 7 always set — motor is present */

    if (prec->ndir) msta |= MSTA_DIRECTION;  /* 0=pos, 1=neg */
    if (prec->dmov) msta |= MSTA_DONE;
    if (prec->hls)  msta |= MSTA_PLUS_LS;
    if (prec->hsen) msta |= MSTA_HOME_SWITCH;
    if (prec->lls)  msta |= MSTA_MINUS_LS;
    if (prec->athm) msta |= MSTA_HOMED;
    if (prec->athm) msta |= MSTA_AT_HOME;
    if (prec->lerr) msta |= MSTA_PROBLEM;
    if (prec->movn) msta |= MSTA_MOVING;
    if (prec->hmng) msta |= MSTA_HOMING;

    /* Set comm error bit if any link is in alarm */
    if (prec->stat == LINK_ALARM)
        msta |= MSTA_COMM_ERR;

    prec->msta = msta;
}

/* -------------------------------------------------------------------------
 * trigger_move
 * Send motion parameters and pulse bMoveCmd.
 * Called when a move is authorized (SPMG=GO or SPMG=MOVE).
 * ------------------------------------------------------------------------- */
static void trigger_move(tcmotorRecord *prec)
{
    /* Forward motion parameters to PLC.
     * Caller must have set PACT=TRUE so PP DB_LINK targets process. */
    if (prec->velo != prec->lvel) {
        write_output(prec, &prec->out_velo, prec->velo);
        prec->lvel = prec->velo;
    }
    if (prec->accs != prec->lacs) {
        write_output(prec, &prec->out_accs, prec->accs);
        write_output(prec, &prec->out_decs, prec->accs);
        prec->lacs = prec->accs;
    }
    /* Forward VAL to fPosition */
    write_output(prec, &prec->out_val, prec->val);
    prec->lovl = prec->val;

    /* Pulse bMoveCmd */
    write_output_short(prec, &prec->out_mcmd, 1);
    /* Clear pending flag */
    prec->bvalp = 0;
}

/* -------------------------------------------------------------------------
 * process_spmg
 * Manages bHalt based on SPMG state and detects DMOV rising edge
 * to revert MOVE(2) → PAUSE(1).
 *
 * SPMG states:
 *   0=Stop  — assert bHalt
 *   1=Pause — assert bHalt
 *   2=Move  — clear bHalt, revert to Pause when DMOV goes high
 *   3=Go    — clear bHalt, stay at Go
 * ------------------------------------------------------------------------- */
static void process_spmg(tcmotorRecord *prec)
{
    short halt = (prec->spmg == SPMG_STOP || prec->spmg == SPMG_PAUSE) ? 1 : 0;

    /* Forward bHalt only when changed */
    if (halt != prec->lsto) {
        write_output_short(prec, &prec->out_stop, halt);
        prec->lsto = halt;
    }

    /* Detect DMOV rising edge (0→1) when in MOVE state → revert to PAUSE */
    if (prec->spmg == SPMG_MOVE && prec->dmov && !prec->pdmov) {
        prec->spmg = SPMG_PAUSE;
        /* Assert bHalt for new Pause state */
        if (prec->lsto != 1) {
            write_output_short(prec, &prec->out_stop, 1);
            prec->lsto = 1;
        }
        /* Post SPMG monitor for the revert */
        recGblGetTimeStamp(prec);
        db_post_events(prec, &prec->spmg, DBE_VALUE | DBE_LOG);
        prec->lspmg = prec->spmg;
    }
    /* Update previous DMOV for edge detection next cycle */
    prec->pdmov = prec->dmov;
}

/* -------------------------------------------------------------------------
 * init_record
 *
 * Pass 0: nothing to do -- links not yet resolved
 * Pass 1: read all input and readback links for initial values,
 *         seed last-sent output cache to suppress spurious ADS writes.
 * ------------------------------------------------------------------------- */
static long init_record(struct dbCommon *pcommon, int pass)
{
    tcmotorRecord *prec = (tcmotorRecord *)pcommon;
    double v;

    if (pass == 0) {
        return 0;
    }

    /* Read pure input links */
    read_input(prec, &prec->inp_rbv,  &prec->rbv);

    v = prec->dmov; read_input(prec, &prec->inp_dmov, &v); prec->dmov = (short)v;
    v = prec->movn; read_input(prec, &prec->inp_movn, &v); prec->movn = (short)v;
    v = prec->hls;  read_input(prec, &prec->inp_hls,  &v); prec->hls  = (short)v;
    v = prec->lls;  read_input(prec, &prec->inp_lls,  &v); prec->lls  = (short)v;
    v = prec->athm; read_input(prec, &prec->inp_athm, &v); prec->athm = (short)v;
    v = prec->ndir; read_input(prec, &prec->inp_ndir, &v); prec->ndir = (short)v;
    v = prec->pdir; read_input(prec, &prec->inp_pdir, &v); prec->pdir = (short)v;
    v = prec->lerr; read_input(prec, &prec->inp_err,  &v); prec->lerr = (short)v;
    v = prec->hsen; read_input(prec, &prec->inp_hsen, &v); prec->hsen = (short)v;
    v = prec->hmng; read_input(prec, &prec->inp_hmng, &v); prec->hmng = (short)v;

    /* Compute derived fields */
    compute_tdir(prec);
    compute_msta(prec);

    /* Explicitly zero BINIT2 so autosave restore cannot skip the init block */
    prec->binit2 = 0;

    /* Schedule delayed callback to read output field initial values
     * from PLC _RBV records after ADS has had time to connect. */
    callbackSetCallback(init_callback, &prec->initcb);
    callbackSetUser(prec, &prec->initcb);
    callbackSetPriority(priorityMedium, &prec->initcb);
    callbackRequestDelayed(&prec->initcb,
                           prec->idly > 0.0 ? prec->idly : 30.0);

    /* Mark initialization complete */
    prec->binit = 1;

    return 0;
}

/* -------------------------------------------------------------------------
 * process
 *
 * Called when triggered by any CP link update or a CA write to an output
 * field (pp(TRUE) in dbd) or a special() handler.
 * ------------------------------------------------------------------------- */
static long process(struct dbCommon *pcommon)
{
    tcmotorRecord *prec = (tcmotorRecord *)pcommon;
    unsigned short monitor_mask;
    double v;

    prec->pact = TRUE;

    /* Guard: if process() fires before init_record pass 1 completes
     * (via CP link from a source record), skip all processing. */
    if (!prec->binit) {
        prec->pact = FALSE;
        return 0;
    }

    /* ------------------------------------------------------------------
     * Read all pure input links
     * ------------------------------------------------------------------ */
    read_input(prec, &prec->inp_rbv,  &prec->rbv);

    v = prec->dmov; read_input(prec, &prec->inp_dmov, &v); prec->dmov = (short)v;

    v = prec->movn; read_input(prec, &prec->inp_movn, &v); prec->movn = (short)v;

    v = prec->hls;  read_input(prec, &prec->inp_hls,  &v); prec->hls  = (short)v;

    v = prec->lls;  read_input(prec, &prec->inp_lls,  &v); prec->lls  = (short)v;

    v = prec->athm; read_input(prec, &prec->inp_athm, &v); prec->athm = (short)v;

    v = prec->ndir; read_input(prec, &prec->inp_ndir, &v); prec->ndir = (short)v;

    v = prec->pdir; read_input(prec, &prec->inp_pdir, &v); prec->pdir = (short)v;

    v = prec->lerr; read_input(prec, &prec->inp_err,  &v); prec->lerr = (short)v;

    v = prec->hsen; read_input(prec, &prec->inp_hsen, &v); prec->hsen = (short)v;

    v = prec->hmng; read_input(prec, &prec->inp_hmng, &v); prec->hmng = (short)v;

    /* Read EGU from NC:Eu:Val_RBV — char waveform, read as string.
     * Buffer is sized to prec->egu so a wider PV is truncated, not overrun. */
    if (prec->inp_egu.type != CONSTANT) {
        char egu_buf[sizeof(prec->egu)] = {0};
        long egu_n = sizeof(egu_buf) - 1;
        if (!dbGetLink(&prec->inp_egu, DBR_CHAR, egu_buf, NULL, &egu_n)) {
            egu_buf[sizeof(egu_buf) - 1] = '\0';
            if (strncmp(prec->egu, egu_buf, sizeof(prec->egu)) != 0) {
                strncpy(prec->egu, egu_buf, sizeof(prec->egu) - 1);
                prec->egu[sizeof(prec->egu) - 1] = '\0';
                db_post_events(prec, &prec->egu, DBE_VALUE | DBE_LOG);
            }
        }
    }

    /* ------------------------------------------------------------------
     * Compute derived fields
     * ------------------------------------------------------------------ */
    compute_tdir(prec);
    compute_msta(prec);

    /* Set record severity based on axis state:
     * - Limit hit (HLS=0 or LLS=0) -> MAJOR alarm
     * - PLC error (LERR=1)         -> MAJOR alarm
     * - Moving normally             -> NO_ALARM */
    if (!prec->hls || !prec->lls)
        recGblSetSevr(prec, STATE_ALARM, MAJOR_ALARM);
    else if (prec->lerr)
        recGblSetSevr(prec, STATE_ALARM, MAJOR_ALARM);

    /* ------------------------------------------------------------------
     * SPMG state machine — handle DMOV rising edge for MOVE→PAUSE revert.
     * bHalt and bMoveCmd are now driven from special() on SPMG/VAL writes.
     * ------------------------------------------------------------------ */
    if (prec->binit)
        process_spmg(prec);

    /* ------------------------------------------------------------------
     * Recompute LVIO after RBV and soft limits are updated
     * ------------------------------------------------------------------ */
    compute_lvio(prec);

    /* ------------------------------------------------------------------
     * Timestamps and alarms
     * ------------------------------------------------------------------ */
    recGblGetTimeStamp(prec);

    /* ------------------------------------------------------------------
     * Post monitors — only post when value has actually changed.
     * Each field is compared against its L* (last-posted) counterpart.
     * ------------------------------------------------------------------ */
    monitor_mask = recGblResetAlarms(prec);

    #define POST_IF_CHG_S(fld, last) \
        if (prec->fld != prec->last) { \
            db_post_events(prec, &prec->fld, DBE_VALUE | DBE_LOG); \
            prec->last = prec->fld; \
        }
    #define POST_IF_CHG_D(fld, last) \
        if (prec->fld != prec->last) { \
            db_post_events(prec, &prec->fld, DBE_VALUE | DBE_LOG); \
            prec->last = prec->fld; \
        }

    if (prec->lrbv != prec->rbv) {
        db_post_events(prec, &prec->rbv, DBE_VALUE | DBE_LOG);
        prec->lrbv = prec->rbv;
    }
    /* VAL: alarm changes only — value change is posted by CA put path
     * (pp(TRUE)) and by RBK_VAL CP when PLC confirms. Posting here too
     * would generate a third event. */
    if (monitor_mask)
        db_post_events(prec, &prec->val, monitor_mask);

    POST_IF_CHG_S(dmov, ldmov)
    POST_IF_CHG_S(movn, lmovn)
    POST_IF_CHG_S(hls,  lhls)
    POST_IF_CHG_S(lls,  llls)
    POST_IF_CHG_S(athm, lathm)
    POST_IF_CHG_S(tdir, ltdir)
    if (prec->binit) POST_IF_CHG_S(spmg, lspmg)
    POST_IF_CHG_S(lvio, llvio)    POST_IF_CHG_S(jogf, ljogf)
    POST_IF_CHG_S(jogr, ljogr)
    if (prec->msta != prec->lmsta) {
        db_post_events(prec, &prec->msta, DBE_VALUE | DBE_LOG);
        prec->lmsta = prec->msta;
    }
    /* pp(TRUE) fields (VELO, ACCS, HLM, LLM, BDST, CNEN, VAL) are posted
     * automatically by the CA put path — posting again here would double
     * the monitor events. Output field monitors posted in special(). */

    #undef POST_IF_CHG_S
    #undef POST_IF_CHG_D

    recGblFwdLink(prec);
    prec->udf   = 0;
    prec->pact  = FALSE;
    return 0;
}

/* -------------------------------------------------------------------------
 * special
 *
 * Called when a field marked special(SPC_MOD) is written by a CA client.
 * Handles HOMF, HOMR, and SPMG which require side effects beyond a simple
 * dbPutLink.
 *
 * after=0: called before the field is updated (validation opportunity)
 * after=1: called after the field is updated — this is where we act
 * ------------------------------------------------------------------------- */
static long special(DBADDR *paddr, int after)
{
    tcmotorRecord *prec = (tcmotorRecord *)paddr->precord;
    short hmode;

    if (!after)
        return 0;

    /* Set PACT=TRUE so PP DB_LINK targets process when written.
     * Without PACT, dbPutLink writes the value but the target record
     * does not process, so the ADS driver never sends it to the PLC. */
    prec->pact = TRUE;

    if (paddr->pfield == (void *)&prec->val) {
        /*
         * VAL written — store internally and post monitor.
         * Forward to fPosition and trigger move ONLY if SPMG=GO(3) or MOVE(2).
         * On STOP(0) or PAUSE(1), just queue — motion will fire when
         * SPMG transitions to GO or MOVE.
         */
        prec->bvalp = 1;  /* mark pending move */
        recGblGetTimeStamp(prec);
        if (prec->val != prec->lval) {
            db_post_events(prec, &prec->val, DBE_VALUE | DBE_LOG);
            prec->lval = prec->val;
        }
        if (prec->spmg == SPMG_GO || prec->spmg == SPMG_MOVE) {
            trigger_move(prec);
        }

    } else if (paddr->pfield == (void *)&prec->velo) {
        /*
         * VELO written — store internally, post monitor.
         * Forward to PLC only if SPMG allows motion.
         */
        recGblGetTimeStamp(prec);
        if (prec->velo != prec->lvel) {
            db_post_events(prec, &prec->velo, DBE_VALUE | DBE_LOG);
            prec->lvel = prec->velo;
        }
        if (prec->spmg == SPMG_GO || prec->spmg == SPMG_MOVE) {
            write_output(prec, &prec->out_velo, prec->velo);
            prec->lvel = prec->velo;
        }

    } else if (paddr->pfield == (void *)&prec->accs) {
        /*
         * ACCS written — store internally, post monitor.
         * Forward to PLC only if SPMG allows motion.
         */
        recGblGetTimeStamp(prec);
        if (prec->accs != prec->lacs) {
            db_post_events(prec, &prec->accs, DBE_VALUE | DBE_LOG);
            prec->lacs = prec->accs;
        }
        if (prec->spmg == SPMG_GO || prec->spmg == SPMG_MOVE) {
            write_output(prec, &prec->out_accs, prec->accs);
            write_output(prec, &prec->out_decs, prec->accs);
            prec->lacs = prec->accs;
        }

    } else if (paddr->pfield == (void *)&prec->vbas) {
        recGblGetTimeStamp(prec);
        db_post_events(prec, &prec->vbas, DBE_VALUE | DBE_LOG);
        prec->lvbs = prec->vbas;
        write_output(prec, &prec->out_vbas, prec->vbas);

    } else if (paddr->pfield == (void *)&prec->vmax) {
        recGblGetTimeStamp(prec);
        db_post_events(prec, &prec->vmax, DBE_VALUE | DBE_LOG);
        prec->lvmx = prec->vmax;
        write_output(prec, &prec->out_vmax, prec->vmax);

    } else if (paddr->pfield == (void *)&prec->cnen) {
        write_output_short(prec, &prec->out_cnen, prec->cnen);
        prec->lcne = prec->cnen;
        recGblGetTimeStamp(prec);
        if (prec->cnen != prec->lcne) {
            db_post_events(prec, &prec->cnen, DBE_VALUE | DBE_LOG);
            prec->lcne = prec->cnen;
        }

    } else if (paddr->pfield == (void *)&prec->bdst) {
        write_output(prec, &prec->out_bdst, prec->bdst);
        prec->lbds = prec->bdst;
        recGblGetTimeStamp(prec);
        if (prec->bdst != prec->lbds) {
            db_post_events(prec, &prec->bdst, DBE_VALUE | DBE_LOG);
            prec->lbds = prec->bdst;
        }

    } else if (paddr->pfield == (void *)&prec->hlm) {
        write_output(prec, &prec->out_hlm, prec->hlm);
        prec->lhlm = prec->hlm;
        recGblGetTimeStamp(prec);
        if (prec->hlm != prec->lhlm) {
            db_post_events(prec, &prec->hlm, DBE_VALUE | DBE_LOG);
            prec->lhlm = prec->hlm;
        }

    } else if (paddr->pfield == (void *)&prec->llm) {
        write_output(prec, &prec->out_llm, prec->llm);
        prec->lllm = prec->llm;
        recGblGetTimeStamp(prec);
        if (prec->llm != prec->lllm) {
            db_post_events(prec, &prec->llm, DBE_VALUE | DBE_LOG);
            prec->lllm = prec->llm;
        }

    } else if (paddr->pfield == (void *)&prec->stop) {
        if (prec->stop) {
            write_output_short(prec, &prec->out_stop, 1);
            prec->lsto = 1;
            prec->spmg = SPMG_STOP;
            /* STOP is a momentary command — clear after sending */
            prec->stop = 0;
            db_post_events(prec, &prec->stop, DBE_VALUE | DBE_LOG);
        } else {
            prec->spmg = SPMG_GO;
            write_output_short(prec, &prec->out_stop, 0);
            prec->lsto = 0;
            if (prec->bvalp)
                trigger_move(prec);
        }
        recGblGetTimeStamp(prec);
        db_post_events(prec, &prec->spmg, DBE_VALUE | DBE_LOG);
        prec->lspmg = prec->spmg;

    } else if (paddr->pfield == (void *)&prec->homf) {
        /*
         * Home forward rising edge — home via low limit switch.
         * Set eHomeMode = LOW_LIMIT(1) then trigger bHomeCmd.
         */
        if (prec->homf) {
            hmode = HOME_LOW_LIMIT;
            write_output_short(prec, &prec->out_hmod, hmode);
            write_output_short(prec, &prec->out_homf, 1);
        }

    } else if (paddr->pfield == (void *)&prec->homr) {
        /*
         * Home reverse rising edge — home via high limit switch.
         * Set eHomeMode = HIGH_LIMIT(2) then trigger bHomeCmd.
         */
        if (prec->homr) {
            hmode = HOME_HIGH_LIMIT;
            write_output_short(prec, &prec->out_hmod, hmode);
            write_output_short(prec, &prec->out_homf, 1);
        }

    } else if (paddr->pfield == (void *)&prec->spmg) {
        recGblGetTimeStamp(prec);
        if (prec->spmg == SPMG_GO || prec->spmg == SPMG_MOVE) {
            write_output_short(prec, &prec->out_stop, 0);
            prec->lsto = 0;
            if (prec->bvalp)
                trigger_move(prec);
        } else {
            write_output_short(prec, &prec->out_stop, 1);
            prec->lsto = 1;
        }
        if (prec->spmg != prec->lspmg) {
            db_post_events(prec, &prec->spmg, DBE_VALUE | DBE_LOG);
            prec->lspmg = prec->spmg;
        }

    } else if (paddr->pfield == (void *)&prec->jogf) {
        /*
         * Jog forward rising edge:
         *   1. Write VAL into NC:JogIncrFwd:Goal only if value changed.
         *   2. Pulse bJogFwdCmd — PLC executes jog and clears the cmd.
         * No action on falling edge (PLC already cleared the command).
         */
        if (prec->jogf) {
            if (prec->val != prec->ljif) {
                write_output(prec, &prec->out_jif, prec->val);
                prec->ljif = prec->val;
            }
            write_output_short(prec, &prec->out_jogf, 1);
        }

    } else if (paddr->pfield == (void *)&prec->jogr) {
        /*
         * Jog reverse rising edge:
         *   1. Write VAL into NC:JogIncrBwd:Goal only if value changed.
         *   2. Pulse bJogBwdCmd — PLC executes jog and clears the cmd.
         * No action on falling edge (PLC already cleared the command).
         */
        if (prec->jogr) {
            if (prec->val != prec->ljir) {
                write_output(prec, &prec->out_jir, prec->val);
                prec->ljir = prec->val;
            }
            write_output_short(prec, &prec->out_jogr, 1);
        }
    }

    prec->pact = FALSE;
    return 0;
}

/* -------------------------------------------------------------------------
 * cvt_dbaddr
 * ------------------------------------------------------------------------- */
static long cvt_dbaddr(DBADDR *paddr)
{
    tcmotorRecord *prec = (tcmotorRecord *)paddr->precord;

    /* DBF_STRING fields need explicit type/size so CA sends them as strings.
     * field_size must match the dbd-declared size, not MAX_STRING_SIZE. */
    if (paddr->pfield == (void *)&prec->egu) {
        paddr->field_type     = DBF_STRING;
        paddr->field_size     = sizeof(prec->egu);
        paddr->dbr_field_type = DBR_STRING;
    }
    /* SHORT/MENU fields */
    else if (paddr->pfield == (void *)&prec->dmov ||
             paddr->pfield == (void *)&prec->movn ||
             paddr->pfield == (void *)&prec->hls  ||
             paddr->pfield == (void *)&prec->lls  ||
             paddr->pfield == (void *)&prec->athm ||
             paddr->pfield == (void *)&prec->tdir ||
             paddr->pfield == (void *)&prec->lerr ||
             paddr->pfield == (void *)&prec->hsen ||
             paddr->pfield == (void *)&prec->hmng ||
             paddr->pfield == (void *)&prec->ndir ||
             paddr->pfield == (void *)&prec->pdir ||
             paddr->pfield == (void *)&prec->lvio ||
             paddr->pfield == (void *)&prec->cnen ||
             paddr->pfield == (void *)&prec->stop ||
             paddr->pfield == (void *)&prec->homf ||
             paddr->pfield == (void *)&prec->homr ||
             paddr->pfield == (void *)&prec->jogf ||
             paddr->pfield == (void *)&prec->jogr) {
        paddr->field_type     = DBF_SHORT;
        paddr->field_size     = sizeof(epicsInt16);
        paddr->dbr_field_type = DBR_SHORT;
    }
    /* MENU field (SPMG) */
    else if (paddr->pfield == (void *)&prec->spmg) {
        paddr->field_type     = DBF_ENUM;
        paddr->field_size     = sizeof(epicsEnum16);
        paddr->dbr_field_type = DBR_ENUM;
    }
    /* ULONG field (MSTA) */
    else if (paddr->pfield == (void *)&prec->msta) {
        paddr->field_type     = DBF_ULONG;
        paddr->field_size     = sizeof(epicsUInt32);
        paddr->dbr_field_type = DBR_LONG;
    }
    /* DOUBLE fields — default */
    else {
        paddr->field_type     = DBF_DOUBLE;
        paddr->field_size     = sizeof(epicsFloat64);
        paddr->dbr_field_type = DBR_DOUBLE;
    }
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
 * Metadata routines
 * ------------------------------------------------------------------------- */
static long get_units(DBADDR *paddr, char *units)
{
    tcmotorRecord *prec = (tcmotorRecord *)paddr->precord;

    if (paddr->pfield == (void *)&prec->val  ||
        paddr->pfield == (void *)&prec->rbv  ||
        paddr->pfield == (void *)&prec->hlm  ||
        paddr->pfield == (void *)&prec->llm  ||
        paddr->pfield == (void *)&prec->bdst) {
        strncpy(units, prec->egu, DB_UNITS_SIZE);
    } else if (paddr->pfield == (void *)&prec->velo ||
               paddr->pfield == (void *)&prec->vbas ||
               paddr->pfield == (void *)&prec->vmax) {
        snprintf(units, DB_UNITS_SIZE, "%s/s", prec->egu);
    } else if (paddr->pfield == (void *)&prec->accs) {
        snprintf(units, DB_UNITS_SIZE, "%s/s^2", prec->egu);
    } else {
        units[0] = '\0';
    }
    return 0;
}

static long get_precision(const DBADDR *paddr, long *precision)
{
    tcmotorRecord *prec = (tcmotorRecord *)paddr->precord;

    /* For position/velocity/limit fields, use the record's PREC field directly.
     * For everything else, defer to recGblGetPrec to pick a sensible default.
     * Note: recGblGetPrec overwrites *precision unconditionally, so we must
     * return early for the value-like fields rather than fall through. */
    if (paddr->pfield == (void *)&prec->val  ||
        paddr->pfield == (void *)&prec->rbv  ||
        paddr->pfield == (void *)&prec->hlm  ||
        paddr->pfield == (void *)&prec->llm  ||
        paddr->pfield == (void *)&prec->drvh ||
        paddr->pfield == (void *)&prec->drvl ||
        paddr->pfield == (void *)&prec->hopr ||
        paddr->pfield == (void *)&prec->lopr ||
        paddr->pfield == (void *)&prec->velo ||
        paddr->pfield == (void *)&prec->vbas ||
        paddr->pfield == (void *)&prec->vmax ||
        paddr->pfield == (void *)&prec->accs ||
        paddr->pfield == (void *)&prec->bdst) {
        *precision = prec->prec;
        return 0;
    }

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
    tcmotorRecord *prec = (tcmotorRecord *)paddr->precord;

    /* Limit switches are active-low: 0 = limit hit (alarm), 1 = clear.
     * Set alarm thresholds so Typhos shows orange only when value = 0. */
    if (paddr->pfield == (void *)&prec->hls ||
        paddr->pfield == (void *)&prec->lls) {
        pad->upper_alarm_limit   = 0.0;
        pad->upper_warning_limit = 0.0;
        pad->lower_warning_limit = 0.5;
        pad->lower_alarm_limit   = 0.5;
    } else {
        pad->upper_alarm_limit   = 0.0;
        pad->upper_warning_limit = 0.0;
        pad->lower_warning_limit = 0.0;
        pad->lower_alarm_limit   = 0.0;
    }
    return 0;
}