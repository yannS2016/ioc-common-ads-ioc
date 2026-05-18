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
 *   TDIR:  computed from INP_NDIR and INP_PDIR booleans:
 *             1 if bPositiveDirection, 0 if bNegativeDirection,
 *             unchanged when motor is stopped (neither active).
 *
 *   MSTA:  motor status word computed from all status bools following
 *             motor record bit convention (see bit layout in dbd).
 *
 *   SPMG:  Stop/Pause/Move/Go state machine owned by tcmotor,
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
 * Author: Yann Stephen Mandza
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

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

/* 
 * Delayed initialization callback:  reads output field initial values
 * from PLC _RBV records after IDLY seconds (configurable per-instance).
 * Single shot:  no retry. Set IDLY in the db template to tune timing.
 *  */
static void init_output_fields(tcmotorRecord *prec)
{
    epicsInt32 cnen_val = 0;
    double rbk_val  = 0, rbk_velo = 0, rbk_accs = 0;
    double rbk_hlm  = 0, rbk_llm  = 0, rbk_bdst = 0;
    double rbk_off  = 0;

    dbGetLink(&prec->rbk_val,  DBR_DOUBLE, &rbk_val,  NULL, NULL);
    dbGetLink(&prec->rbk_velo, DBR_DOUBLE, &rbk_velo, NULL, NULL);
    dbGetLink(&prec->rbk_accs, DBR_DOUBLE, &rbk_accs, NULL, NULL);
    dbGetLink(&prec->rbk_cnen, DBR_ENUM,   &cnen_val, NULL, NULL);
    dbGetLink(&prec->rbk_hlm,  DBR_DOUBLE, &rbk_hlm,  NULL, NULL);
    dbGetLink(&prec->rbk_llm,  DBR_DOUBLE, &rbk_llm,  NULL, NULL);
    dbGetLink(&prec->rbk_bdst, DBR_DOUBLE, &rbk_bdst, NULL, NULL);
    dbGetLink(&prec->rbk_off,  DBR_DOUBLE, &rbk_off,  NULL, NULL);

    /* VBAS and VMAX:  base and max velocity from PLC */
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
    prec->off  = rbk_off;   prec->loff = rbk_off;

    errlogPrintf("tcmotor %s init: val=%.3f velo=%.3f vbas=%.3f vmax=%.3f "
                 "accs=%.3f cnen=%d hlm=%.3f llm=%.3f bdst=%.3f off=%.3f\n",
                 prec->name, prec->val, prec->velo, prec->vbas, prec->vmax,
                 prec->accs, prec->cnen, prec->hlm, prec->llm, prec->bdst,
                 prec->off);

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
    db_post_events(prec, &prec->off,  DBE_VALUE | DBE_LOG);
    prec->lval = prec->val;

    dbScanUnlock((dbCommon *)prec);
}

static void init_callback(CALLBACK *pcb)
{
    tcmotorRecord *prec;
    callbackGetUser(prec, pcb);
    init_output_fields(prec);
}


/* 
 * SPMG state values:  generated from menu(tcmotorSPMG) in tcmotorRecord.dbd
 * via DBDINC. The header defines tcmotorSPMGEnum with:
 *   tcmotorSPMG_Stop=0, tcmotorSPMG_Pause=1, tcmotorSPMG_Move=2, tcmotorSPMG_Go=3
 * Use short aliases for readability throughout this file.
 *  */
#define SPMG_STOP  tcmotorSPMG_Stop
#define SPMG_PAUSE tcmotorSPMG_Pause
#define SPMG_MOVE  tcmotorSPMG_Move
#define SPMG_GO    tcmotorSPMG_Go

/* 
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
 *  */
#define HOME_LOW_LIMIT  2   /* HOMF: menu index 2 = LOW_LIMIT  */
#define HOME_HIGH_LIMIT 3   /* HOMR: menu index 3 = HIGH_LIMIT */

/* 
 * MSTA bit masks:  canonical motor record layout.
 * See motor.h in epics-modules/motor for the upstream definition.
 *
 * Bits unused by tcmotor (POSITION, SLIP_STALL, GAIN_SUPPORT) retain their
 * canonical positions and stay zero; this leaves room for future Typhos
 * label tables to line up regardless of which bits a given record sets.
 * HOMING (bit 15) is a tcmotor-specific extension:  motor record has no
 * equivalent, but the PLC provides it and it's useful for status display.
 *  */
#define MSTA_DIRECTION   0x0001  /* bit 0:  last raw direction (0=neg, 1=pos) */
#define MSTA_DONE        0x0002  /* bit 1:  motion is complete */
#define MSTA_PLUS_LS     0x0004  /* bit 2:  plus limit switch hit */
#define MSTA_HOMLS      0x0008  /* bit 3:  home limit switch active */
/*                       0x0010   * bit 4:  unused */
#define MSTA_POSITION    0x0020  /* bit 5:  closed-loop position control (unused) */
#define MSTA_SLIP_STALL  0x0040  /* bit 6:  slip/stall detected (PLC handles via LERR) */
#define MSTA_HOME     0x0080  /* bit 7:  axis is at home position */
#define MSTA_PRESENT     0x0100  /* bit 8:  encoder present (always set) */
#define MSTA_PROBLEM     0x0200  /* bit 9:  driver/hardware error */
#define MSTA_MOVING      0x0400  /* bit 10:  non-zero velocity */
#define MSTA_GAIN_SUPPORT 0x0800 /* bit 11:  closed-loop available (unused) */
#define MSTA_COMM_ERR    0x1000  /* bit 12:  controller comm error */
#define MSTA_MINUS_LS    0x2000  /* bit 13:  minus limit switch hit */
#define MSTA_HOMED       0x4000  /* bit 14:  axis has been homed (latched) */
#define MSTA_HOMING      0x8000  /* bit 15:  homing in progress (tcmotor extension) */

/* 
 * Forward declarations
 *  */
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

/* 
 * Record support entry table (EPICS Base 7)
*/
struct rset tcmotorRSET = {
    RSETNUMBER,
    NULL,           /* report */
    NULL,           /* initialize */
    init_record,
    process,
    special,
    NULL,
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

/* 
 * Helper: read one input link into a double, return 0 on success.
 * Leaves *pval unchanged if the link is CONSTANT (not configured).
 *
*/
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

/* 
 * Helper: write a double value to an output link
 * 
*/
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

/* 
 * Helper: write a short value to an output link
 *
*/
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

/* 
 * compute_lvio
 * Limit violation: 1 if RBV outside soft limits, 0 otherwise.
 * Only checks when limits are non-zero (unconfigured links leave HLM/LLM=0).
*/
static void compute_lvio(tcmotorRecord *prec)
{
    if (prec->hlm != 0.0 || prec->llm != 0.0)
        prec->lvio = (prec->rbv > prec->hlm || prec->rbv < prec->llm) ? 1 : 0;
    else
        prec->lvio = 0;
}

/* 
 * compute_tdir
 * Derive TDIR from NDIR/PDIR bools.
 * 1 = positive direction, 0 = negative direction.
 * Left unchanged when motor is stopped (neither flag active).
*/
static void compute_tdir(tcmotorRecord *prec)
{
    if (prec->pdir)
        prec->tdir = 1;
    else if (prec->ndir)
        prec->tdir = 0;
    /* else: motor stopped, leave TDIR unchanged (last known direction) */
}

/* 
 * compute_msta
 * Build the MSTA motor status word from individual status bools.
 * Bit layout follows motor record convention.
 *  */
static void compute_msta(tcmotorRecord *prec)
{
    epicsUInt32 msta = MSTA_PRESENT;  /* always set:  encoder is present */

    /* Direction comes from TDIR (which compute_tdir derives from NDIR/PDIR
     * with hysteresis when the motor is stopped). Bit set = positive. */
    if (prec->tdir)  msta |= MSTA_DIRECTION;

    if (prec->dmov)  msta |= MSTA_DONE;

    /* Limit switches: PLC convention is active-low (HLS/LLS=0 means
     * "limit hit"), motor record MSTA convention is active-high. Invert. */
    if (!prec->hls)  msta |= MSTA_PLUS_LS;
    if (!prec->lls)  msta |= MSTA_MINUS_LS;

    /* Home-related bits, per motor record convention:
     *   HOMLS   (bit 3) <- ATHM:  state of the home limit switch (instantaneous)
     *   HOME    (bit 7) <- HOMED && |RBV-HPOS| < tolerance
     *                       Axis has been homed AND is currently sitting at
     *                       the configured home position. Tolerance is half
     *                       the least-significant displayed digit (derived
     *                       from PREC) so it auto-scales: PREC=3 yields
     *                       0.0005 EGU, PREC=0 yields 0.5 EGU.
     *   HOMED   (bit 14)<- HOMED:  homing routine completed (latched) */
    if (prec->athm)  msta |= MSTA_HOMLS;
    if (prec->homed) {
        double tol = pow(10.0, -(double)prec->prec) * 0.5;
        if (fabs(prec->rbv - prec->hpos) < tol)
            msta |= MSTA_HOME;
    }
    if (prec->homed) msta |= MSTA_HOMED;

    if (prec->lerr)  msta |= MSTA_PROBLEM;
    if (prec->movn)  msta |= MSTA_MOVING;
    if (prec->hmng)  msta |= MSTA_HOMING;

    /* COMM_ERR is set if any input/output link failed last cycle.
     * Note: prec->stat reflects last cycle's committed status (this cycle's
     * is still being accumulated in nsta until recGblResetAlarms runs), so
     * COMM_ERR lags by one process cycle. At a 100ms scan rate this is
     * not operationally significant. */
    if (prec->stat == LINK_ALARM)
        msta |= MSTA_COMM_ERR;

    prec->msta = msta;
}

/* 
 * trigger_move
 * Send motion parameters and pulse bMoveCmd.
 * Called when a move is authorized (SPMG=GO or SPMG=MOVE).
 *  */
static void trigger_move(tcmotorRecord *prec)
{
    /* Forward motion parameters and request the move.
     * Caller must have set PACT=TRUE so PP DB_LINK targets process.
     *
     * VELO and ACCS may have been staged during Pause (their special()
     * handlers post the monitor but skip the ADS write when motion is
     * blocked, leaving L* stale). The change-detect guards below catch
     * those staged updates and flush them to the PLC just before the
     * move command. */
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

/* 
 * process_spmg
 * Manages bHalt based on SPMG state and detects DMOV rising edge
 * to revert MOVE(2) → PAUSE(1).
 *
 * SPMG states:
 *   0=Stop:  assert bHalt
 *   1=Pause:  assert bHalt
 *   2=Move:  clear bHalt, revert to Pause when DMOV goes high
 *   3=Go  :  clear bHalt, stay at Go
 *  */
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

/* 
 * init_record
 *
 * Pass 0: nothing to do -- links not yet resolved
 * Pass 1: read all input and readback links for initial values,
 *         seed last-sent output cache to suppress spurious ADS writes.
 *  */
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
    v = prec->homed; read_input(prec, &prec->inp_homed, &v); prec->homed = (short)v;
    read_input(prec, &prec->inp_hpos, &prec->hpos); prec->lhpos = prec->hpos;
    v = prec->hmng; read_input(prec, &prec->inp_hmng, &v); prec->hmng = (short)v;

    /* Bidirectional fields: read once from PLC and seed L* cache so the
     * first CA write sees a no-op comparison if the value hasn't changed.
     * init_callback will also seed from RBK_* later as belt-and-braces.
     * Only commissioning-class fields where the PLC may legitimately be
     * a write source are seeded here. */
    read_input(prec, &prec->inp_off, &prec->off); prec->loff = prec->off;
    read_input(prec, &prec->inp_hlm, &prec->hlm); prec->lhlm = prec->hlm;
    read_input(prec, &prec->inp_llm, &prec->llm); prec->lllm = prec->llm;

    /* Seed VAL from RBV so the first CA put doesn't issue a spurious
     * move to position 0. UDF is cleared so the record is considered
     * defined immediately, matching motor record convention.
     * pdmov is initialized to match dmov so the VAL←RBV sync doesn't
     * fire a spurious edge on the first process() cycle. */
    prec->val   = prec->rbv;
    prec->lval  = prec->val;
    prec->pdmov = prec->dmov;
    prec->udf   = FALSE;

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

/* 
 * process
 *
 * Called when triggered by any CP link update or a CA write to an output
 * field (pp(TRUE) in dbd) or a special() handler.
 *  */
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

    /*
     * Read all pure input links
     * */
    read_input(prec, &prec->inp_rbv,  &prec->rbv);

    v = prec->dmov; read_input(prec, &prec->inp_dmov, &v); prec->dmov = (short)v;

    v = prec->movn; read_input(prec, &prec->inp_movn, &v); prec->movn = (short)v;

    v = prec->hls;  read_input(prec, &prec->inp_hls,  &v); prec->hls  = (short)v;

    v = prec->lls;  read_input(prec, &prec->inp_lls,  &v); prec->lls  = (short)v;

    v = prec->athm; read_input(prec, &prec->inp_athm, &v); prec->athm = (short)v;

    v = prec->ndir; read_input(prec, &prec->inp_ndir, &v); prec->ndir = (short)v;

    v = prec->pdir; read_input(prec, &prec->inp_pdir, &v); prec->pdir = (short)v;

    v = prec->lerr; read_input(prec, &prec->inp_err,  &v); prec->lerr = (short)v;


    v = prec->homed; read_input(prec, &prec->inp_homed, &v); prec->homed = (short)v;

    read_input(prec, &prec->inp_hpos, &prec->hpos);

    v = prec->hmng; read_input(prec, &prec->inp_hmng, &v); prec->hmng = (short)v;

    /* Read EGU from NC:Eu:Val_RBV:  char waveform, read as string.
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

    /*
     * Bidirectional fields: continuous read from PLC.
     *
     * For commissioning-class fields where the PLC may legitimately be a
     * source of writes (soft limits, user offset), we continuously mirror
     * the _RBV PV and propagate PLC-originated changes to CA clients.
     *
     * Flow:
     *   - CA writes value -> special() pushes to PLC if value != L*
     *   - PLC echoes via INP_* -> read here -> value == L* -> no-op
     *   - PLC originates a change -> read here -> value != L* (snapshot)
     *     -> sync L* and post monitor
     *
     * Motion-parameter fields (VELO, ACCS, VBAS, VMAX, BDST, CNEN) are
     * deliberately NOT bidirectional. They are EPICS-owned per motor
     * record convention: seeded once from the PLC at init via RBK_*,
     * written from CA thereafter. PLC-side changes to these are not
     * propagated back to the record.
     *
     * VAL is also EPICS-owned, with a separate convention: it tracks RBV
     * at init and after each move completes (DMOV rising edge). See the
     * VAL-sync block below.
     * */
    #define SYNC_FROM_PLC_D(fld, link, cache) do { \
        double _pre = prec->cache; \
        read_input(prec, &prec->link, &prec->fld); \
        if (prec->fld != _pre) { \
            prec->cache = prec->fld; \
            db_post_events(prec, &prec->fld, DBE_VALUE | DBE_LOG); \
        } \
    } while (0)

    SYNC_FROM_PLC_D(off, inp_off, loff);
    SYNC_FROM_PLC_D(hlm, inp_hlm, lhlm);
    SYNC_FROM_PLC_D(llm, inp_llm, lllm);

    #undef SYNC_FROM_PLC_D

    /*
     * VAL<- RBV synchronization on move completion.
     *
     * Per motor record convention, VAL tracks the current readback after
     * each move so the next CA put doesn't replay a stale setpoint as a
     * new move. We detect move completion via DMOV rising edge (0→1) so
     * mid-move RBV changes don't cancel the commanded motion.
     *
     * SPMG gating: only sync when SPMG allows motion (Go or Move). If the
     * operator paused or stopped mid-move, the DMOV rising edge reflects
     * an *interruption*, not a completion:  the staged VAL is still the
     * operator's intended target, and SPMG->Go should resume to it via
     * the bvalp flag. Without this gate, pausing mid-move would clobber
     * VAL with the current position and lose the commanded target.
     *
     * This covers both CA-initiated moves (operator writes VAL -> bvalp ->
     * trigger_move -> DMOV cycles 1->0->1 with SPMG still Go/Move) and
     * PLC-initiated moves (DMOV cycles externally; we accept the sync if
     * the operator hasn't paused us out of the SPMG=Go state).
     *
     * pdmov is updated at the end of process_spmg(), so this check sees
     * the previous cycle's value as expected. */
    if (prec->dmov && !prec->pdmov) {
        if (prec->spmg == SPMG_GO || prec->spmg == SPMG_MOVE) {
            /* Move completed normally: sync VAL<- RBV so the next CA put
             * doesn't replay the same setpoint as a new move. */
            if (prec->val != prec->rbv) {
                prec->val = prec->rbv;
                prec->lval = prec->val;
                db_post_events(prec, &prec->val, DBE_VALUE | DBE_LOG);
            }
        } else {
            /* Move was interrupted by Pause or Stop. VAL still holds the
             * operator's commanded target. Re-arm bvalp so that the next
             * SPMG->Go re-issues the move via trigger_move().
             * Only re-arm if VAL actually differs from RBV:  otherwise
             * there's no pending move to resume. */
            if (prec->val != prec->rbv) {
                prec->bvalp = 1;
            }
        }
    }

    /*
     * Compute derived fields
     * */
    compute_tdir(prec);
    compute_msta(prec);

    /*
     * Recompute LVIO after RBV and soft limits are updated.
     * compute_lvio() must run before the alarm block below so LVIO can
     * contribute to severity.
     * */
    compute_lvio(prec);

    /* Set record severity based on axis state. Each call to recGblSetSevr
     * only raises severity (priority is highest-wins), so multiple sources
     * can contribute and the worst one survives.
     *
     *   HLS=0 (high limit hit)  -> HLSV (default MAJOR)
     *   LLS=0 (low limit hit)   -> LLSV (default MAJOR)
     *   LVIO=1 (soft limit hit) -> LSV  (default MAJOR)
     *   LERR=1 (PLC error)      -> hardcoded MAJOR; equivalent to motor
     *                              record SLIP_STALL/PROBLEM, always severe
     */
    if (!prec->hls)
        recGblSetSevr(prec, HIGH_ALARM, prec->hlsv);
    if (!prec->lls)
        recGblSetSevr(prec, LOW_ALARM,  prec->llsv);
    if (prec->lvio)
        recGblSetSevr(prec, HW_LIMIT_ALARM, prec->lsv);
    if (prec->lerr)
        recGblSetSevr(prec, STATE_ALARM, MAJOR_ALARM);

    /*
     * SPMG state machine:  handle DMOV rising edge for MOVE->PAUSE revert.
     * bHalt and bMoveCmd are now driven from special() on SPMG/VAL writes.
     * */
    if (prec->binit)
        process_spmg(prec);

    /*
     * Timestamps and alarms
     * */
    recGblGetTimeStamp(prec);

    /*
     * Post monitors:  only post when value has actually changed.
     * Each field is compared against its L* (last-posted) counterpart.
     * */
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
    /* VAL: alarm changes only:  value change is posted by CA put path
     * (pp(TRUE)) and by RBK_VAL CP when PLC confirms. Posting here too
     * would generate a third event. */
    if (monitor_mask)
        db_post_events(prec, &prec->val, monitor_mask);

    POST_IF_CHG_S(dmov, ldmov)
    POST_IF_CHG_S(movn, lmovn)
    POST_IF_CHG_S(hls,  lhls)
    POST_IF_CHG_S(lls,  llls)
    POST_IF_CHG_S(athm, lathm)
    POST_IF_CHG_S(homed, lhom)
    POST_IF_CHG_D(hpos, lhpos)
    POST_IF_CHG_S(tdir, ltdir)
    if (prec->binit) POST_IF_CHG_S(spmg, lspmg)
    POST_IF_CHG_S(lvio, llvio)    POST_IF_CHG_S(jogf, ljogf)
    POST_IF_CHG_S(jogr, ljogr)
    if (prec->msta != prec->lmsta) {
        db_post_events(prec, &prec->msta, DBE_VALUE | DBE_LOG);
        prec->lmsta = prec->msta;
    }
    /* pp(TRUE) fields (VELO, ACCS, HLM, LLM, BDST, CNEN, VAL) are posted
     * automatically by the CA put path:  posting again here would double
     * the monitor events. Output field monitors posted in special(). */

    #undef POST_IF_CHG_S
    #undef POST_IF_CHG_D

    recGblFwdLink(prec);
    prec->udf   = 0;
    prec->pact  = FALSE;
    return 0;
}

/* 
 * special
 *
 * Called when a field marked special(SPC_MOD) is written by a CA client.
 * Handles HOMF, HOMR, and SPMG which require side effects beyond a simple
 * dbPutLink.
 *
 * after=0: called before the field is updated (validation opportunity)
 * after=1: called after the field is updated:  this is where we act
 *  */
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
         * VAL written:  store internally and post monitor.
         * Forward to fPosition and trigger move ONLY if SPMG=GO(3) or MOVE(2).
         * On STOP(0) or PAUSE(1), just queue:  motion will fire when
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
         * VELO written:  post monitor on change. Push to PLC only if SPMG
         * allows motion. lvel tracks "last sent to PLC", not "last written
         * by CA", so a deferred VELO change during Pause/Stop remains
         * pending until SPMG transitions and trigger_move (on the next
         * VAL write or SPMG->Go) pushes the new value.
         */
        recGblGetTimeStamp(prec);
        if (prec->velo != prec->lvel) {
            db_post_events(prec, &prec->velo, DBE_VALUE | DBE_LOG);
            if (prec->spmg == SPMG_GO || prec->spmg == SPMG_MOVE) {
                write_output(prec, &prec->out_velo, prec->velo);
                prec->lvel = prec->velo;
            }
            /* If SPMG blocks motion, lvel stays stale on purpose:  that's
             * the signal to trigger_move() that a push is owed. */
        }

    } else if (paddr->pfield == (void *)&prec->accs) {
        /*
         * ACCS written:  post monitor on change. Push to PLC (both
         * acceleration and deceleration) only if SPMG allows motion.
         * Same deferred-write semantics as VELO.
         */
        recGblGetTimeStamp(prec);
        if (prec->accs != prec->lacs) {
            db_post_events(prec, &prec->accs, DBE_VALUE | DBE_LOG);
            if (prec->spmg == SPMG_GO || prec->spmg == SPMG_MOVE) {
                write_output(prec, &prec->out_accs, prec->accs);
                write_output(prec, &prec->out_decs, prec->accs);
                prec->lacs = prec->accs;
            }
        }

    } else if (paddr->pfield == (void *)&prec->vbas) {
        /* CA-only field (no INP_VBAS): suppress redundant ADS write,
         * always post monitor. */
        if (prec->vbas != prec->lvbs) {
            write_output(prec, &prec->out_vbas, prec->vbas);
            prec->lvbs = prec->vbas;
        }
        recGblGetTimeStamp(prec);
        db_post_events(prec, &prec->vbas, DBE_VALUE | DBE_LOG);

    } else if (paddr->pfield == (void *)&prec->vmax) {
        /* CA-only field (no INP_VMAX): suppress redundant ADS write,
         * always post monitor. */
        if (prec->vmax != prec->lvmx) {
            write_output(prec, &prec->out_vmax, prec->vmax);
            prec->lvmx = prec->vmax;
        }
        recGblGetTimeStamp(prec);
        db_post_events(prec, &prec->vmax, DBE_VALUE | DBE_LOG);

    } else if (paddr->pfield == (void *)&prec->cnen) {
        /* CA-only field (no INP_CNEN): suppress redundant ADS write,
         * always post monitor. */
        if (prec->cnen != prec->lcne) {
            write_output_short(prec, &prec->out_cnen, prec->cnen);
            prec->lcne = prec->cnen;
        }
        recGblGetTimeStamp(prec);
        db_post_events(prec, &prec->cnen, DBE_VALUE | DBE_LOG);

    } else if (paddr->pfield == (void *)&prec->bdst) {
        /* CA-only field (no INP_BDST): suppress redundant ADS write,
         * always post monitor. */
        if (prec->bdst != prec->lbds) {
            write_output(prec, &prec->out_bdst, prec->bdst);
            prec->lbds = prec->bdst;
        }
        recGblGetTimeStamp(prec);
        db_post_events(prec, &prec->bdst, DBE_VALUE | DBE_LOG);

    } else if (paddr->pfield == (void *)&prec->hlm) {
        /* CA-only field (no INP_HLM): suppress redundant ADS write,
         * always post monitor. */
        if (prec->hlm != prec->lhlm) {
            write_output(prec, &prec->out_hlm, prec->hlm);
            prec->lhlm = prec->hlm;
        }
        recGblGetTimeStamp(prec);
        db_post_events(prec, &prec->hlm, DBE_VALUE | DBE_LOG);

    } else if (paddr->pfield == (void *)&prec->llm) {
        /* CA-only field (no INP_LLM): suppress redundant ADS write,
         * always post monitor. */
        if (prec->llm != prec->lllm) {
            write_output(prec, &prec->out_llm, prec->llm);
            prec->lllm = prec->llm;
        }
        recGblGetTimeStamp(prec);
        db_post_events(prec, &prec->llm, DBE_VALUE | DBE_LOG);

    } else if (paddr->pfield == (void *)&prec->off) {
        /* CA write to OFF: push to PLC and post a monitor.
         * LOFF is updated to suppress process() from re-posting when the
         * PLC echoes the value back via INP_OFF. If the value matches what
         * LOFF already holds (e.g. CA write of the same value we just read
         * from the PLC), we skip the OUT_OFF push to avoid a redundant
         * ADS write:  process() already keeps OFF and LOFF in sync. */
        if (prec->off != prec->loff) {
            write_output(prec, &prec->out_off, prec->off);
            prec->loff = prec->off;
        }
        recGblGetTimeStamp(prec);
        db_post_events(prec, &prec->off, DBE_VALUE | DBE_LOG);

    } else if (paddr->pfield == (void *)&prec->stop) {
        if (prec->stop) {
            write_output_short(prec, &prec->out_stop, 1);
            prec->lsto = 1;
            prec->spmg = SPMG_STOP;
            /* STOP is a momentary command:  clear after sending */
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
         * Home forward rising edge:  home via low limit switch.
         * Set eHomeMode = LOW_LIMIT(1) then trigger bHomeCmd.
         */
        if (prec->homf) {
            hmode = HOME_LOW_LIMIT;
            write_output_short(prec, &prec->out_hmod, hmode);
            write_output_short(prec, &prec->out_homf, 1);
        }

    } else if (paddr->pfield == (void *)&prec->homr) {
        /*
         * Home reverse rising edge:  home via high limit switch.
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
         *   2. Pulse bJogFwdCmd:  PLC executes jog and clears the cmd.
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
         *   2. Pulse bJogBwdCmd:  PLC executes jog and clears the cmd.
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

/* 
 * cvt_dbaddr
 *  */
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
             paddr->pfield == (void *)&prec->homed ||
             paddr->pfield == (void *)&prec->tdir ||
             paddr->pfield == (void *)&prec->lerr ||
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
    /* DOUBLE fields:  default */
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

/* 
 * Metadata routines
 *  */
static long get_units(DBADDR *paddr, char *units)
{
    tcmotorRecord *prec = (tcmotorRecord *)paddr->precord;

    if (paddr->pfield == (void *)&prec->val  ||
        paddr->pfield == (void *)&prec->rbv  ||
        paddr->pfield == (void *)&prec->hlm  ||
        paddr->pfield == (void *)&prec->llm  ||
        paddr->pfield == (void *)&prec->bdst ||
        paddr->pfield == (void *)&prec->off  ||
        paddr->pfield == (void *)&prec->hpos) {
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
        paddr->pfield == (void *)&prec->bdst ||
        paddr->pfield == (void *)&prec->off  ||
        paddr->pfield == (void *)&prec->hpos) {
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
