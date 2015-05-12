/*********************  P r o g r a m  -  M o d u l e ***********************
 *
 *         Name: m11_drv.c
 *      Project: M11 module driver (MDIS4)
 *
 *       Author: kp
 *        $Date: 2013/09/10 11:11:29 $
 *    $Revision: 1.6 $
 *
 *  Description: Low-level driver for M11 modules
 *
 * The M11 is an M-Module with an 68230 chip.
 *
 * The MDIS4 driver will not support all of the 68230 features.
 *
 * It will support:
 *
 * - 2x8-Mode: Port A and B are used as two 8 bit input or output ports
 *   (channel 0 and 1). Each pin can be defined seperately as in or output.
 *   Both Input and output is unbuffered. No handshakelines are used.
 *
 * - 1x16 Mode: Port A and B form an 16 bit input or output.
 *   In this mode, data direction can be defined for the whole 16 bit port
 *   only, not for each pin. Writing to the port changes the data on Port A
 *   and B at the same time. Input and output can be done on channel 0.
 *   Channel 1 does not exist in this mode.
 *
 * - Port C: (channel 2) Forms a two bit input/output port.
 *   Each pin can be defined seperately as in or output.
 *
 * - Handshake lines H1..H4 (channel 3) can be used as general purpose
 *   input pins, that can generate interrupts (signals) at a configurable
 *   edge. H3 and H4 cannot generate interrupts in 1x16 mode.
 *
 * - Timer (channel 4): Compliant to MDIS Timer Profile (first device
 *   that defines MDIS Timer profile). Can be used to produce one-shot
 *   timeouts or periodic interrupts (signals). A write to the channel
 *   sets the initial timer value. A read from the device reads the
 *   current timer value. The timer is started and stopped with SetStats
 *   M_TMR_RUN
 *
 * Features of the MDIS3 driver that will be removed or changed:
 * - Buffers for input/output removed (senseless)
 * - In 2x8 mode, data direction can be defined for each pin seperately
 * - One shot mode for timer added
 * - Timer will not be resetted when read
 * - Timer will not be stopped when read.
 * - PC0 toggle mode on timer interrupt removed
 *
 *
 *     Required: OSS, DESC, DBG, ID libraries
 *     Switches: _ONE_NAMESPACE_PER_DRIVER_
 *
 *-------------------------------[ History ]---------------------------------
 *
 * $Log: m11_drv.c,v $
 * Revision 1.6  2013/09/10 11:11:29  gv
 * R: Porting to MDIS5
 * M: Changed according to MDIS Porting Guide 0.9
 *
 * Revision 1.5  2012/12/05 10:29:30  ww
 * added set- and getstat calls to supported m99_latency test.
 * modified irq routine to handle timer interrupt of m99_latency test.
 *
 * Revision 1.4  2006/03/07 10:10:38  DPfeuffer
 * - bugfix: M11_SetStat(M11_SIGCLR_Hx): prevent to remove not installed signals
 * - M11_SetStat(), M11_GetStat(): M_TMR_XXX description fixed
 *
 * Revision 1.3  2004/04/26 17:49:07  cs
 * moved swapped variant defines to driver_sw.mak
 * replaced OSS_IrqMask/OSS_IrqUnMask with OSS_IrqMaskR/OSS_IrqRestore
 * minor typecasts for WIN2k compiler compatibility
 * bugix: fixed M11_Irq answer
 *        now output of LL_IRQ_DEVICE if interrupt caused by device
 *
 * Revision 1.2  2003/11/07 13:56:27  dschmidt
 * SET_HX_SENSE was wrong
 *
 * Revision 1.1  1999/11/03 15:40:38  kp
 * Initial Revision
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 1999 by MEN Mikro Elektronik GmbH, Nuernberg, Germany
 ****************************************************************************/

static const char RCSid[]="$Id: m11_drv.c,v 1.6 2013/09/10 11:11:29 gv Exp $";

#define _NO_LL_HANDLE		/* ll_defs.h: don't define LL_HANDLE struct */

#include <MEN/men_typs.h>   /* system dependent definitions   */
#include <MEN/maccess.h>    /* hw access macros and types     */
#include <MEN/dbg.h>        /* debug functions                */
#include <MEN/oss.h>        /* oss functions                  */
#include <MEN/desc.h>       /* descriptor functions           */
#include <MEN/modcom.h>     /* ID PROM functions              */
#include <MEN/mdis_api.h>   /* MDIS global defs               */
#include <MEN/mdis_com.h>   /* MDIS common defs               */
#include <MEN/mdis_err.h>   /* MDIS error codes               */
#include <MEN/ll_defs.h>    /* low-level driver definitions   */

#include <MEN/m_tmr_drv.h>	/* Defs for Timer profile 		  */

/*-----------------------------------------+
|  DEFINES                                 |
+-----------------------------------------*/
/* general */
#define CH_NUMBER			5			/* number of device channels */
#define USE_IRQ				TRUE		/* interrupt required  */
#define ADDRSPACE_COUNT		1			/* nr of required address spaces */
#define ADDRSPACE_SIZE		0x36		/* size of address space */
#define MOD_ID_MAGIC		0x5346      /* ID PROM magic word */
#define MOD_ID_SIZE			128			/* ID PROM size [bytes] */
#define MOD_ID				11			/* ID PROM module ID */

/* latency */
#define S_LATENCY

/* debug settings */
#define DBG_MYLEVEL			llHdl->dbgLevel
#define DBH					llHdl->dbgHdl

/* register offsets of 68230 */
#define PGC_REG    0x00    /* port general control */
#define PSR_REG    0x02    /* port service request */
#define PADD_REG   0x04    /* port A data dir */
#define PBDD_REG   0x06    /* port B data dir */
#define PCDD_REG   0x08    /* port C data dir */
#define PIV_REG    0x0a    /* port irq vector */
#define PAC_REG    0x0c    /* port A control */
#define PBC_REG    0x0e    /* port B control */
#define PAD_REG    0x10    /* port A data */
#define PBD_REG    0x12    /* port B data */
#define PAA_REG    0x14    /* port A alternate */
#define PBA_REG    0x16    /* port B alternate */
#define PCD_REG    0x18    /* port C data */
#define PS_REG     0x1a    /* port status */
#define TC_REG     0x20    /* timer control */
#define TIV_REG    0x22    /* timer irq vector */
#define CPH_REG    0x26    /* counter preload high */
#define CPM_REG    0x28    /* counter preload middle */
#define CPL_REG    0x2a    /* counter preload low */
#define CNTH_REG   0x2e    /* counter high */
#define CNTM_REG   0x30    /* counter middle */
#define CNTL_REG   0x32    /* counter low */
#define TS_REG     0x34    /* timer status */

/* misc */
#define DIR_IN		0
#define DIR_OUT		1

/*-------+
| MACROS |
+-------*/

#define M230_WRITE(ma,off,val)		MWRITE_D16(ma,off,val)
#define M230_READ(ma,off)			(MREAD_D16(ma,off)&0xff)
#define M230_SETMASK(ma,off,val)	MSETMASK_D16(ma,off,val)
#define M230_CLRMASK(ma,off,val)	MCLRMASK_D16(ma,off,val)

#define SET_HX_SENSE( mask )	    M230_WRITE( llHdl->ma, PGC_REG, \
                                    (M230_READ( llHdl->ma, PGC_REG ) & 0xf0) |(mask))
#ifdef  S_LATENCY
#define M99_MAX_SIGNALS 4
#define maM68230    ma
#define m99Hdl      llHdl
#endif

/*-----------------------------------------+
|  TYPEDEFS                                |
+-----------------------------------------*/
/* low-level handle */
typedef struct {
	/* general */
    int32           memAlloc;		/* size allocated for the handle */
    OSS_HANDLE      *osHdl;         /* oss handle */
    OSS_IRQ_HANDLE  *irqHdl;        /* irq handle */
    DESC_HANDLE     *descHdl;       /* desc handle */
    MACCESS         ma;             /* hw access handle */
	MDIS_IDENT_FUNCT_TBL idFuncTbl;	/* id function table */
	/* debug */
    u_int32         dbgLevel;		/* debug level */
	DBG_HANDLE      *dbgHdl;        /* debug handle */

	/* port I/O */
	int32			portCfg;		/* M11_2X8 or M11_1X16 */
	int32			portABDir;		/* direction for A&B in 1X16 mode */
	u_int16			portShadow[CH_NUMBER];/* output shadow registers */
	OSS_SIG_HANDLE	*hxSig[4];		/* signals installed for Hx pins */

	/* timer */
	int32			tmrMode;		/* M_TMR_STOP / M_TMR_START... */
	OSS_SIG_HANDLE	*tmrZeroSig;	/* Signal installed for zero detect */

	/* irq */
	int32			portIrqEn;		/* mask of enabled Hx irqs */
	int32			tmrIrqEn;		/* BOOL: timer IRQ enabled */
	int32			globIrqEn;		/* BOOL: global IRQ enabled */
	/* misc */
    u_int32         irqCount;       /* interrupt counter */
    u_int32         idCheck;		/* id check enabled */
#ifdef  S_LATENCY
    /* m99 compatibility */
    int             m99flag;
    u_int32         medPreLoad;     /* medium irq rate */
    int32           laststep;       /* last step of irq rate */
    u_int32         timerval;       /* current timervalue */
	u_int32			irqLatency; 	/* current interrupt latency  */
	u_int32			maxIrqLatency; 	/* max. interrupt latency  */
    OSS_SIG_HANDLE  *cond[M99_MAX_SIGNALS];
#endif
} LL_HANDLE;

/* include files which need LL_HANDLE */
#include <MEN/ll_entry.h>   /* low-level driver jump table  */
#include <MEN/m11_drv.h>   	/* M11 driver header file */

/*-----------------------------------------+
|  PROTOTYPES                              |
+-----------------------------------------*/
static int32 M11_Init(DESC_SPEC *descSpec, OSS_HANDLE *osHdl,
					   MACCESS *ma, OSS_SEM_HANDLE *devSemHdl,
					   OSS_IRQ_HANDLE *irqHdl, LL_HANDLE **llHdlP);
static int32 M11_Exit(LL_HANDLE **llHdlP );
static int32 M11_Read(LL_HANDLE *llHdl, int32 ch, int32 *value);
static int32 M11_Write(LL_HANDLE *llHdl, int32 ch, int32 value);
static int32 M11_SetStat(LL_HANDLE *llHdl,int32 ch, int32 code, INT32_OR_64 value32_or_64);
static int32 M11_GetStat(LL_HANDLE *llHdl, int32 ch, int32 code, INT32_OR_64 *value32_or_64P);
static int32 M11_BlockRead(LL_HANDLE *llHdl, int32 ch, void *buf, int32 size,
							int32 *nbrRdBytesP);
static int32 M11_BlockWrite(LL_HANDLE *llHdl, int32 ch, void *buf, int32 size,
							 int32 *nbrWrBytesP);
static int32 M11_Irq(LL_HANDLE *llHdl );
static int32 M11_Info(int32 infoType, ... );

static char* Ident( void );
static int32 Cleanup(LL_HANDLE *llHdl, int32 retCode);

static int32 SetGeneralMode( LL_HANDLE *llHdl, int32 mode, int32 dir );
static int32 SetPortDir( LL_HANDLE *llHdl, int32 ch, u_int32 dirMask );
static int32 SetHxIrqEn( LL_HANDLE *llHdl, int32 hx, int32 enable );
static void IrqSetup( LL_HANDLE *llHdl );
static int32 GetTimerState( LL_HANDLE *llHdl );

static void     setTime( LL_HANDLE *llHdl, int32 timerval );
static u_int32  getTime( LL_HANDLE *llHdl );

/**************************** M11_GetEntry *********************************
 *
 *  Description:  Initialize driver's jump table
 *
 *---------------------------------------------------------------------------
 *  Input......:  ---
 *  Output.....:  drvP  pointer to the initialized jump table structure
 *  Globals....:  ---
 ****************************************************************************/
#ifdef _ONE_NAMESPACE_PER_DRIVER_
    extern void LL_GetEntry( LL_ENTRY* drvP )
#else
    extern void M11_GetEntry( LL_ENTRY* drvP )
#endif
{
    drvP->init        = M11_Init;
    drvP->exit        = M11_Exit;
    drvP->read        = M11_Read;
    drvP->write       = M11_Write;
    drvP->blockRead   = M11_BlockRead;
    drvP->blockWrite  = M11_BlockWrite;
    drvP->setStat     = M11_SetStat;
    drvP->getStat     = M11_GetStat;
    drvP->irq         = M11_Irq;
    drvP->info        = M11_Info;
}

/******************************** M11_Init ***********************************
 *
 *  Description:  Allocate and return low-level handle, initialize hardware
 *
 *                The function initializes all channels with the
 *                definitions made in the descriptor. The interrupt
 *                is disabled.
 *
 *                The following descriptor keys are used:
 *
 *                Descriptor key        Default          Range
 *                --------------------  ---------------  -------------
 *                DEBUG_LEVEL_DESC      OSS_DBG_DEFAULT  see dbg.h
 *                DEBUG_LEVEL           OSS_DBG_DEFAULT  see dbg.h
 *                ID_CHECK              1                0..1
 *
 *                Note to ID_CHECK: Not all M11 modules have an IDPROM.
 *                So ID_CHECK is disabled by default.
 *---------------------------------------------------------------------------
 *  Input......:  descSpec   pointer to descriptor data
 *                osHdl      oss handle
 *                ma         hw access handle
 *                devSemHdl  device semaphore handle
 *                irqHdl     irq handle
 *  Output.....:  llHdlP     pointer to low-level driver handle
 *                return     success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M11_Init(
    DESC_SPEC       *descP,
    OSS_HANDLE      *osHdl,
    MACCESS         *ma,
    OSS_SEM_HANDLE  *devSemHdl,
    OSS_IRQ_HANDLE  *irqHdl,
    LL_HANDLE       **llHdlP
)
{
    LL_HANDLE *llHdl = NULL;
    u_int32 gotsize;
    int32 error;
    u_int32 value;

    /*------------------------------+
    |  prepare the handle           |
    +------------------------------*/
	*llHdlP = NULL;		/* set low-level driver handle to NULL */

	/* alloc */
    if ((llHdl = (LL_HANDLE*)OSS_MemGet(
    				osHdl, sizeof(LL_HANDLE), &gotsize)) == NULL)
       return(ERR_OSS_MEM_ALLOC);

	/* clear */
    OSS_MemFill(osHdl, gotsize, (char*)llHdl, 0x00);

	/* init */
    llHdl->memAlloc   = gotsize;
    llHdl->osHdl      = osHdl;
    llHdl->irqHdl     = irqHdl;
    llHdl->ma 		  = *ma;

    /*------------------------------+
    |  init id function table       |
    +------------------------------*/
	/* driver's ident function */
	llHdl->idFuncTbl.idCall[0].identCall = Ident;
	/* library's ident functions */
	llHdl->idFuncTbl.idCall[1].identCall = DESC_Ident;
	llHdl->idFuncTbl.idCall[2].identCall = OSS_Ident;
	/* terminator */
	llHdl->idFuncTbl.idCall[3].identCall = NULL;

    /*------------------------------+
    |  prepare debugging            |
    +------------------------------*/
	DBG_MYLEVEL = OSS_DBG_DEFAULT;	/* set OS specific debug level */
	DBGINIT((NULL,&DBH));

    /*------------------------------+
    |  scan descriptor              |
    +------------------------------*/
	/* prepare access */
    if ((error = DESC_Init(descP, osHdl, &llHdl->descHdl)))
		return( Cleanup(llHdl,error) );

    /* DEBUG_LEVEL_DESC */
    if ((error = DESC_GetUInt32(llHdl->descHdl, OSS_DBG_DEFAULT,
								&value, "DEBUG_LEVEL_DESC")) &&
		error != ERR_DESC_KEY_NOTFOUND)
		return( Cleanup(llHdl,error) );

	DESC_DbgLevelSet(llHdl->descHdl, value);	/* set level */

    /* DEBUG_LEVEL */
    if ((error = DESC_GetUInt32(llHdl->descHdl, OSS_DBG_DEFAULT,
								&llHdl->dbgLevel, "DEBUG_LEVEL")) &&
		error != ERR_DESC_KEY_NOTFOUND)
		return( Cleanup(llHdl,error) );

    DBGWRT_1((DBH, "LL - M11_Init\n"));

    /* ID_CHECK */
    if ((error = DESC_GetUInt32(llHdl->descHdl, TRUE,
								&llHdl->idCheck, "ID_CHECK")) &&
		error != ERR_DESC_KEY_NOTFOUND)
		return( Cleanup(llHdl,error) );

    /*------------------------------+
    |  check module ID              |
    +------------------------------*/
	if (llHdl->idCheck) {
		int modIdMagic = m_read((U_INT32_OR_64)llHdl->ma, 0);
		int modId      = m_read((U_INT32_OR_64)llHdl->ma, 1);

		if (modIdMagic != MOD_ID_MAGIC) {
			DBGWRT_ERR((DBH," *** M11_Init: illegal magic=0x%04x\n",
						modIdMagic));
			error = ERR_LL_ILL_ID;
			return( Cleanup(llHdl,error) );
		}
		if (modId != MOD_ID) {
			DBGWRT_ERR((DBH," *** M11_Init: illegal id=%d\n",modId));
			error = ERR_LL_ILL_ID;
			return( Cleanup(llHdl,error) );
		}
	}

    /*------------------------------+
    |  init hardware                |
    +------------------------------*/

	/*--- reset 68230 ---*/
	M230_WRITE( *ma, PGC_REG, 0x00 );	/* port control reset */
	M230_WRITE( *ma, PS_REG,  0x0f );	/* reset H1..H4 */
	M230_WRITE( *ma, TC_REG,  0x00 );	/* timer disable */
	M230_WRITE( *ma, TS_REG,  0x01 );	/* timer reset */


	/*--- init I/O ports: 2x8 mode, all pins to input ---*/
	SetGeneralMode( llHdl, M11_2X8, DIR_IN ); /* general */

	M230_WRITE( *ma, PSR_REG, 0x08 ); 	/* no DMA, autovectored IRQ */

	/*--- clear all outputs ---*/
	M11_Write( llHdl, 0, 0x00 );
	M11_Write( llHdl, 1, 0x00 );
	M11_Write( llHdl, 2, 0x00 );

	SetPortDir( llHdl, 0, 0x00 );		/* port A */
	SetPortDir( llHdl, 1, 0x00 );		/* port B */
	SetPortDir( llHdl, 2, 0x00 );		/* port C */


	SetHxIrqEn( llHdl, 0, 0x00 );		/* no IRQs for H1..4*/
	SetHxIrqEn( llHdl, 1, 0x00 );
	SetHxIrqEn( llHdl, 2, 0x00 );
	SetHxIrqEn( llHdl, 3, 0x00 );

	SET_HX_SENSE( 0x00 );				/* set default polarity for H1..4 */

	M230_WRITE( *ma, TC_REG, 0xc0 );	/* autovectored, disable irqs */

	*llHdlP = llHdl;	/* set low-level driver handle */

	return(ERR_SUCCESS);
}

/****************************** M11_Exit *************************************
 *
 *  Description:  De-initialize hardware and clean up memory
 *
 *                The function deinitializes all channels by
 *                disabling the timer and set all ports to input.
 *                The interrupt is disabled.
 *
 *---------------------------------------------------------------------------
 *  Input......:  llHdlP  	pointer to low-level driver handle
 *  Output.....:  return    success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M11_Exit(
   LL_HANDLE    **llHdlP
)
{
	MACCESS ma = (*llHdlP)->ma;

    LL_HANDLE *llHdl = *llHdlP;
	int32 error = 0;

    DBGWRT_1((DBH, "LL - M11_Exit\n"));

    /*------------------------------+
    |  de-init hardware             |
    +------------------------------*/
	SetPortDir( llHdl, 0, 0x00 ); 		/* all ports to input */
	if( llHdl->portCfg == M11_2X8 )
		SetPortDir( llHdl, 1, 0x00 );
	SetPortDir( llHdl, 2, 0x00 );

	/*--- reset 68230 ---*/
	M230_WRITE( ma, PGC_REG, 0x00 );	/* port control reset */
	M230_WRITE( ma, PS_REG,  0x0f );	/* reset H1..H4 */
	M230_WRITE( ma, TC_REG,  0x00 );	/* timer disable */
	M230_WRITE( ma, TS_REG,  0x01 );	/* timer reset */

    /*------------------------------+
    |  clean up memory               |
    +------------------------------*/
	*llHdlP = NULL;		/* set low-level driver handle to NULL */
	error = Cleanup(llHdl,error);

	return(error);
}

/****************************** M11_Read *************************************
 *
 *  Description:  Read a value from the device
 *
 *  The function reads the value from the current channel.
 *
 *  Channels 0,1,2,3 (Port A,B,C,Hx): For all pins configured as inputs
 *  reads the raw unbuffered values. Pins configured as outputs return the
 *  value from the output latch.
 *
 *  Channel 4 (Timer): Reads the current timer value
 *
 *  If configured for 1X16 mode, Channel 1 does not exist. Channel 0 returns
 *  a 16 bit value (D15..D8 = Port A7..A0 and D7..D0 = Port B7..B0)
 *
 *---------------------------------------------------------------------------
 *  Input......:  llHdl    low-level handle
 *                ch       current channel
 *  Output.....:  valueP   read value
 *                return   success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M11_Read(
    LL_HANDLE *llHdl,
    int32 ch,
    int32 *valueP
)
{
	MACCESS ma = llHdl->ma;
	int32 error=ERR_SUCCESS;

    DBGWRT_1((DBH, "LL - M11_Read: ch=%d\n",ch));

	switch( ch ){
	case 0:
		/*-----------------------+
		|  PORT A (or PORT A&B)  |
		+-----------------------*/
		if( llHdl->portCfg == M11_1X16 ){
			/*--- port A&B (16 bit reg) ---*/
			*valueP = (M230_READ( ma, PAD_REG )<<8) |
				      M230_READ( ma, PBD_REG );
		}
		else
			/*--- port A (8 bit reg) ---*/
			*valueP = M230_READ( ma, PAD_REG );
		break;

	case 1:
		/*----------+
		|  Port B   |
		+----------*/
		if( llHdl->portCfg == M11_1X16 ){
			DBGWRT_ERR((DBH, "*** M11_Read: attempt to use ch1 "
						" in M11_1X16 mode\n"));
			error = ERR_LL_ILL_CHAN;
			break;
		}

		*valueP = M230_READ( ma, PBD_REG );
		break;

	case 2:
		/*----------+
		|  Port C   |
		+----------*/
		*valueP = M230_READ( ma, PCD_REG )&0x03;
		break;

	case 3:
		/*----------+
		|  H1..4    |
		+----------*/
		*valueP = (M230_READ( ma, PS_REG ) & 0xf0) >> 4;
		break;

	case 4:
		/*----------+
		|  Timer    |
		+----------*/

	    {
			u_int8 hi, mid, lo, hi2, mid2;

			/*--- read timer reg ---*/
			do {
				hi 	= M230_READ( ma, CNTH_REG );
				mid = M230_READ( ma, CNTM_REG );
				lo  = M230_READ( ma, CNTL_REG );

				/*--- read mid and hi again to check for wrap ---*/
				mid2 = M230_READ( ma, CNTM_REG );
				hi2  = M230_READ( ma, CNTH_REG );
			} while( (hi!=hi2) ||(mid != mid2));

			*valueP = ((u_int32)hi<<16) | ((u_int32)mid<<8) | lo;

		}
		break;

	default:
		error = ERR_LL_ILL_CHAN;
		break;

	}

	DBGWRT_1((DBH,"M11_Read: error=%d value=0x%x\n", error,*valueP ));

	return(error);
}

/****************************** M11_Write ************************************
 *
 *  Description:  Write a value to the device
 *
 *  The function writes a value to the current channel.
 *
 *  Channels 0,1,2 (Port A,B,C): directly updates all pins configured as
 *  outputs.
 *
 *  Channel 3 (Hx) returns always ERR_LL_ILL_DIR
 *
 *  Channel 4 (Timer): writes the timer preload register. Restarts the
 *  timer if it was started before.
 *
 *---------------------------------------------------------------------------
 *  Input......:  llHdl    low-level handle
 *                ch       current channel
 *                value    value to write
 *  Output.....:  return   success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M11_Write(
    LL_HANDLE *llHdl,
    int32 ch,
    int32 value
)
{
	MACCESS ma = llHdl->ma;
	int32 error=ERR_SUCCESS;

    DBGWRT_1((DBH, "LL - M11_Write: ch=%d value=0x%x\n",ch, value));

	switch( ch ){
	case 0:
		/*-----------------------+
		|  PORT A (or PORT A&B)  |
		+-----------------------*/
		if( llHdl->portCfg == M11_1X16 ){
			/*--- port A&B (16 bit reg) ---*/
			M230_WRITE( ma, PAD_REG, value >> 8 );
			M230_WRITE( ma, PBD_REG, value);
		}
		else
			/*--- port A (8 bit reg) ---*/
			M230_WRITE( ma, PAD_REG, value);
		break;

	case 1:
		/*----------+
		|  Port B   |
		+----------*/
		if( llHdl->portCfg == M11_1X16 ){
			DBGWRT_ERR((DBH, "*** M11_Write: attempt to use ch1 "
						" in M11_1X16 mode\n"));
			error = ERR_LL_ILL_CHAN;
			break;
		}

		M230_WRITE( ma, PBD_REG, value);
		break;

	case 2:
		/*----------+
		|  Port C   |
		+----------*/
		M230_WRITE( ma, PCD_REG, value & 0x3);
		break;

	case 3:
		error = ERR_LL_ILL_DIR;
		break;

	case 4:
		/*----------+
		|  Timer    |
		+----------*/
		/*--- load preload register ---*/
	    {
			u_int8 tcVal;
			OSS_IRQ_STATE irqState;

			irqState = OSS_IrqMaskR( llHdl->osHdl, llHdl->irqHdl );

			tcVal = M230_READ( ma, TC_REG );
			M230_WRITE( ma, TC_REG, tcVal & ~0x1 );	/* stop timer */

			M230_WRITE( ma, CPH_REG, (value >> 16) & 0xff);	/* preload high */
			M230_WRITE( ma, CPM_REG, (value >>  8) & 0xff);	/* preload mid */
			M230_WRITE( ma, CPL_REG, value & 0xff);			/* preload low */
			M230_WRITE( ma, TS_REG, 0x1 ); 					/* timer reset */
			M230_WRITE( ma, TC_REG, tcVal ); 				/* timer restore */

			OSS_IrqRestore( llHdl->osHdl, llHdl->irqHdl, irqState );
		}
		break;

	default:
		error = ERR_LL_ILL_CHAN;
		break;
	}

	/*--- update shadow regs ---*/
	if( error == ERR_SUCCESS )
		llHdl->portShadow[ch] = (int16)value;

	return(error);
}

/****************************** M11_SetStat **********************************
 *
 *  Description:  Set the driver status
 *
 *  The following status codes are supported:
 *
 *  Code                 Description                     Values
 *  -------------------  ------------------------------  ----------
 *  M11_SETPINS          set pins in curr. channel
 *  M11_CLRPINS          clear pins in curr. channel
 *  M11_PORTCFG		   	 define port A/B mode            M11_2X8/M11_1X16
 *  M11_PINDIR			 setup data direction reg.
 *  M11_SIGSET_H[1..4]	 install signal for Hx pin		 sigCode
 *                       and enable Hx IRQ
 *  M11_SIGCLR_H[1..4]	 remove signal for Hx pin
 *  M11_HXSENSE			 define polarity for Hx IRQ
 *  M_TMR_RUN			 perform timer action			 M_TMR_STOP
 *														 M_TMR_START_ONE_SHOT
 *														 M_TMR_START_FREE_RUNNING
 *
 *  M_TMR_SIGSET_ZERO	 install signal for timer		 sigCode
 *  M_TMR_SIGCLR_ZERO	 remove signal for timer
 *  M_LL_DEBUG_LEVEL     driver debug level          	 see dbg.h
 *  M_MK_IRQ_ENABLE      interrupt enable            	 0..1
 *  M_LL_IRQ_COUNT       interrupt counter           	 0..max
 *
 *---------------------------------------------------------------------------
 *  Input......:  llHdl         low-level handle
 *                code          status code
 *                ch            current channel
 *                value32_or_64 data or
 *                              pointer to block data structure (M_SG_BLOCK)  (*)
 *                              (*) = for block status codes
 *  Output.....:  return        success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M11_SetStat(
    LL_HANDLE *llHdl,
    int32  code,
    int32  ch,
    INT32_OR_64 value32_or_64
)
{
    int32       value  = (int32)value32_or_64;  /* 32bit value     */
    /* INT32_OR_64 valueP = value32_or_64;         /\* stores 32/64bit pointer *\/ ts: unused -> compiler warning */

	int32 error = ERR_SUCCESS;

    DBGWRT_1((DBH, "LL - M11_SetStat: ch=%d code=0x%04x value=0x%x\n",
			  ch,code,value));

    switch(code) {
	case M11_SETPINS:
		/*--------------------------+
		|  SET pins                 |
		+--------------------------*/
		error = M11_Write( llHdl, ch, llHdl->portShadow[ch] | value);
		break;

	case M11_CLRPINS:
		/*--------------------------+
		|  CLEAR pins               |
		+--------------------------*/
		error = M11_Write( llHdl, ch, llHdl->portShadow[ch] & ~value);
		break;

	case M_LL_DEBUG_LEVEL:
        /*--------------------------+
        |  debug level              |
        +--------------------------*/
		llHdl->dbgLevel = value;
		break;

	case M_MK_IRQ_ENABLE:
        /*--------------------------+
		|  enable interrupts        |
        +--------------------------*/
        DBGWRT_1((DBH, "LL - M11_SetStat: M_MK_IRQ_ENABLE: value=%d\n",value));
 		llHdl->globIrqEn = value;
#ifdef S_LATENCY
        if (llHdl->m99flag)
        {
            if( value ) {
                MWRITE_D16( m99Hdl->maM68230, TC_REG, 0xa1 );    /* timer irq (enabled) */
            }
            else {
                MWRITE_D16( m99Hdl->maM68230, TC_REG, 0x81 );    /* timer irq (disabled) */
			    MWRITE_D16(m99Hdl->maM68230, TS_REG, 0xff);  /* clear interrupt */
		    }
		}
		else
#endif
		    IrqSetup(llHdl);
        break;

	case M_LL_IRQ_COUNT:
        /*--------------------------+
        |  set irq counter          |
        +--------------------------*/
		llHdl->irqCount = value;
		break;

	case M11_PORTCFG:
	{
		/*--------------------------+
		|  Define PORT A/B mode     |
		+--------------------------*/
		u_int8 modeChanged = (llHdl->portCfg != value );

		/* If mode changed, define port A/B to input, disable Hx IRQs */
		if( modeChanged ){
			M11_SetStat( llHdl, M11_SIGCLR_H1, 0, 0 );
			M11_SetStat( llHdl, M11_SIGCLR_H2, 0, 0 );
			M11_SetStat( llHdl, M11_SIGCLR_H3, 0, 0 );
			M11_SetStat( llHdl, M11_SIGCLR_H4, 0, 0 );

			SetPortDir( llHdl, 0, 0x00 );
			SetPortDir( llHdl, 1, 0x00 );

			M11_Write( llHdl, 0, 0x00 ); /* clear outputs */
			M11_Write( llHdl, 1, 0x00 );

			llHdl->portABDir = DIR_IN;
		}

		/* setup new mode */
		error = SetGeneralMode( llHdl, value, llHdl->portABDir );

		break;
	}
	case M11_PINDIR:
		/*--------------------------+
		|  Per PIN data dircection  |
		+--------------------------*/
		error = SetPortDir( llHdl, ch, value );
		break;

	case M11_SIGSET_H1:
	case M11_SIGSET_H2:
	case M11_SIGSET_H3:
	case M11_SIGSET_H4:
		/*----------------------------------------+
		|  Install/enable IRQ and signal for Hx   |
		+----------------------------------------*/
	    {
			u_int32 hx = code-M11_SIGSET_H1;

            /* check for signal already installed */
			if( llHdl->hxSig[hx] ){
				error = ERR_OSS_SIG_SET;
				break;
			}
			if( (error = OSS_SigCreate( llHdl->osHdl, value,
										&llHdl->hxSig[hx])))
				break;

			error = SetHxIrqEn( llHdl, hx, TRUE );
		}
		break;

	case M11_SIGCLR_H1:
	case M11_SIGCLR_H2:
	case M11_SIGCLR_H3:
	case M11_SIGCLR_H4:
		/*------------------+
		|  Disable Hx IRQ   |
		+------------------*/
	    {
			u_int32 hx = code-M11_SIGCLR_H1;

			if( (error = SetHxIrqEn( llHdl, hx, FALSE )))
				break;

			/* signal must be installed! */
			if( llHdl->hxSig[hx] ){
				if( (error = OSS_SigRemove( llHdl->osHdl,
											&llHdl->hxSig[hx])))
					break;
			}

		}
		break;

	case M11_HXSENSE:
		/*--------------+
		|  HX Polarity  |
		+--------------*/
		SET_HX_SENSE(value);
		break;

	case M_TMR_RUN:
		/*------------------+
		|  Start/Stop timer |
		+------------------*/
	{
		OSS_IRQ_STATE irqState;

		irqState = OSS_IrqMaskR( llHdl->osHdl, llHdl->irqHdl );

		switch( value ){

		case M_TMR_STOP:
			M230_CLRMASK( llHdl->ma, TC_REG, 0x1 );	/* disable counter */
			llHdl->tmrMode = M_TMR_STOP;
			break;

		case M_TMR_START_ONE_SHOT:
			llHdl->tmrMode = M_TMR_START_ONE_SHOT;
			M230_CLRMASK( llHdl->ma, TC_REG, 0x1 );	/* restart counter */
			M230_SETMASK( llHdl->ma, TC_REG, 0x1 );
			break;

		case M_TMR_START_FREE_RUNNING:
			llHdl->tmrMode = M_TMR_START_FREE_RUNNING;
			M230_CLRMASK( llHdl->ma, TC_REG, 0x1 );	/* restart counter */
			M230_SETMASK( llHdl->ma, TC_REG, 0x1 );
			break;

		default:
			error = ERR_LL_ILL_PARAM;
			break;
		}
		OSS_IrqRestore( llHdl->osHdl, llHdl->irqHdl, irqState );

		break;
	}
	case M_TMR_SIGSET_ZERO:
		/*----------------------------------+
		|  Install signal for zero reached  |
		+----------------------------------*/
		if( llHdl->tmrZeroSig ){
			error = ERR_OSS_SIG_SET;
			break;
		}
		if( (error = OSS_SigCreate( llHdl->osHdl, value,
									&llHdl->tmrZeroSig)))
			break;

		llHdl->tmrIrqEn = TRUE;
		IrqSetup( llHdl );

		break;

	case M_TMR_SIGCLR_ZERO:
		/*----------------------------------+
		|  Remove signal for zero reached   |
		+----------------------------------*/
		if (llHdl->tmrZeroSig == NULL) {
			DBGWRT_ERR((DBH, " *** M11_SetStat: tmr signal not installed\n"));
			error = ERR_OSS_SIG_CLR ;
			break;
		}

		llHdl->tmrIrqEn = FALSE;
		IrqSetup( llHdl );

		error = OSS_SigRemove( llHdl->osHdl, &llHdl->tmrZeroSig);
		break;

#ifdef  S_LATENCY
        /*--------------------------+
        |  define irq rate          |
        +--------------------------*/
        case M99_TIMERVAL:
        {
            u_int8  tc_reg;

            DBGWRT_1((DBH, "LL - M11_SetStat: M99_TIMERVAL: value=%d\n",value));
            DBGWRT_1((DBH, "LL - M11_SetStat: M99_TIMERVAL: ma=0x%x\n",m99Hdl->maM68230));

            if( value<1 || 0xffffff<value )           /* illgal timer value ? */
                return(ERR_LL_ILL_PARAM);

            m99Hdl->medPreLoad = value;
            m99Hdl->laststep = -1;

            tc_reg = (u_int8)MREAD_D16( m99Hdl->maM68230, TC_REG ); /* get timer control */
            MWRITE_D16( m99Hdl->maM68230, TC_REG, tc_reg & 0xfe);   /* timer halt */
            setTime( m99Hdl, value );                               /* load timer */
            MWRITE_D16( m99Hdl->maM68230, TS_REG, 0x01 );           /* timer reset */
            MWRITE_D16( m99Hdl->maM68230, TC_REG, tc_reg );         /* timer control restore */

            m99Hdl->m99flag = 1;
            break;
        }
        /*--------------------------+
        |  set/clr signal cond 1..4 |
        +--------------------------*/
        case M99_SIG_set_cond1:
        case M99_SIG_set_cond2:
        case M99_SIG_set_cond3:
        case M99_SIG_set_cond4:
		{
		    int32  cond;
#ifdef MASK_IRQ_ILLEGAL
		   OSS_IRQ_STATE irqState;
           irqState = OSS_IrqMaskR( m99Hdl->osHdl, m99Hdl->irqHdl );  /* DISABLE irqs */
#endif
           DBGWRT_1((DBH, "LL - M11_SetStat: M99_SIG_set_condx\n"));

		   cond = code - (M99_SIG_set_cond1);    /* condition nr */
           if( m99Hdl->cond[cond] != NULL )	     /* already defined ? */
           {
               error = ERR_OSS_SIG_SET;        /* can't set ! */
           }
           else
           {
               error = OSS_SigCreate( m99Hdl->osHdl, value, &m99Hdl->cond[cond]);
           }/*if*/
#ifdef MASK_IRQ_ILLEGAL
           OSS_IrqRestore( m99Hdl->osHdl, m99Hdl->irqHdl, irqState );  /* ENABLE irqs */
#endif
           break;
		}
        case M99_SIG_clr_cond1:
        case M99_SIG_clr_cond2:
        case M99_SIG_clr_cond3:
        case M99_SIG_clr_cond4:
        {
		    int32  cond;

            DBGWRT_1((DBH, "LL - M11_SetStat: M99_SIG_clr_condx\n"));

            cond = code - (M99_SIG_clr_cond1);       /* condition nr */

            if( m99Hdl->cond[cond] == NULL )   /* not defined ? */
            {
               error = ERR_OSS_SIG_SET;        /* can't clr ! */
            }
            else
            {
			   error = (OSS_SigRemove( m99Hdl->osHdl, &m99Hdl->cond[cond] ) );
            }/*if*/
            break;
         }
#endif
        /*--------------------------+
        |  (unknown)                |
        +--------------------------*/
	default:
		error = ERR_LL_UNK_CODE;
    }

	return(error);
}

/****************************** M11_GetStat **********************************
 *
 *  Description:  Get the driver status
 *
 *  The following status codes are supported:
 *
 *  Code                 Description                 	 Values
 *  -------------------  ------------------------------	 ----------
 *  M11_PORTCFG		   	 get port A/B mode               M11_2X8/M11_1X16
 *  M11_PINDIR			 get data direction reg.
 *  M11_SIGSET_H[1..4]	 get install signal for Hx pin	 sigCode
 *  M11_HXSENSE			 get polarity of Hx IRQ
 *  M_TMR_RESOLUTION     get timer resolution (ticks/sec) 250000
 *  M_TMR_RUN			 get timer state			 	 M_TMR_STOP
 *														 M_TMR_START_ONE_SHOT
 *														 M_TMR_START_FREE_RUNNING
 *
 *  M_TMR_SIGSET_ZERO	 get signal for timer		 	 sigCode

 *  M_LL_DEBUG_LEVEL     driver debug level          	 see dbg.h
 *  M_LL_CH_NUMBER       number of channels          	 5
 *  M_LL_CH_DIR          direction of curr. chan.    	 M_CH_IN for ch 3
 *														 M_CH_INOUT otherwise
 *  M_LL_CH_LEN          length of curr. ch. [bits]  	 see code
 *  M_LL_CH_TYP          description of curr. chan.  	 M_CH_BINARY fpr ch0..3
 *														 M_CH_PROFILE_TMR ch4
 *  M_LL_IRQ_COUNT       interrupt counter           	 0..max
 *  M_LL_ID_CHECK        EEPROM is checked           	 0..1
 *  M_LL_ID_SIZE         EEPROM size [bytes]         	 128
 *  M_LL_BLK_ID_DATA     EEPROM raw data            	 -
 *  M_MK_BLK_REV_ID      ident function table ptr    	 -
 *
 *---------------------------------------------------------------------------
 *  Input......:  llHdl          low-level handle
 *                code           status code
 *                ch             current channel
 *                value32_or_64P pointer to block data structure (M_SG_BLOCK)(*)
 *                               (*) = for block status codes
 *  Output.....:  value32_or_64P data ptr or pointer to block data
 *                               structure (M_SG_BLOCK)(*)
 *
 *                return         success (0) or error code
 *                               (*) = for block status codes
 *  Globals....:  ---
 ****************************************************************************/
static int32 M11_GetStat(
    LL_HANDLE *llHdl,
    int32  code,
    int32  ch,
    INT32_OR_64 *value32_or_64P
)
{
    int32       *valueP   = (int32*)value32_or_64P;         /* pointer to 32bit value  */
    INT32_OR_64 *value64P = value32_or_64P;                 /* stores 32/64bit pointer  */
    M_SG_BLOCK  *blk      = (M_SG_BLOCK*)value32_or_64P;    /* stores block struct pointer */
	MACCESS ma = llHdl->ma;

	int32 error = ERR_SUCCESS;

    DBGWRT_1((DBH, "LL - M11_GetStat: ch=%d code=0x%04x\n",
			  ch,code));

    switch(code)
    {
	case M_LL_DEBUG_LEVEL:
        /*--------------------------+
		|  debug level              |
        +--------------------------*/
		*valueP = llHdl->dbgLevel;
		break;
	case M_LL_CH_NUMBER:
        /*--------------------------+
        |  number of channels       |
        +--------------------------*/
		*valueP = CH_NUMBER;
		break;

	case M_LL_CH_DIR:
        /*--------------------------+
        |  channel direction        |
        +--------------------------*/
		switch(ch){
		case 0: *valueP = M_CH_INOUT; break; /* A */
		case 1: *valueP = M_CH_INOUT; break; /* B */
		case 2: *valueP = M_CH_INOUT; break; /* C */
		case 3: *valueP = M_CH_IN; 	  break; /* H */
		case 4: *valueP = M_CH_INOUT; break; /* TMR */
		}
		break;

	case M_LL_CH_LEN:
        /*--------------------------+
		|  channel length [bits]    |
        +--------------------------*/
		switch(ch){
		case 0: *valueP = llHdl->portCfg==M11_2X8 ? 8 : 16; break; /* A */
		case 1: *valueP = llHdl->portCfg==M11_2X8 ? 8 :  0; break; /* B */
		case 2: *valueP = 2; 								break; /* C */
		case 3: *valueP = 4; 	  							break; /* H */
		case 4: *valueP = 24; 								break; /* TMR */
		}
		break;

	case M_LL_CH_TYP:
        /*--------------------------+
        |  channel type info        |
        +--------------------------*/
		switch(ch){
		case 0: *valueP = M_CH_BINARY; break; /* A */
		case 1: *valueP = M_CH_BINARY; break; /* B */
		case 2: *valueP = M_CH_BINARY; break; /* C */
		case 3: *valueP = M_CH_BINARY; break; /* H */
		case 4: *valueP = M_CH_PROFILE_TMR;break; /* TMR */
		}
		break;

	case M_LL_IRQ_COUNT:
        /*--------------------------+
        |  irq counter              |
        +--------------------------*/
		*valueP = llHdl->irqCount;
		break;
	case M_LL_ID_CHECK:
        /*--------------------------+
        |  ID PROM check enabled    |
        +--------------------------*/
		*valueP = llHdl->idCheck;
		break;
	case M_LL_ID_SIZE:
        /*--------------------------+
        |   ID PROM size            |
        +--------------------------*/
		*valueP = MOD_ID_SIZE;
		break;
	case M_LL_BLK_ID_DATA:
	{
        /*--------------------------+
        |   ID PROM data            |
        +--------------------------*/
		u_int32 n;
		u_int16 *dataP = (u_int16*)blk->data;

		if (blk->size < MOD_ID_SIZE)		/* check buf size */
			return(ERR_LL_USERBUF);

		for (n=0; n<MOD_ID_SIZE/2; n++)		/* read MOD_ID_SIZE/2 words */
			*dataP++ = (int16)m_read((U_INT32_OR_64)llHdl->ma, (int8)n);

		break;
	}
	case M_MK_BLK_REV_ID:
        /*--------------------------+
		|   ident table pointer     |
		|   (treat as non-block!)   |
		+--------------------------*/
		*value64P = (INT32_OR_64)&llHdl->idFuncTbl;
		break;

	case M11_PORTCFG:
		/*--------------------------+
		|  Port A/B configuration   |
		+--------------------------*/
		*valueP = llHdl->portCfg;
		break;

	case M11_PINDIR:
		/*--------------------------+
		|  Port Pin Direction       |
		+--------------------------*/
		switch(ch){
		case 0:
			if( llHdl->portCfg == M11_2X8 )
				*valueP = M230_READ( ma, PADD_REG );
			else
				*valueP = (M230_READ( ma, PADD_REG )<<8) |
					      M230_READ( ma, PBDD_REG );
			break;

		case 1:
			if( llHdl->portCfg == M11_1X16 )
				error = ERR_LL_ILL_CHAN;
			else
				*valueP = M230_READ( ma, PBDD_REG );
			break;

		case 2:
			*valueP = M230_READ( ma, PCDD_REG ) & 0x3;
			break;

		case 3:
			*valueP = 0;
			break;
		default:
			error = ERR_LL_ILL_CHAN;
			break;
		}
		break;

	case M11_SIGSET_H1:
	case M11_SIGSET_H2:
	case M11_SIGSET_H3:
	case M11_SIGSET_H4:
	{
		/*--------------+
		|  Signal code  |
		+--------------*/
		int32 dummy;
		int32 hx = code - M11_SIGSET_H1;

		/* return signal */
		if (llHdl->hxSig[hx] == NULL)
			*valueP = 0x00;
		else
			OSS_SigInfo(llHdl->osHdl, llHdl->hxSig[hx], valueP, &dummy);
		break;
	}

	case M11_HXSENSE:
		/*--------------+
		|  HX Polarity  |
		+--------------*/
		*valueP = M230_READ( ma, PGC_REG ) & 0xf;
		break;

	case M_TMR_RESOLUTION:
		/*-------------------+
		|  Timer resolution  |
		+-------------------*/
		*valueP = 250000;
		break;

	case M_TMR_RUN:
		/*-------------------+
		|  Get timer state   |
		+-------------------*/
		*valueP = GetTimerState( llHdl );

		break;

	case M_TMR_SIGSET_ZERO:
	{
		/*--------------+
		|  Signal code  |
		+--------------*/
		int32 dummy;

		/* return signal */
		if (llHdl->tmrZeroSig == NULL)
			*valueP = 0x00;
		else
			OSS_SigInfo(llHdl->osHdl, llHdl->tmrZeroSig, valueP, &dummy);
		break;
	}
#ifdef  S_LATENCY
    /*--------------------------+
    |  get current irq rate     |
    +--------------------------*/
    case M99_TIMERVAL:
        DBGWRT_1((DBH, "LL - M11_GetStat: M99_TIMERVAL\n"));

        *valueP = m99Hdl->timerval;
        break;
    /*----------------------------------------+
    |  get timer ticks elapsed since last irq |
    +----------------------------------------*/
	case M99_GET_TIME:
	{
		OSS_IRQ_STATE irqState;
	    DBGWRT_1((DBH, "LL - M11_GetStat: M99_GET_TIME\n"));
		irqState = OSS_IrqMaskR( m99Hdl->osHdl, m99Hdl->irqHdl );  /* DISABLE irqs */
		*valueP = m99Hdl->timerval - getTime( m99Hdl );
		OSS_IrqRestore( m99Hdl->osHdl, m99Hdl->irqHdl, irqState );
		break;
	}
	case M99_IRQ_LAT:
	    DBGWRT_1((DBH, "LL - M11_GetStat: M99_IRQ_LAT\n"));
	    *valueP = m99Hdl->irqLatency;
		break;

	/*--------------------------+
    |  get     signal cond 1..4 |
    +--------------------------*/
    case M99_SIG_set_cond1:
    case M99_SIG_set_cond2:
    case M99_SIG_set_cond3:
    case M99_SIG_set_cond4:
    {
        int32   cond;
        int32   processId;

	    DBGWRT_1((DBH, "LL - M11_GetStat: M99_SIG_set_condx\n"));

        cond = code - (M99_SIG_set_cond1);      /* condition nr */

        if( m99Hdl->cond[cond] == NULL )   /* not defined ? */
        {
		   *valueP = 0;
        }
        else
        {
		   OSS_SigInfo( m99Hdl->osHdl, m99Hdl->cond[cond], valueP, &processId);
	    }/*if*/
       break;
    }
#endif

	default:
        /*--------------------------+
        |  (unknown)                |
        +--------------------------*/
		error = ERR_LL_UNK_CODE;
    }

	return(error);
}

/******************************* M11_BlockRead *******************************
 *
 *  Description:  Read a data block from the device (NOT SUPPORTED FOR M11)
 *
 *---------------------------------------------------------------------------
 *  Input......:  llHdl        low-level handle
 *                ch           current channel
 *                buf          data buffer
 *                size         data buffer size
 *  Output.....:  nbrRdBytesP  number of read bytes
 *                return       success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M11_BlockRead(
     LL_HANDLE *llHdl,
     int32     ch,
     void      *buf,
     int32     size,
     int32     *nbrRdBytesP
)
{
    DBGWRT_1((DBH, "LL - M11_BlockRead: ch=%d, size=%d\n",ch,size));

	/* return number of read bytes */
	*nbrRdBytesP = 0;

	return(ERR_LL_ILL_FUNC);
}

/****************************** M11_BlockWrite *******************************
 *
 *  Description:  Write a data block to the device (NOT SUPPORTED FOR M11)
 *
 *---------------------------------------------------------------------------
 *  Input......:  llHdl        low-level handle
 *                ch           current channel
 *                buf          data buffer
 *                size         data buffer size
 *  Output.....:  nbrWrBytesP  number of written bytes
 *                return       success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M11_BlockWrite(
     LL_HANDLE *llHdl,
     int32     ch,
     void      *buf,
     int32     size,
     int32     *nbrWrBytesP
)
{
    DBGWRT_1((DBH, "LL - M11_BlockWrite: ch=%d, size=%d\n",ch,size));

	/* return number of written bytes */
	*nbrWrBytesP = 0;

	return(ERR_LL_ILL_FUNC);
}


/****************************** M11_Irq *************************************
 *
 *  Description:  Interrupt service routine
 *
 *                The interrupt is triggered when a timer or Hx interrupt
 *				  occurs.
 *
 *                If the driver can detect the interrupt's cause it returns
 *                LL_IRQ_DEVICE or LL_IRQ_DEV_NOT, otherwise LL_IRQ_UNKNOWN.
 *
 *---------------------------------------------------------------------------
 *  Input......:  llHdl    low-level handle
 *  Output.....:  return   LL_IRQ_DEVICE	irq caused by device
 *                         LL_IRQ_DEV_NOT   irq not caused by device
 *                         LL_IRQ_UNKNOWN   unknown
 *  Globals....:  ---
 ****************************************************************************/
static int32 M11_Irq(
   LL_HANDLE *llHdl
)
{
	MACCESS ma = llHdl->ma;
	u_int8 hxStat, tmStat=0;
	int32 hx, mask;

    IDBGWRT_1((DBH, ">>> M11_Irq: "));

	/*--- determine IRQ sources ---*/
	if ( (hxStat = M230_READ( ma, PS_REG ) & 0xf) != 0 )
		M230_WRITE( ma, PS_REG, hxStat); 			        /* reset H1..H4 irq */

	if( llHdl->tmrIrqEn )
    {
		if( ( tmStat = M230_READ( ma, TS_REG )) & 0x1 )
        {
			M230_WRITE( ma, TS_REG, tmStat); 			    /* reset ZDS */

			/*--- set timer mode to stopped in one shot mode ---*/
			if( llHdl->tmrMode == M_TMR_START_ONE_SHOT )
		    {
				M230_CLRMASK( ma, TC_REG, 0x1 );            /* stop timer */
				llHdl->tmrMode = M_TMR_STOP;
			}
			/*------------------------+
			|  Send signal for timer  |
			+------------------------*/
			if( llHdl->tmrZeroSig )
				OSS_SigSend( llHdl->osHdl, llHdl->tmrZeroSig );
		}
	}
#ifdef  S_LATENCY
    if (llHdl->m99flag)
    {
        register int32 count;
 	    u_int32 	   tval;

		if( ( tmStat = M230_READ( ma, TS_REG )) & 0x1 )
        {
			M230_WRITE( ma, TS_REG, tmStat); 			            /* reset ZDS */

	        /* get elapsed time since timer expired */
	        tval = m99Hdl->timerval - getTime( m99Hdl );

	        if( tval > m99Hdl->maxIrqLatency )
		        m99Hdl->maxIrqLatency = tval;

	        m99Hdl->irqLatency = tval;

            //MWRITE_D16(m99Hdl->maM68230, TS_REG, 0xff);             /* clear interrupt */

            /*------------------+
            | send signal       |
            +------------------*/
            count  = m99Hdl->irqCount & 0x03;                       /* 0..3 counter */

            if( m99Hdl->cond[count] != NULL )   	                /* signal installed ? */
        		OSS_SigSend( m99Hdl->osHdl, m99Hdl->cond[count] );
        }
    }
#endif

    IDBGWRT_1((DBH, "hxStat=%x tmStat=%x\n", hxStat, tmStat));

	/*------------------------------+
	|  Send Signals for HX events   |
	+------------------------------*/
	for( hx=0,mask=0x1; hx<4; hx++,mask<<=1 ){

		if( (hxStat & mask & llHdl->portIrqEn) && llHdl->hxSig[hx]) {
			IDBGWRT_2((DBH, " send sig for H%d\n", hx+1 ));

			OSS_SigSend( llHdl->osHdl, llHdl->hxSig[hx] );
		}
	}

	llHdl->irqCount++;

	return (hxStat || tmStat) ? LL_IRQ_DEVICE : LL_IRQ_UNKNOWN ;
}

/****************************** M11_Info ************************************
 *
 *  Description:  Get information about hardware and driver requirements
 *
 *                The following info codes are supported:
 *
 *                Code                      Description
 *                ------------------------  -----------------------------
 *                LL_INFO_HW_CHARACTER      hardware characteristics
 *                LL_INFO_ADDRSPACE_COUNT   nr of required address spaces
 *                LL_INFO_ADDRSPACE         address space information
 *                LL_INFO_IRQ               interrupt required
 *                LL_INFO_LOCKMODE          process lock mode required
 *
 *                The LL_INFO_HW_CHARACTER code returns all address and
 *                data modes (ORed) which are supported by the hardware
 *                (MDIS_MAxx, MDIS_MDxx).
 *
 *                The LL_INFO_ADDRSPACE_COUNT code returns the number
 *                of address spaces used by the driver.
 *
 *                The LL_INFO_ADDRSPACE code returns information about one
 *                specific address space (MDIS_MAxx, MDIS_MDxx). The returned
 *                data mode represents the widest hardware access used by
 *                the driver.
 *
 *                The LL_INFO_IRQ code returns whether the driver supports an
 *                interrupt routine (TRUE or FALSE).
 *
 *                The LL_INFO_LOCKMODE code returns which process locking
 *                mode the driver needs (LL_LOCK_xxx).
 *
 *---------------------------------------------------------------------------
 *  Input......:  infoType	   info code
 *                ...          argument(s)
 *  Output.....:  return       success (0) or error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 M11_Info(
   int32  infoType,
   ...
)
{
    int32   error = ERR_SUCCESS;
    va_list argptr;

    va_start(argptr, infoType );

    switch(infoType) {
		/*-------------------------------+
        |  hardware characteristics      |
        |  (all addr/data modes ORed)    |
        +-------------------------------*/
        case LL_INFO_HW_CHARACTER:
		{
			u_int32 *addrModeP = va_arg(argptr, u_int32*);
			u_int32 *dataModeP = va_arg(argptr, u_int32*);

			*addrModeP = MDIS_MA08;
			*dataModeP = MDIS_MD08 | MDIS_MD16;
			break;
	    }
		/*-------------------------------+
        |  nr of required address spaces |
        |  (total spaces used)           |
        +-------------------------------*/
        case LL_INFO_ADDRSPACE_COUNT:
		{
			u_int32 *nbrOfAddrSpaceP = va_arg(argptr, u_int32*);

			*nbrOfAddrSpaceP = ADDRSPACE_COUNT;
			break;
	    }
		/*-------------------------------+
        |  address space type            |
        |  (widest used data mode)       |
        +-------------------------------*/
        case LL_INFO_ADDRSPACE:
		{
			u_int32 addrSpaceIndex = va_arg(argptr, u_int32);
			u_int32 *addrModeP = va_arg(argptr, u_int32*);
			u_int32 *dataModeP = va_arg(argptr, u_int32*);
			u_int32 *addrSizeP = va_arg(argptr, u_int32*);

			if (addrSpaceIndex >= ADDRSPACE_COUNT)
				error = ERR_LL_ILL_PARAM;
			else {
				*addrModeP = MDIS_MA08;
				*dataModeP = MDIS_MD16;
				*addrSizeP = ADDRSPACE_SIZE;
			}

			break;
	    }
		/*-------------------------------+
        |   interrupt required           |
        +-------------------------------*/
        case LL_INFO_IRQ:
		{
			u_int32 *useIrqP = va_arg(argptr, u_int32*);

			*useIrqP = USE_IRQ;
			break;
	    }
		/*-------------------------------+
        |   process lock mode            |
        +-------------------------------*/
        case LL_INFO_LOCKMODE:
		{
			u_int32 *lockModeP = va_arg(argptr, u_int32*);

			*lockModeP = LL_LOCK_CALL;
			break;
	    }
		/*-------------------------------+
        |   (unknown)                    |
        +-------------------------------*/
        default:
			error = ERR_LL_ILL_PARAM;
    }

    va_end(argptr);
    return(error);
}

/*******************************  Ident  ************************************
 *
 *  Description:  Return ident string
 *
 *---------------------------------------------------------------------------
 *  Input......:  -
 *  Output.....:  return  pointer to ident string
 *  Globals....:  -
 ****************************************************************************/
static char* Ident( void )	/* nodoc */
{
    return( "M11 - M11 low level driver: $Id: m11_drv.c,v 1.6 2013/09/10 11:11:29 gv Exp $" );
}

/********************************* Cleanup **********************************
 *
 *  Description: Close all handles, free memory and return error code
 *		         NOTE: The low-level handle is invalid after this function is
 *                     called.
 *
 *---------------------------------------------------------------------------
 *  Input......: llHdl		low-level handle
 *               retCode    return value
 *  Output.....: return	    retCode
 *  Globals....: -
 ****************************************************************************/
static int32 Cleanup(
   LL_HANDLE    *llHdl,
   int32        retCode		/* nodoc */
)
{
    /*------------------------------+
    |  close handles                |
    +------------------------------*/
	/* clean up desc */
	if (llHdl->descHdl)
		DESC_Exit(&llHdl->descHdl);

	/* clean up debug */
	DBGEXIT((&DBH));

    /*------------------------------+
    |  free memory                  |
    +------------------------------*/
    /* free my handle */
    OSS_MemFree(llHdl->osHdl, (int8*)llHdl, llHdl->memAlloc);

    /*------------------------------+
    |  return error code            |
    +------------------------------*/
	return(retCode);
}

/********************************* SetGeneralMode ****************************
 *
 *  Description: Set General mode (2x8 or 1x16)
 *
 *	For Mode 2X8:
 *   Global Mode 00, SubMode 1X (single buffer out, unlatched in)
 *
 *  For Mode 1X16:
 *   Global Mode 01,
 *     SubMode 00 if output (Dbl buffered in, single buffered out)
 *     SubMode 01 if input  (Dbl buffered out, non latched in)
 *---------------------------------------------------------------------------
 *  Input......: llHdl		low-level handle
 *				 mode		M11_2X8 | M11_1X16
 *				 dir		direction (0=input, 1=output)
 *							   (only used for M11_1X16)
 *  Output.....: returns	error code
 *  Globals....: -
 ****************************************************************************/
static int32 SetGeneralMode( LL_HANDLE *llHdl, int32 mode, int32 dir ) /* nodoc */
{
	MACCESS ma = llHdl->ma;

	DBGWRT_2((DBH," SetGeneralMode %d dir=%s\n", mode, dir==DIR_IN ?"in":"out"));

	switch( mode ){
	case M11_2X8:
		M230_CLRMASK( ma, PGC_REG, 0xf0 ); /* major mode 0 */
		M230_SETMASK( ma, PGC_REG, 0x30 );

		M230_CLRMASK( ma, PAC_REG, 0xf8 ); /* clear down submodes */
		M230_CLRMASK( ma, PBC_REG, 0xf8 );
		M230_SETMASK( ma, PAC_REG, 0x80 ); /* set submode 1X */
		M230_SETMASK( ma, PBC_REG, 0x80 );
		break;

	case M11_1X16:
		M230_CLRMASK( ma, PGC_REG, 0xf0 ); /* major mode 1 */
		M230_SETMASK( ma, PGC_REG, 0x70 );

		M230_CLRMASK( ma, PAC_REG, 0xf8 ); /* clear down submodes */
		M230_CLRMASK( ma, PBC_REG, 0xf8 );

		if( dir == DIR_IN ){
			M230_SETMASK( ma, PAC_REG, 0x40 ); /* set submode 01 */
			M230_SETMASK( ma, PBC_REG, 0x40 ); /* set submode 01 */
		} /* else submode 00 for output */

		break;

	default:
		return ERR_LL_ILL_PARAM;
	}

	llHdl->portCfg = mode;
	return ERR_SUCCESS;
}

/********************************* SetPortDir ********************************
 *
 *  Description: Setup per-pin data direction for port 0,1,2
 *
 *
 *---------------------------------------------------------------------------
 *  Input......: llHdl		low-level handle
 *				 ch			channel number
 *				 dirMask	direction bits (bit=0 means input)
 *  Output.....: returns	error code
 *  Globals....: -
 ****************************************************************************/
static int32 SetPortDir( LL_HANDLE *llHdl, int32 ch, u_int32 dirMask ) /* nodoc */
{
	MACCESS ma = llHdl->ma;

	DBGWRT_2((DBH," SetPortDir ch%d dirMask=0x%x\n", ch, dirMask ));

	switch( ch ){
	case 0:
		/*--- PORT A ---*/

		/*
		 * In 1X16 mode, all pins must have the same direction
		 */
		if( llHdl->portCfg == M11_1X16 ){
			if( dirMask != 0x0000 && dirMask != 0xffff ){
				DBGWRT_ERR((DBH, "*** M11:SetPortDir: all pins must have "
							" same dir in M11_1X16 mode (value=%x)\n", dirMask));
				return ERR_LL_ILL_DIR;
			}

			llHdl->portABDir = (dirMask == 0xffff) ? DIR_OUT : DIR_IN;

			if( llHdl->portABDir == DIR_IN ){
				M230_WRITE( ma, PADD_REG, 0x00 );
				M230_WRITE( ma, PBDD_REG, 0x00 );
				M230_CLRMASK( ma, PBC_REG, 0xc0 );
				M230_SETMASK( ma, PBC_REG, 0x40 ); /* set submode 01 */
			}
			else {
				M230_WRITE( ma, PADD_REG, 0xff );
				M230_WRITE( ma, PBDD_REG, 0xff );
				M230_CLRMASK( ma, PBC_REG, 0xc0 ); /* set submode 00 */
			}

		}
		else {
			/* 2X8 mode */
			M230_WRITE( ma, PADD_REG, dirMask );
		}
		break;

	case 1:
		/*--- PORT B ---*/
		if( llHdl->portCfg == M11_1X16 ){
			DBGWRT_ERR((DBH, "*** M11:SetPortDir: attempt to use ch1 "
						" in M11_1X16 mode\n"));
			return ERR_LL_ILL_CHAN;
		}
		M230_WRITE( ma, PBDD_REG, dirMask );
		break;

	case 2:
		/*--- PORT C ---*/
		M230_WRITE( ma, PCDD_REG, (M230_READ(ma,PCDD_REG) & 0xfc) | dirMask );
		break;

	case 3:
		/*--- HS1..4 ---*/
		if( dirMask != 0 ){
			DBGWRT_ERR((DBH, "*** M11:SetPortDir: HS1..4 can't be output!\n"));
			return ERR_LL_ILL_DIR;
		}
		break;

	default:
		return ERR_LL_ILL_CHAN;
	}
	return ERR_SUCCESS;
}

/********************************* SetHxIrqEn *********************************
 *
 *  Description: Enable/Disable handshake interrupts
 *
 *
 *---------------------------------------------------------------------------
 *  Input......: llHdl		low-level handle
 *				 hx			pin number (0..3)
 *				 enable		0=disable irq, 1=enable irq
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
static int32 SetHxIrqEn( LL_HANDLE *llHdl, int32 hx, int32 enable )	/* nodoc */
{
	MACCESS ma = llHdl->ma;
	u_int32 offs, mask;

	DBGWRT_2((DBH," SetHxIrqEn H%d en=%d\n", hx+1, enable ));

	switch( hx ){
	case 0:	offs = PAC_REG; mask=0x0002; break;
	case 1:	offs = PAC_REG; mask=0x0004; break;
	case 2:	offs = PBC_REG; mask=0x0002; break;
	case 3:	offs = PBC_REG; mask=0x0004; break;
	default: return ERR_LL_ILL_PARAM;
	}

	if( enable ){

		if( (llHdl->portCfg == M11_1X16) && (hx >= 2) ){
			DBGWRT_ERR((DBH,
						"*** M11:SetHxIrqEn H%d can't be used in 1X16 mode\n",
						hx+1));
			return ERR_LL_ILL_PARAM;
		}

		llHdl->portIrqEn |= (1L<<hx);
		M230_SETMASK( ma, offs, mask );
	}
	else {
		M230_CLRMASK( ma, offs, mask );
		llHdl->portIrqEn &= ~(1L<<hx);
	}
	IrqSetup( llHdl );

	return ERR_SUCCESS;
}

/********************************* IrqSetup *********************************
 *
 *  Description: Sets up the register to enable or disable interrupts
 *
 *
 *---------------------------------------------------------------------------
 *  Input......: llHdl		low-level handle
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
static void IrqSetup( LL_HANDLE *llHdl ) /* nodoc */
{
	int32 portIrqEn = llHdl->portIrqEn ? 1:0;
	int32 tmrIrqEn  = llHdl->tmrIrqEn;
	u_int8 val;

	DBGWRT_2((DBH," IrqSetup glob %d  port 0x%x  tmr %d\n",
			  llHdl->globIrqEn, llHdl->portIrqEn, llHdl->tmrIrqEn));

	if( llHdl->globIrqEn == FALSE )
		portIrqEn = tmrIrqEn = FALSE;

	/*--- setup port C config ---*/
	val = M230_READ( llHdl->ma, PCDD_REG );
	M230_WRITE( llHdl->ma, PCDD_REG,
				(val & 0xd3) | (portIrqEn<<5) | (tmrIrqEn<<3));

	/*--- timer output control ---*/
	val = M230_READ( llHdl->ma, TC_REG );
	M230_WRITE( llHdl->ma, TC_REG, (val & 0x1f) | ((tmrIrqEn ? 7:6) << 5 ));
}

/********************************* GetTimerState ****************************
 *
 *  Description: Determine if timer is (logically) running
 *
 *
 *---------------------------------------------------------------------------
 *  Input......: llHdl		low-level handle
 *  Output.....: returns	timer mode (M_TMR_START_xxx)
 *  Globals....: -
 ****************************************************************************/
static int32 GetTimerState( LL_HANDLE *llHdl ) /* nodoc */
{
	int32 rval;
	OSS_IRQ_STATE irqState;

	irqState = OSS_IrqMaskR( llHdl->osHdl, llHdl->irqHdl );

	/*
     * In one-shot mode, check if timer passed zero,
	 * change timer state to stopped if so
	 */
	if( llHdl->tmrMode == M_TMR_START_ONE_SHOT ){
		if( M230_READ( llHdl->ma, TS_REG ) & 0x1 )
			llHdl->tmrMode = M_TMR_STOP;
	}
	rval = llHdl->tmrMode;

	OSS_IrqRestore( llHdl->osHdl, llHdl->irqHdl, irqState );

	return rval;
}

#ifdef S_LATENCY
/******************************* setTime *************************************
 *
 *  Description:  Load timer value
 *
 *---------------------------------------------------------------------------
 *  Input......:  m99Hdl ll drv handle
 *                timerval  timer value
 *  Output.....:  -
 *  Globals....:  -
 ****************************************************************************/
static void setTime
(
    LL_HANDLE *m11Hdl,
    int32      timerval
)
{
    m11Hdl->timerval = timerval;

    MWRITE_D16( m11Hdl->maM68230, CPL_REG, timerval       & 0xff);
    MWRITE_D16( m11Hdl->maM68230, CPM_REG, timerval >> 8  & 0xff);
    MWRITE_D16( m11Hdl->maM68230, CPH_REG, timerval >> 16 & 0xff);
}

static u_int32 getTime( LL_HANDLE *m11Hdl )
{
	u_int32 low1, low2, mid, high;
	MACCESS ma = m11Hdl->maM68230;

	do {
		low1 = MREAD_D16( ma, CNTL_REG ) & 0xff;
		mid  = MREAD_D16( ma, CNTM_REG ) & 0xff;
		high = MREAD_D16( ma, CNTH_REG ) & 0xff;
		low2 = MREAD_D16( ma, CNTL_REG ) & 0xff;
		/*DBGWRT_3((DBH,"h=%02x m=%02x l1=%02x l2=%02x\n",
		  high, mid, low1, low2 ));*/
	} while( low2 > low1 );

	return ((u_int32)(high<<16)) + ((u_int32)(mid<<8)) + low1;
}
#endif
