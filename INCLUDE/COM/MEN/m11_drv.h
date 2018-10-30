/***********************  I n c l u d e  -  F i l e  ************************
 *
 *         Name: m11_drv.h
 *
 *       Author: kp
 *        $Date: 2013/09/10 11:11:44 $
 *    $Revision: 2.5 $
 *
 *  Description: Header file for M11 driver
 *               - M11 specific status codes
 *               - M11 function prototypes
 *
 *     Switches: _ONE_NAMESPACE_PER_DRIVER_
 *               _LL_DRV_
 *               S_LATENCY
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 1998 by MEN mikro elektronik GmbH, Nuernberg, Germany
 ****************************************************************************/
/*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _M11_DRV_H
#define _M11_DRV_H

#ifdef __cplusplus
      extern "C" {
#endif

/*-----------------------------------------+
|  TYPEDEFS                                |
+-----------------------------------------*/
/* none */

/*-----------------------------------------+
|  DEFINES                                 |
+-----------------------------------------*/

/* M11 specific status codes (STD) */        /* S,G: S=setstat, G=getstat */

/* codes M_DEV_OF+0x00 .. M_DEV_OF+0x0f reserved for M_TMR codes */

#define M11_PORTCFG         (M_DEV_OF+0x10)    /* S,G: setup 2x8 or 1x16 configuration 	*/
#define M11_PINDIR         	(M_DEV_OF+0x11)    /* S,G: Pin specific data direction 		*/
#define M11_SETPINS			(M_DEV_OF+0x12)	   /* S  : OR value into output latch 		*/
#define M11_CLRPINS			(M_DEV_OF+0x13)    /* S  : AND negated value with outputs   */
#define M11_HXSENSE			(M_DEV_OF+0x14)	   /* S,G: define H1..H4 polarity			*/
#define M11_SIGSET_H1		(M_DEV_OF+0x20)	   /* S,G: enable/install sig for H1 irq	*/
#define M11_SIGSET_H2		(M_DEV_OF+0x21)	   /* S,G: enable/install sig for H2 irq	*/
#define M11_SIGSET_H3		(M_DEV_OF+0x22)	   /* S,G: enable/install sig for H3 irq	*/
#define M11_SIGSET_H4		(M_DEV_OF+0x23)	   /* S,G: enable/install sig for H4 irq	*/
#define M11_SIGCLR_H1		(M_DEV_OF+0x28)	   /* S  : remove sig for H1 irq			*/
#define M11_SIGCLR_H2		(M_DEV_OF+0x29)	   /* S  : remove sig for H1 irq			*/
#define M11_SIGCLR_H3		(M_DEV_OF+0x2a)	   /* S  : remove sig for H1 irq			*/
#define M11_SIGCLR_H4		(M_DEV_OF+0x2b)	   /* S  : remove sig for H1 irq			*/

/* cloned from m99, for internal test purposes (M99 replacement) */
#ifdef  S_LATENCY
# define M99_TIMERVAL        (M_DEV_OF+0x1)    /* G,S: timer value */
# define M99_JITTER          (M_DEV_OF+0x2)    /* G,S: jitter mode enable */            //not supportted
# define M99_IRQCOUNT        (M_DEV_OF+0x3)    /* G,S: irq counter */                //not supportted
# define M99_SIG_set_cond1   (M_DEV_OF+0x4)    /* G,S: signal */
# define M99_SIG_set_cond2   (M_DEV_OF+0x5)    /* G,S: signal */
# define M99_SIG_set_cond3   (M_DEV_OF+0x6)    /* G,S: signal */
# define M99_SIG_set_cond4   (M_DEV_OF+0x7)    /* G,S: signal */
# define M99_SIG_clr_cond1   (M_DEV_OF+0x8)    /*   S: signal */
# define M99_SIG_clr_cond2   (M_DEV_OF+0x9)    /*   S: signal */
# define M99_SIG_clr_cond3   (M_DEV_OF+0xa)    /*   S: signal */
# define M99_SIG_clr_cond4   (M_DEV_OF+0xb)    /*   S: signal */
# define M99_GET_TIME	     (M_DEV_OF+0xc)	   /* G  : get elapsed counter value */
# define M99_MAX_IRQ_LAT	 (M_DEV_OF+0xd)	   /* G,S: max irq latency ticks */        ///not supportted
# define M99_IRQ_LAT	  	 (M_DEV_OF+0xe)	   /* G  : last irq latency ticks */

# define M11_TIMERVAL        M99_TIMERVAL
# define M11_GET_TIME	     M99_GET_TIME
# define M11_IRQ_LAT	     M99_IRQ_LAT
# define M11_SIG_set_cond1   M99_SIG_set_cond1
# define M11_SIG_set_cond2   M99_SIG_set_cond2
# define M11_SIG_set_cond3   M99_SIG_set_cond3
# define M11_SIG_set_cond4   M99_SIG_set_cond4
# define M11_SIG_clr_cond1   M99_SIG_clr_cond1
# define M11_SIG_clr_cond2   M99_SIG_clr_cond2
# define M11_SIG_clr_cond3   M99_SIG_clr_cond3
# define M11_SIG_clr_cond4   M99_SIG_clr_cond4

#endif /* S_LATENCY */

/* M11_PORTCFG values */
#define M11_2X8				0				   /* port A/B operate independent */
#define M11_1X16			1				   /* port A/B form a 16 bit register */

/*-----------------------------------------+
|  PROTOTYPES                              |
+-----------------------------------------*/
#ifdef _LL_DRV_

#ifndef _ONE_NAMESPACE_PER_DRIVER_

#	ifdef ID_SW
#		define M11_GetEntry M11_SW_GetEntry
#	endif

	extern void M11_GetEntry(LL_ENTRY* drvP);
//#else
//	extern void LL_GetEntry(LL_ENTRY* drvP);
#endif

#endif /* _LL_DRV_ */

/*-----------------------------------------+
|  BACKWARD COMPATIBILITY TO MDIS4         |
+-----------------------------------------*/
#ifndef U_INT32_OR_64
 /* we have an MDIS4 men_types.h and mdis_api.h included */
 /* only 32bit compatibility needed!                     */
 #define INT32_OR_64  int32
 #define U_INT32_OR_64 u_int32
 typedef INT32_OR_64  MDIS_PATH;
#endif /* U_INT32_OR_64 */

#ifdef __cplusplus
      }
#endif

#endif /* _M11_DRV_H */
