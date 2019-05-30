/***********************  I n c l u d e  -  F i l e  ************************
 *
 *         Name: m_tmr_drv.h
 *
 *       Author: kp
 *
 *  Description: Header file for MDIS4 LL driver implementing M_TMR profile
 *               
 *     Switches: ---
 *
 *---------------------------------------------------------------------------
 * Copyright (c) 1999-2019, MEN Mikro Elektronik GmbH
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

#ifndef _M_TMR_DRV_H
#define _M_TMR_DRV_H

#ifdef __cplusplus
      extern "C" {
#endif


/*-----------------------------------------+
|  DEFINES                                 |
+-----------------------------------------*/
/* profile type (returned in M_LL_CH_TYP */
#define M_CH_PROFILE_TMR	100


/* M_TMR specific status codes (STD) */			/* S,G: S=setstat, G=getstat			*/
#define M_TMR_RESOLUTION	(M_DEV_OF+0xe0)		/*   G: Decrements per second			*/
#define M_TMR_RUN			(M_DEV_OF+0xe1)		/* S,G: Start/Stop timer				*/
#define M_TMR_SIGSET_ZERO	(M_DEV_OF+0xe2)		/* G,S: Send signal when zero reached	*/
#define M_TMR_SIGCLR_ZERO	(M_DEV_OF+0xe3)		/*   S: Clear above signal 				*/

/* M_TMR_RUN values */
#define M_TMR_STOP					0
#define M_TMR_START_ONE_SHOT		1
#define M_TMR_START_FREE_RUNNING	2

#ifdef __cplusplus
      }
#endif

#endif /* M_TMR_DRV_H */

