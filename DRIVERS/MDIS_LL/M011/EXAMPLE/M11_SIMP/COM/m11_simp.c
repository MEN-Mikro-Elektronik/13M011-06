/****************************************************************************
 ************                                                    ************
 ************                   M11_SIMP                        ************
 ************                                                    ************
 ****************************************************************************
 *
 *       Author: kp
 *
 *  Description: Simple example program for the M11 driver
 *
 *               Defines port A/B as inputs and read pins of port A/B
 *
 *     Required: libraries: mdis_api, usr_oss
 *     Switches: -
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

#include <stdio.h>
#include <string.h>
#include <MEN/men_typs.h>
#include <MEN/mdis_api.h>
#include <MEN/usr_oss.h>
#include <MEN/m11_drv.h>

static const char IdentString[]=MENT_XSTR(MAK_REVISION);

/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/
/* none */

/*--------------------------------------+
|   TYPDEFS                             |
+--------------------------------------*/
/* none */

/*--------------------------------------+
|   EXTERNALS                           |
+--------------------------------------*/
/* none */

/*--------------------------------------+
|   GLOBALS                             |
+--------------------------------------*/
/* none */

/*--------------------------------------+
|   PROTOTYPES                          |
+--------------------------------------*/
static void PrintError(char *info);


/********************************* main *************************************
 *
 *  Description: Program main function
 *
 *---------------------------------------------------------------------------
 *  Input......: argc,argv	argument counter, data ..
 *  Output.....: return	    success (0) or error (1)
 *  Globals....: -
 ****************************************************************************/
int main(int argc, char *argv[])
{
	MDIS_PATH	path;
	int32		value;
	char		*device;

	if (argc < 2 || strcmp(argv[1],"-?")==0) {
		printf("Syntax: m11_simp <device> <chan>\n");
		printf("Function: M11 example for port input\n");
		printf("Options:\n");
		printf("    device       device name\n");
		printf("\n");
		return(1);
	}

	device = argv[1];

	/*--------------------+
    |  open path          |
    +--------------------*/
	if ((path = M_open(device)) < 0) {
		PrintError("open");
		return(1);
	}

	/*--------------------+
    |  config             |
    +--------------------*/
	/* channel number */
	if ((M_setstat(path, M_MK_CH_CURRENT, 0)) < 0) {
		PrintError("setstat M_MK_CH_CURRENT");
		goto abort;
	}

	/* set port A to input */
	if ((M_setstat(path, M11_PINDIR, 0x00)) < 0) {
		PrintError("setstat M11_PINDIR");
		goto abort;
	}

	/* channel number */
	if ((M_setstat(path, M_MK_CH_CURRENT, 1)) < 0) {
		PrintError("setstat M_MK_CH_CURRENT");
		goto abort;
	}

	/* set port B to input */
	if ((M_setstat(path, M11_PINDIR, 0x00)) < 0) {
		PrintError("setstat M11_PINDIR");
		goto abort;
	}


	/*--------------------+
    |  Read               |
    +--------------------*/

	/* channel number */
	if ((M_setstat(path, M_MK_CH_CURRENT, 0)) < 0) {
		PrintError("setstat M_MK_CH_CURRENT");
		goto abort;
	}

	if( M_read( path, &value )){
		PrintError("read Port A");
		goto abort;
	}

	printf("Port A: 0x%02x\n", value );

	/* channel number */
	if ((M_setstat(path, M_MK_CH_CURRENT, 1)) < 0) {
		PrintError("setstat M_MK_CH_CURRENT");
		goto abort;
	}

	if( M_read( path, &value )){
		PrintError("read Port B");
		goto abort;
	}

	printf("Port B: 0x%02x\n", value );

	/*--------------------+
    |  cleanup            |
    +--------------------*/
	abort:
	if (M_close(path) < 0)
		PrintError("close");

	return(0);
}

/********************************* PrintError *******************************
 *
 *  Description: Print MDIS error message
 *
 *---------------------------------------------------------------------------
 *  Input......: info	info string
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
static void PrintError(char *info)
{
	printf("*** can't %s: %s\n", info, M_errstring(UOS_ErrnoGet()));
}
