/****************************************************************************
 ************                                                    ************
 ************                   M11_SIMP                        ************
 ************                                                    ************
 ****************************************************************************
 *
 *       Author: kp
 *        $Date: 2013/09/10 11:11:37 $
 *    $Revision: 1.2 $
 *
 *  Description: Simple example program for the M11 driver
 *
 *               Defines port A/B as inputs and read pins of port A/B
 *
 *     Required: libraries: mdis_api, usr_oss
 *     Switches: -
 *
 *-------------------------------[ History ]---------------------------------
 *
 * $Log: m11_simp.c,v $
 * Revision 1.2  2013/09/10 11:11:37  gv
 * R: Porting to MDIS5
 * M: Changed according to MDIS Porting Guide 0.9
 *
 * Revision 1.1  1999/11/03 15:40:49  kp
 * Initial Revision
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 1999 by MEN mikro elektronik GmbH, Nuernberg, Germany
 ****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <MEN/men_typs.h>
#include <MEN/mdis_api.h>
#include <MEN/usr_oss.h>
#include <MEN/m11_drv.h>

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
