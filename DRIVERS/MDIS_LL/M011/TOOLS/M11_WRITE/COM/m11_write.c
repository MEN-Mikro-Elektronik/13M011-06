/****************************************************************************
 ************                                                    ************
 ************                    M11_WRITE                        ************
 ************                                                    ************
 ****************************************************************************
 *
 *       Author: kp
 *
 *  Description: Read all M11 channels and print it out
 *               Doesn't change configuration
 *
 *     Required: libraries: mdis_api, usr_oss, usr_utl
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
#include <stdlib.h>
#include <string.h>
#include <MEN/men_typs.h>
#include <MEN/usr_oss.h>
#include <MEN/usr_utl.h>
#include <MEN/mdis_api.h>
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
static void usage(void);
static void PrintMdisError(char *info);


/********************************* usage ************************************
 *
 *  Description: Print program usage
 *
 *---------------------------------------------------------------------------
 *  Input......: -
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
static void usage(void)
{
	printf("Usage: m11_write [<opts>] <device> [<opts>]\n");
	printf("Function: Configure and write to M011\n");
	printf("  device       device name..................... [none]    \n");
	printf("Options:\n");
	printf("  -A=xx        set data direction for port A    [none]    \n");
	printf("  -B=xx        set data direction for port B    [none]    \n");
	printf("  -C=x         set data direction for port C    [none]    \n");
	printf("  -H=x         set polarity for H4..H1          [none]    \n");
	printf("  -G=2X8|1X16  set port A/B configuration       [none]    \n");
	printf("  -a=xx        write value xx to Port A                   \n");
	printf("  -b=xx        write value xx to Port B                   \n");
	printf("  -c=x         write value x  to Port C                   \n");
	printf("  -h=<pin>     set pin (e.g. -+=A4 set PA4)               \n");
	printf("  -l=<pin>     clear pin (e.g. --=A4 clears PA4)          \n");
	printf("\n");
	printf("Copyright (c) 1999-2019, MEN Mikro Elektronik GmbH\n%s\n", IdentString);
}

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
	MDIS_PATH	path=0;
	int32		chan,value,n,mask;
	char		*device,*errstr,buf[40], *str;

	/*--------------------+
    |  check arguments    |
    +--------------------*/
	if ((errstr = UTL_ILLIOPT("A=B=C=H=G=a=b=c=h=l=?", buf))) {
		printf("*** %s\n", errstr);
		return(1);
	}

	if (UTL_TSTOPT("?")) {						/* help requested ? */
		usage();
		return(1);
	}

	/*--------------------+
    |  get arguments      |
    +--------------------*/
	for (device=NULL, n=1; n<argc; n++)
		if (*argv[n] != '-') {
			device = argv[n];
			break;
		}

	if (!device) {
		usage();
		return(1);
	}

	/*--------------------+
    |  open path          |
    +--------------------*/
	if ((path = M_open(device)) < 0) {
		PrintMdisError("open");
		return(1);
	}

	/*-------------------+
	|  Configure         |
	+-------------------*/

	/*--- port A/B ---*/
	if( (str = UTL_TSTOPT("G="))){
		int32 mode;

		if( strcmp( str, "2X8") == 0 )
			mode = M11_2X8;
		else if( strcmp( str, "1X16") == 0 )
			mode = M11_1X16;
		else {
			fprintf(stderr,"Bad mode for -G\n");
			goto abort;
		}
		printf("Changing port A/B mode to %s\n", str );

		if( M_setstat( path, M11_PORTCFG, mode )){
			PrintMdisError("setstat M11_PORTCFG");
			goto abort;
		}
	}

	/*--- port A/B/C direction ---*/
	if( (str = UTL_TSTOPT("A="))){
		value = strtol(str, &errstr, 16);
		/*sscanf( str, "%lx", &value );*/

		printf("Setting port A direction to 0x%02x\n", value );

		M_setstat( path, M_MK_CH_CURRENT, 0 );
		if( M_setstat( path, M11_PINDIR, value )){
			PrintMdisError("setstat M11_PINDIR Port A");
			goto abort;
		}
	}

	if( (str = UTL_TSTOPT("B="))){
		value = strtol(str, &errstr, 16);
		/*sscanf( str, "%lx", &value );*/

		printf("Setting port B direction to 0x%02x\n", value );

		M_setstat( path, M_MK_CH_CURRENT, 1 );
		if( M_setstat( path, M11_PINDIR, value )){
			PrintMdisError("setstat M11_PINDIR Port B");
			goto abort;
		}
	}

	if( (str = UTL_TSTOPT("C="))){
		value = strtol(str, &errstr, 16);
		/*sscanf( str, "%lx", &value );*/

		printf("Setting port C direction to 0x%02x\n", value );

		M_setstat( path, M_MK_CH_CURRENT, 2 );
		if( M_setstat( path, M11_PINDIR, value )){
			PrintMdisError("setstat M11_PINDIR Port C");
			goto abort;
		}
	}

	/*--- HX Sense ---*/
	if( (str = UTL_TSTOPT("H="))){
		value = strtol(str, &errstr, 16);
		/*sscanf( str, "%lx", &value );*/

		printf("Setting HX sense to 0x%02x\n", value );

		if( M_setstat( path, M11_HXSENSE, value )){
			PrintMdisError("setstat M11_HXSENSE");
			goto abort;
		}
	}

	/*----------------------+
	|  Write/Change values  |
	+----------------------*/
	if( (str = UTL_TSTOPT("a="))){
		value = strtol(str, &errstr, 16);
		/*sscanf( str, "%lx", &value );*/

		printf("Writing 0x%02x to port A\n", value );

		M_setstat( path, M_MK_CH_CURRENT, 0 );
		if( M_write( path, value )){
			PrintMdisError("write port A");
			goto abort;
		}
	}

	if( (str = UTL_TSTOPT("b="))){
		value = strtol(str, &errstr, 16);
		/*sscanf( str, "%lx", &value );*/

		printf("Writing 0x%02x to port B\n", value );

		M_setstat( path, M_MK_CH_CURRENT, 1 );
		if( M_write( path, value )){
			PrintMdisError("write port B");
			goto abort;
		}
	}

	if( (str = UTL_TSTOPT("c="))){
		value = strtol(str, &errstr, 16);
		/*sscanf( str, "%lx", &value );*/

		printf("Writing 0x%02x to port C\n", value );

		M_setstat( path, M_MK_CH_CURRENT, 2 );
		if( M_write( path, value )){
			PrintMdisError("write port C");
			goto abort;
		}
	}

	if( (str = UTL_TSTOPT("h="))){

		switch(str[0]){
		case 'A': chan=0; break;
		case 'B': chan=1; break;
		case 'C': chan=2; break;
		default:
			fprintf(stderr,"Bad format %s for option\n", str );
			goto abort;
		}
		value = atoi( str+1 );
		mask = 1L<<value;

		printf("Setmask 0x%02x on port %c\n", mask, 'A'+chan );

		M_setstat( path, M_MK_CH_CURRENT, chan );
		if( M_setstat( path, M11_SETPINS, mask )){
			PrintMdisError("setstat M11_SETPINS");
			goto abort;
		}
	}

	if( (str = UTL_TSTOPT("l="))){

		switch(str[0]){
		case 'A': chan=0; break;
		case 'B': chan=1; break;
		case 'C': chan=2; break;
		default:
			fprintf(stderr,"Bad format %s for option\n", str );
			goto abort;
		}

		value = atoi( str+1 );
		mask = 1L<<value;

		printf("Clearmask 0x%02x on port %c\n", mask, 'A'+chan );

		M_setstat( path, M_MK_CH_CURRENT, chan );
		if( M_setstat( path, M11_CLRPINS, mask )){
			PrintMdisError("setstat M11_CLRPINS");
			goto abort;
		}
	}

	/*--------------------+
    |  cleanup            |
    +--------------------*/
	abort:
	if (M_close(path) < 0)
		PrintMdisError("close");

	return(0);
}

/********************************* PrintMdisError ***************************
 *
 *  Description: Print MDIS error message
 *
 *---------------------------------------------------------------------------
 *  Input......: info	info string
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
static void PrintMdisError(char *info)
{
	printf("*** Error while %s: %s\n", info, M_errstring(UOS_ErrnoGet()));
}
