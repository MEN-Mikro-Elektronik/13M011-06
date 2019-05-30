/****************************************************************************
 ************                                                    ************
 ************                    M11_READ                        ************
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
static int32 G_sigCnt;

/*--------------------------------------+
|   PROTOTYPES                          |
+--------------------------------------*/
static void usage(void);
static void PrintMdisError(char *info);
static void PrintUosError(char *info);
static char *Bin2Str( u_int32 value, int n );


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
	printf("Usage: m11_read [<opts>] <device> [<opts>]\n");
	printf("Function: Read all M011 channels and configurations\n");
	printf("          Installs signals for Hx events\n");
	printf("  device       device name..................... [none]    \n");
	printf("Options:\n");
	printf("  -l           loop mode....................... [no]      \n");
	printf("\n");
	printf("Copyright (c) 1999-2019, MEN Mikro Elektronik GmbH\n%s\n", IdentString);
}

/********************************* SigHandler ********************************
 *
 *  Description: Handle Signals
 *
 *
 *---------------------------------------------------------------------------
 *  Input......: sigCode		signal code
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
static void __MAPILIB SigHandler( u_int32 sigCode )
{
	G_sigCnt++;
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
	int32		mode,loopmode,value,n;
	char		*device,*errstr,buf[40];

	/*--------------------+
    |  check arguments    |
    +--------------------*/
	if ((errstr = UTL_ILLIOPT("l?", buf))) {	/* check args */
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

	G_sigCnt = 0;
	loopmode = (UTL_TSTOPT("l") ? 1 : 0);

	/*--------------------+
    |  open path          |
    +--------------------*/
	if ((path = M_open(device)) < 0) {
		PrintMdisError("open");
		return(1);
	}


	/*--------------------+
	|  Read configuration |
	+--------------------*/

	/* get PORT A/B mode */
	if( M_getstat( path, M11_PORTCFG, &mode )){
		PrintMdisError("setstat M11_PORTCFG");
		goto abort;
	}

	printf("Port A/B mode: %s\n", mode==M11_2X8 ? "2x8" : "1x16" );

	/* get data direction */
	if( M_getstat( path, M11_PINDIR, &value )){
		PrintMdisError("setstat M11_PORTCFG");
		goto abort;
	}

	if( mode == M11_2X8 ){
		printf("Port A DDR: %s\n", Bin2Str(value,8) );

		M_setstat(path, M_MK_CH_CURRENT, 1);
		M_getstat( path, M11_PINDIR, &value );
		printf("Port B DDR: %s\n", Bin2Str(value,8) );
	}
	else {
		printf("Port A DDR: %s\n", Bin2Str(value,16 ));
	}

	M_setstat(path, M_MK_CH_CURRENT, 2);
	M_getstat( path, M11_PINDIR, &value );
	printf("Port C DDR: %s\n", Bin2Str(value,2) );

	/* get HX Sense */
	M_getstat( path, M11_HXSENSE, &value );
	printf("HX Sense  : %s\n", Bin2Str(value,4) );

	/*--------------------------------+
	|  Install Signals for Hx events  |
	+--------------------------------*/
	if( UOS_SigInit( SigHandler ) < 0 ){
		PrintUosError("UOS_SigInit");
		goto abort;
	}

	if( UOS_SigInstall( UOS_SIG_USR1 ) < 0 ){
		PrintUosError("UOS_SigInstall");
		goto abort;
	}

	for( n=0; n<4; n++ ){
		if( mode == M11_1X16 && n>=2 )
			break;

		if( M_setstat( path, M11_SIGSET_H1+n, UOS_SIG_USR1 )){
			PrintMdisError("setstat M11_SIGSET_Hx");
			goto abort;
		}
	}
	/*--- enable global irqs ---*/
	M_setstat( path, M_MK_IRQ_ENABLE, TRUE );

	printf("\n");

	if( mode==M11_2X8 ){
		printf("A=7654 3210 B=7654 3210 C=10 Hx=4321             \n");
		printf("----------- ----------- ---- ------- ------------\n");
	}
	else {
		printf("A=FEDC BA98 7654 3210 C=10 Hx=4321             \n");
		printf("--------------------- ---- ------- ------------\n");
	}
	/*--------------------+
	|  Read all channels  |
	+--------------------*/
	do {
		M_setstat(path, M_MK_CH_CURRENT, 0);

		if( M_read( path, &value ) < 0){
			PrintMdisError("M_read Port A");
			goto abort;
		}
		printf("A=%s ", Bin2Str(value,mode==M11_2X8 ? 8:16));

		if( mode==M11_2X8 ){
			M_setstat(path, M_MK_CH_CURRENT, 1);

			if( M_read( path, &value ) < 0){
				PrintMdisError("M_read Port B");
				goto abort;
			}
			printf("B=%s ", Bin2Str(value,8));
		}

		M_setstat(path, M_MK_CH_CURRENT, 2);

		if( M_read( path, &value ) < 0){
			PrintMdisError("M_read Port C");
			goto abort;
		}
		printf("C=%s ", Bin2Str(value,2));

		M_setstat(path, M_MK_CH_CURRENT, 3);

		if( M_read( path, &value ) < 0){
			PrintMdisError("M_read Port H");
			goto abort;
		}
		printf("Hx=%s ", Bin2Str(value,4));

		M_setstat(path, M_MK_CH_CURRENT, 4);

		if( M_read( path, &value ) < 0){
			PrintMdisError("M_read TMR");
			goto abort;
		}
		printf("TMR=0x%06x ", value);


		/*--- show number of signals occurred ---*/
		UOS_SigMask();
		n = G_sigCnt;
		G_sigCnt = 0;
		UOS_SigUnMask();

		if( n )
			printf("%d Hx EVENTS!", n);

		printf("\n");

		UOS_Delay(100);
	} while(loopmode && UOS_KeyPressed() == -1);

	/*--------------------+
    |  cleanup            |
    +--------------------*/
	abort:
	/*--- disable global irqs ---*/
	M_setstat( path, M_MK_IRQ_ENABLE, FALSE );

	for( n=0; n<4; n++ ){
		if( mode == M11_1X16 && n>=2 )
			break;
		if( M_setstat( path, M11_SIGCLR_H1+n, 0)){
			PrintMdisError("setstat M11_SIGCLR_Hx");
		}
	}

	if( UOS_SigRemove( UOS_SIG_USR1 ) < 0 ){
		PrintUosError("UOS_SigInstall");
	}

	if( UOS_SigExit() < 0 ){
		PrintUosError("UOS_SigExit");
	}


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
	printf("*** can't %s: %s\n", info, M_errstring(UOS_ErrnoGet()));
}

/********************************* PrintUosError ****************************
 *
 *  Description: Print UOS error message
 *
 *---------------------------------------------------------------------------
 *  Input......: info	info string
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
static void PrintUosError(char *info)
{
	printf("*** can't %s: %s\n", info, UOS_ErrString(UOS_ErrnoGet()));
}

static char *Bin2Str( u_int32 value, int n )
{
	static char str[50];
	int i, idx=0;

	for(i=n-1;i>=0;i--){
		if( !((i+1)%4) && (i!=n-1)) str[idx++] = ' ';
		str[idx++] = (value & (1<<i)) ? '1': '0';
	}
	str[idx] = '\0';
	return str;
}

