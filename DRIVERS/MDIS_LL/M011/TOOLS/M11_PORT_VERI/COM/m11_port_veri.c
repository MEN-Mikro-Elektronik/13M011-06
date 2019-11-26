/****************************************************************************
 ************                                                    ************
 ************                    M11_PORT_VERI                   ************
 ************                                                    ************
 ****************************************************************************
 *
 *       Author: kp
 *
 *  Description: Verify M11 driver port functionality
 *               Requires test connector with fixed cabling
 *               (all connections made with a 100 ohms resistor)
 *
 *  			PA0 - H1
 *  			PA1 - H2
 *  			PA2 - H3
 *  			PA3 - H4
 *              PB4 - PC0
 *              PB5 - PC1
 *
 *     Required: libraries: mdis_api, usr_oss, usr_utl
 *     Switches: -
 *
 *---------------------------------------------------------------------------
 * Copyright 1999-2019, MEN Mikro Elektronik GmbH
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
	printf("Usage: m11_port_veri [<opts>] <device> [<opts>]\n");
	printf("Function: Verify M11 driver port functionality\n");
	printf("          Requires test connector with fixed cabling\n");
	printf("  device       device name..................... [none]    \n");
	printf("Options:\n");
	printf("\n");
	printf("Copyright 1999-2019, MEN Mikro Elektronik GmbH\n%s\n", IdentString);
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
	int32		value,n;
	char		*device,*errstr,buf[40];

	/*--------------------+
    |  check arguments    |
    +--------------------*/
	if ((errstr = UTL_ILLIOPT("?", buf))) {	/* check args */
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

	/*--------------------+
    |  open path          |
    +--------------------*/
	if ((path = M_open(device)) < 0) {
		PrintMdisError("open");
		return(1);
	}

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

	/*--- enable global irqs ---*/
	M_setstat( path, M_MK_IRQ_ENABLE, TRUE );

	/*--------------------+
	|  Verify 2x8 mode    |
	+--------------------*/

	if( M_setstat( path, M11_PORTCFG, M11_2X8 )){
		PrintMdisError("setstat M11_PORTCFG");
		goto abort;
	}

	printf("Testing 2X8 mode\n");
	printf(" Testing PortC -> PortB\n");

	/*------ set DDR PC0,1=out set DDR PC4,5=in ------*/
	M_setstat(path, M_MK_CH_CURRENT, 2);

	if( M_setstat( path, M11_PINDIR, 0x03 )){
		PrintMdisError("setstat M11_PINDIR C");
		goto abort;
	}

	M_setstat(path, M_MK_CH_CURRENT, 1);

	if( M_setstat( path, M11_PINDIR, 0x00 )){
		PrintMdisError("setstat M11_PINDIR B");
		goto abort;
	}

	/* Write to PC0,1=01 */

	M_setstat(path, M_MK_CH_CURRENT, 2);
	M_write( path, 0x2 );
	UOS_Delay(10);

	M_setstat(path, M_MK_CH_CURRENT, 1);
	M_read( path, &value );
	if( (value&0x30) != 0x20 ){
		printf("*** Verify error #1: port B is 0x%02x should be 0x20\n",
			   (value&0x30) );
		goto abort;
	}


	/* Write to PC0,1=10 */

	M_setstat(path, M_MK_CH_CURRENT, 2);
	M_write( path, 0x1 );
	UOS_Delay(10);

	M_setstat(path, M_MK_CH_CURRENT, 1);
	M_read( path, &value );
	if( (value&0x30) != 0x10 ){
		printf("*** Verify error #2: port B is 0x%02x should be 0x10\n",
			   (value&0x30) );
		goto abort;
	}

	/*------ set DDR PB4=in PB5=out set DDR PC0=out, PC1=in ------*/
	printf(" Testing PortB <-> PortC using SETPINS/CLRPINS\n");

	M_setstat(path, M_MK_CH_CURRENT, 1);

	if( M_setstat( path, M11_PINDIR, 0x20 )){
		PrintMdisError("setstat M11_PINDIR B");
		goto abort;
	}

	M_setstat(path, M_MK_CH_CURRENT, 2);

	if( M_setstat( path, M11_PINDIR, 0x01 )){
		PrintMdisError("setstat M11_PINDIR C");
		goto abort;
	}

	/* Write to PB5=1, PC0=0 */

	M_setstat(path, M_MK_CH_CURRENT, 1);
	M_setstat(path, M11_SETPINS, 0x20 );

	M_setstat(path, M_MK_CH_CURRENT, 2);
	M_setstat(path, M11_CLRPINS, 0x01 );

	UOS_Delay(10);

	M_read( path, &value );		/* read Port C */
	if( (value&0x3) != 0x2 ){
		printf("*** Verify error #3: port C is 0x%02x should be 0x2\n",
			   (value&0x3) );
		goto abort;
	}

	M_setstat(path, M_MK_CH_CURRENT, 1);
	M_read( path, &value );		/* read Port B */
	if( (value&0x30) != 0x20 ){
		printf("*** Verify error #4: port B is 0x%02x should be 0x20\n",
			   (value&0x30) );
		goto abort;
	}

	/*------ set DDR PA0..3=out  ------*/
	printf(" Testing PortA -> PortH\n");
	M_setstat(path, M_MK_CH_CURRENT, 0);
	M_setstat( path, M11_PINDIR, 0x0f );

	M_setstat(path, M_MK_CH_CURRENT, 0);
	M_write( path, 0xf );
	UOS_Delay(10);
	M_setstat(path, M_MK_CH_CURRENT, 3);
	M_read( path, &value );		/* port H */

	if( value != 0xf ){
		printf("*** Verify error #5: port H is 0x%02x should be 0xf\n",
			   value );
		goto abort;
	}

	M_setstat(path, M_MK_CH_CURRENT, 0);
	M_write( path, 0x0 );
	UOS_Delay(10);
	M_setstat(path, M_MK_CH_CURRENT, 3);
	M_read( path, &value );		/* port H */

	if( value != 0x0 ){
		printf("*** Verify error #6: port H is 0x%02x should be 0x0\n",
			   value );
		goto abort;
	}

	M_setstat(path, M_MK_CH_CURRENT, 0);
	M_write( path, 0xa );
	UOS_Delay(10);
	M_setstat(path, M_MK_CH_CURRENT, 3);
	M_read( path, &value );		/* port H */

	if( value != 0xa ){
		printf("*** Verify error #7: port H is 0x%02x should be 0xa\n",
			   value );
		goto abort;
	}


	/*--- Test Hx signals ---*/
	printf(" Testing PortH signals\n");

	M_setstat(path, M_MK_CH_CURRENT, 0);
	M_write( path, 0xf );

	for( n=0; n<4; n++ ){
		if( M_setstat( path, M11_SIGSET_H1+n, UOS_SIG_USR1 )){
			PrintMdisError("setstat M11_SIGSET_Hx");
			goto abort;
		}
	}


	for( n=0; n<4; n++ ){
		G_sigCnt = 0;

		M_setstat( path, M11_CLRPINS,(INT32_OR_64)(1<<n) ); /* clear corr. pin */
		UOS_Delay(10);

		if( G_sigCnt != 1 ){
			printf("*** No signal received for H%d\n", n+1 );
			goto abort;
		}
	}

	for( n=0; n<4; n++ ){
		if( M_setstat( path, M11_SIGCLR_H1+n, 0)){
			PrintMdisError("setstat M11_SIGCLR_Hx");
		}
	}

	/*--------------------+
	|  Verify 1x16 mode   |
	+--------------------*/
	printf("Testing 1X16 mode\n");
	if( M_setstat( path, M11_PORTCFG, M11_1X16 )){
		PrintMdisError("setstat M11_PORTCFG");
		goto abort;
	}

	printf(" Testing PortC -> PortB\n");

	/*------ set DDR PC0,1=out set DDR PA=in ------*/
	M_setstat(path, M_MK_CH_CURRENT, 2);

	if( M_setstat( path, M11_PINDIR, 0x03 )){
		PrintMdisError("setstat M11_PINDIR C");
		goto abort;
	}

	M_setstat(path, M_MK_CH_CURRENT, 0);

	if( M_setstat( path, M11_PINDIR, 0x0000 )){
		PrintMdisError("setstat M11_PINDIR A");
		goto abort;
	}

	/* Write to PC0,1=01 */

	M_setstat(path, M_MK_CH_CURRENT, 2);
	M_write( path, 0x2 );
	UOS_Delay(10);

	M_setstat(path, M_MK_CH_CURRENT, 0);
	M_read( path, &value );
	if( (value&0x0030) != 0x0020 ){
		printf("*** Verify error #10: port A is 0x%02x should be 0x20\n",
			   (value&0x30) );
		goto abort;
	}


	/* Write to PC0,1=10 */

	M_setstat(path, M_MK_CH_CURRENT, 2);
	M_write( path, 0x1 );
	UOS_Delay(10);

	M_setstat(path, M_MK_CH_CURRENT, 0);
	M_read( path, &value );
	if( (value&0x30) != 0x10 ){
		printf("*** Verify error #11: port A is 0x%02x should be 0x10\n",
			   (value&0x30) );
		goto abort;
	}

	/*------ set DDR PA=out ------*/

	printf(" Testing PortA -> PortH\n");
	M_setstat(path, M_MK_CH_CURRENT, 0);
	M_setstat( path, M11_PINDIR, 0xFFFF );

	M_setstat(path, M_MK_CH_CURRENT, 0);
	M_write( path, 0x0f00 );
	UOS_Delay(10);
	M_setstat(path, M_MK_CH_CURRENT, 3);
	M_read( path, &value );		/* port H */

	if( value != 0xf ){
		printf("*** Verify error #5: port H is 0x%02x should be 0xf\n",
			   value );
		goto abort;
	}

	M_setstat(path, M_MK_CH_CURRENT, 0);
	M_write( path, 0x0000 );
	UOS_Delay(10);
	M_setstat(path, M_MK_CH_CURRENT, 3);
	M_read( path, &value );		/* port H */

	if( value != 0x0 ){
		printf("*** Verify error #6: port H is 0x%02x should be 0x0\n",
			   value );
		goto abort;
	}

	M_setstat(path, M_MK_CH_CURRENT, 0);
	M_write( path, 0x0a00 );
	UOS_Delay(10);
	M_setstat(path, M_MK_CH_CURRENT, 3);
	M_read( path, &value );		/* port H */

	if( value != 0xa ){
		printf("*** Verify error #7: port H is 0x%02x should be 0xa\n",
			   value );
		goto abort;
	}


	/*--------------------+
    |  cleanup            |
    +--------------------*/
	abort:
	/*--- disable global irqs ---*/
	M_setstat( path, M_MK_IRQ_ENABLE, FALSE );


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
