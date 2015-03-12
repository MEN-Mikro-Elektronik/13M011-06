#***************************  M a k e f i l e  *******************************
#
#         Author: kp
#          $Date: 2004/04/26 17:49:20 $
#      $Revision: 1.2 $
#
#    Description: Makefile definitions for M11 tool M11_READ
#
#---------------------------------[ History ]---------------------------------
#
#   $Log: program.mak,v $
#   Revision 1.2  2004/04/26 17:49:20  cs
#   removed unused include mdis_err.h
#
#   Revision 1.1  1999/11/03 15:40:43  kp
#   Initial Revision
#
#-----------------------------------------------------------------------------
#   (c) Copyright 1999 by MEN mikro elektronik GmbH, Nuernberg, Germany
#*****************************************************************************

MAK_NAME=m11_read

MAK_LIBS=$(LIB_PREFIX)$(MEN_LIB_DIR)/mdis_api$(LIB_SUFFIX)	\
         $(LIB_PREFIX)$(MEN_LIB_DIR)/usr_oss$(LIB_SUFFIX)	\
         $(LIB_PREFIX)$(MEN_LIB_DIR)/usr_utl$(LIB_SUFFIX)	\

MAK_INCL=$(MEN_INC_DIR)/m11_drv.h	\
         $(MEN_INC_DIR)/men_typs.h	\
         $(MEN_INC_DIR)/mdis_api.h	\
         $(MEN_INC_DIR)/usr_oss.h	\
         $(MEN_INC_DIR)/usr_utl.h	\

MAK_INP1=$(MAK_NAME)$(INP_SUFFIX)

MAK_INP=$(MAK_INP1)