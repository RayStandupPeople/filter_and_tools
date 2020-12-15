/*******************************************************************************
**----------------------------------------------------------------------------**
** Copyright (c) 2020 by G-Pulse.		All rights reserved.
** This software is copyright protected and proprietary to G-Pulse.
** G-Pulse grants to you only those rights as set out in the license conditions.
** All other rights remain with G-Pulse.
**----------------------------------------------------------------------------**
** Administrative Information
** Namespace_:	/Common
** Class_____:	C
** Name______:	Platform_Types.h
** Version___:	V1.0.0
** Revision__:	0
** Author____:	Tang Xin
**----------------------------------------------------------------------------**
** MAY BE CHANGED BY USER [Yes/No]: Yes
**----------------------------------------------------------------------------**
** DESCRIPTION:
** Platform Types definition
*******************************************************************************/
#ifndef PLATFORM_TYPES_H
#define PLATFORM_TYPES_H

/*******************************************************************************
**                      Other Header File Inclusion                           **
*******************************************************************************/


/*******************************************************************************
**                          Macro Definition                         		  **
*******************************************************************************/
#define MSB_FIRST        0    /* big endian bit ordering */
#define LSB_FIRST        1    /* little endian bit ordering */

#define HIGH_BYTE_FIRST  0    /* big endian byte ordering */
#define LOW_BYTE_FIRST   1    /* little endian byte ordering */

#ifndef TRUE
   #define TRUE   1
#endif

#ifndef FALSE
   #define FALSE  0
#endif

#define CPU_BIT_ORDER    LSB_FIRST        /*little endian bit ordering*/

#define CPU_BYTE_ORDER   LOW_BYTE_FIRST   /*little endian byte ordering*/
                      

#define E_OK 0
#define E_NOT_OK 1


/*******************************************************************************
**                          Typedef Definition                         		  **
*******************************************************************************/
typedef unsigned char         boolean;       /*        TRUE .. FALSE           */

typedef signed char           sint8;         /*        -128 .. +127            */
typedef unsigned char         uint8;         /*           0 .. 255             */
typedef signed short          sint16;        /*      -32768 .. +32767          */
typedef unsigned short        uint16;        /*           0 .. 65535           */
typedef signed int           sint32;        /* -2147483648 .. +2147483647     */
typedef unsigned int         uint32;        /*           0 .. 4294967295      */
/* Attention: uint64 is NOT an official AUTOSAR type! */
                                             /*  9223372036854775807           */
typedef unsigned long long    uint64;        /*           0 ..                 */
typedef signed long long      sint64;        /*           0 ..                 */
                                        
typedef signed char           sint8_least;   /* At least 7 bit + 1 bit sign    */
typedef unsigned char         uint8_least;   /* At least 8 bit                 */
typedef signed short          sint16_least;  /* At least 15 bit + 1 bit sign   */
typedef unsigned short        uint16_least;  /* At least 16 bit                */
typedef signed int           sint32_least;  /* At least 31 bit + 1 bit sign   */
typedef unsigned int         uint32_least;  /* At least 32 bit                */
                                        
typedef float                 float32;
typedef double                float64;

typedef  signed int	Std_ReturnType;


/*******************************************************************************
**                  Global Variables With Extern Linkage               		  **
*******************************************************************************/

/*******************************************************************************
**                        Global Function Prototypes              	    	  **
*******************************************************************************/

/*******************************************************************************
** Function Name	: Demo_Fun            			  		      
** Parameter[in] 	: None      								  
** Parameter[out] 	: None                                	    	          
** Return Value	  	: None          
** Note			  	：None
** Description	  	: demo function	                 
*******************************************************************************/
 
#endif  /* PLATFORM_TYPES_H */


/*******************************************************************************
**----------------------------------------------------------------------------**
**              		R E V I S I O N   H I S T O R Y						  **
**----------------------------------------------------------------------------**
** 2020-01-06   1.0.0  Tang Xin      first implementation
**
*******************************************************************************/
