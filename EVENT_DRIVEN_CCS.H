/*
*************************** C HEADER FILE ************************************

project   :
filename  : EVENT_DRIVEN_CCS.H
version   : 2
date      :

******************************************************************************

Copyright (c) 20xx ,
All rights reserved.

******************************************************************************

VERSION HISTORY:
----------------------------
Version      : 1
Date         :
Revised by   :
Description  :

Version      : 2
Date         :
Revised by   :
Description  : *
               *
               *

******************************************************************************
*/

#ifndef  EVENT_DRIVEN_CCS_INCLUDED
#define  EVENT_DRIVEN_CCS_INCLUDED

/****************************************************************************/
/**                                                                        **/
/**                              MODULES USED                              **/
/**                                                                        **/
/****************************************************************************/

//#include <stdlib.h>  // depend on application requirements.
//#include "cdat68k.h"  // define data types
#include "CONFIG_PIC24.H"

#include "CDATPIC24.H"
#include "EVENT_TYPE_CCS.H"

/****************************************************************************/
/**                                                                        **/
/**                        DEFINITIONS AND MACROS                          **/
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/***********    Macro for Declare and Define Memory Blocks ******************/

#define DECLARE_MEM(BLOCK_TYPE_DEF, MEM_ARRAY, STRUCT_Q_ARRAY, N_BLOCK); \
PTR_STRUCT STRUCT_Q_ARRAY[N_BLOCK]; \
BLOCK_TYPE_DEF MEM_ARRAY[N_BLOCK];

/****************************************************************************/

// Queue error codes
#define Q_OK 0
#define Q_FULL 1
#define Q_EMPLY 2

//timer mode
#define SINGLE 0
#define REPEAT 1

//timer enable, disable
#define ON 1
#define OFF 0


/****************************************************************************/
/**                                                                        **/
/**                        TYPEDEFS AND STRUCTURES                         **/
/**                                                                        **/
/****************************************************************************/

//Data type for Tick Counter
typedef volatile int32u tickType;

//Event Structure
typedef volatile struct
{
    EV_TYPE evType;
    void *evDataPtr;
} EV_STRUCT;

//Event Queue Structure
typedef volatile struct
{
    int8u evQLength;
    int8u evQCount;
    int8u evQPut;
    int8u evQGet;
    EV_STRUCT *evQArray;
} EVQ_STRUCT;

//Event Queue function error codes
typedef enum
{
    EVQ_OK,
    EVQ_FULL,
    EVQ_EMPTY
} Q_ERR;

//Structure of pointer for memm queue array
typedef volatile struct voidPtrStrict
{
    void *blockPtr;
} PTR_STRUCT;

//Structure type for Mem Control Block
typedef volatile struct memControlStruct
{
    void *memPtr;
    int16u qArrayLength;
    int16u count;
    int16u put;
    int16u get;
    PTR_STRUCT *qArrayPtr; // Pointer to array of pointer to void
} MCB_STRUCT;

//Structure type for queue of pointer
typedef volatile struct qPtrXStruct
{
    int16u arrayLength;
    int16u count;
    int16u put;
    int16u get;
    PTR_STRUCT *arrayPtr; // Pointer to array of pointer to void
} QPTRX_STRUCT;

//Structure type for queue of int8u
typedef volatile struct q8UStruct
{
    int16u arrayLength;
    int16u count;
    int16u put;
    int16u get;
    int8u *arrayPtr; // Pointer to array of int8u
} Q8UX_STRUCT;

//STimer struct
typedef volatile struct
{
    int8u tmrEnb; //enable (ON) / disable (OFF) software timer
    int8u tmrMode; //operating mode: SINGLE / REPEAT
    tickType tmrInit; // Initial value of tick count
    tickType tmrCount; //Down Counter value
    EVQ_STRUCT *evQPtr; // Pointer to Event Queue
    EV_STRUCT sTmrEvStruct; //software Timer Event Structure
} STMR_STRUCT;

//RX Frame State
typedef volatile enum
{
    FRAME_WAIT,
    FRAME_PROGRESS
} FRAME_STATE; 

//Tx state
typedef volatile enum
{
    TX1_BUSY,
    TX1_READY
} TX1_STATUS;

/****************************************************************************/
/**                                                                        **/
/**                         EXPORTED VARIABLES                             **/
/**                                                                        **/
/****************************************************************************/

#ifndef EVENT_DRIVEN_CCS_C_SRC

extern volatile int16u EvQFullCount;

#endif

/****************************************************************************/
/**                                                                        **/
/**                       EXPORTED FUNCTIONS                               **/
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
//                          Library Functions                               //
/****************************************************************************/

/****************************************************************************/
/*************************  Event Queue Functions  **************************/
/****************************************************************************/

void EvSructInit (EV_STRUCT *evStruct , EV_TYPE evType, void *evDataPtr );

void EvQInit (EVQ_STRUCT *evQ, EV_STRUCT *evArray, int8u qL );

void EvQPut (EVQ_STRUCT *evQ, EV_STRUCT evSource, Q_ERR *errPtr);

void EvQGet (EVQ_STRUCT *evQ, EV_STRUCT *evDest, Q_ERR *errPtr);

int16u EvQCount (EVQ_STRUCT *evQ);

/****************************************************************************/

/****************************************************************************/
/****************************  Memory Functions  ****************************/
/****************************************************************************/

void MemStructArrayInit (void *memArrayPtr, int16u blockByteSize, int16u nBlock,
                         PTR_STRUCT *memQArrayPtr, MCB_STRUCT *memQcbPtr);

void MemPtrPut (MCB_STRUCT *memQcbPtr, void *memBlockPtr, int8u *errPtr);

void MemPtrGet (MCB_STRUCT *memQcbPtr, PTR_STRUCT *destStructPtr, int8u *errPtr);

int16u MemQReadyCount (MCB_STRUCT *memQcbPtr);

/****************************************************************************/

/****************************************************************************/
/***************************** QPTRX Functions  *****************************/
/****************************************************************************/

void QPtrXInit (QPTRX_STRUCT *qStructPtr, PTR_STRUCT *buffPtr, int16u qL);

void QPtrXPut (QPTRX_STRUCT *qStructPtr, PTR_STRUCT ptrStruct, int8u *errPtr);

void QPtrXGet (QPTRX_STRUCT *qStructPtr, PTR_STRUCT *destPtr, int8u *errPtr);

int16u QPtrXCount (QPTRX_STRUCT *qStructPtr);

/****************************************************************************/

/****************************************************************************/
/***************************** Q8UX Functions *******************************/
/****************************************************************************/

void Q8UXInit (Q8UX_STRUCT *qStructPtr, int8u *buffPtr, int16u qL);

void Q8UXPut (Q8UX_STRUCT *qStructPtr, int8u dataSource, int8u *errPtr);

void Q8UXGet (Q8UX_STRUCT *qStructPtr, int8u *destPtr, int8u *errPtr);

int16u Q8UXCount (Q8UX_STRUCT *qStructPtr);

/****************************************************************************/

/****************************************************************************/
/************************ Software Timer Functions  *************************/
/****************************************************************************/

void SoftwareTimer (STMR_STRUCT *sTmrPtr, int8u nMaxSTmr);

void STmrSystemDeInit (STMR_STRUCT *sTmrPtr, int8u nMaxSTmr);

void STmrInit (STMR_STRUCT *sTmrPtr, int8u z, tickType, initValue, int8u mode,
               EV_TYPE evType, void *evDataPtr, EVQ_STRUCT *evQPtr);

void STmrStart (STMR_STRUCT *sTmrPtr, int8u z);

void StmrPause (STMR_STRUCT *sTmrPtr, int8u z);

void STmrStop (STMR_STRUCT *sTmrPtr, int8u z);

/****************************************************************************/

/****************************************************************************/
//                           CPU Dependent Functions  
/****************************************************************************/

void DisableIntr (void);

void EnableIntr (void);

void CpuIdle (void);


/****************************************************************************/


#endif

/****************************************************************************/
/**                                                                        **/
/**                              EOF                                       **/
/**                                                                        **/
/****************************************************************************/