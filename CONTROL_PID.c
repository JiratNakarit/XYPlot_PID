/*
*************************** C SOURCE FILE ************************************
project   :
filename  : CONTROL_PID.C
version   : 2
date      :
******************************************************************************
Copyright (c) 20xx
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

#define CONTROL_PID_C_SRC

/****************************************************************************/
/**                                                                        **/
/**                             MODULES USED                               **/
/**                                                                        **/
/****************************************************************************/

#include "CONTROL_PID.h"
#include "BITWISE_LIB.h"
#include "EVENT_DRIVEN_CCS.H"
#include <math.h>
/****************************************************************************/
/**                                                                        **/
/**                        DEFINITIONS AND MACROS                          **/
/**                                                                        **/
/****************************************************************************/

#define START_DUTY 0
#define STOP 0
#define CW 1 
#define CCW 2
#define MOTOR_L 1
#define MOTOR_R 2
#define RED_X 0
#define RED_Y 0
#define GREEN_X 0
#define GREEN_Y 0
#define BLUE_X 0
#define BLUE_Y 0

#define RX1Q_LN 8
#define RX_CMND_FRM_LN 25
#define START_CHR   '['
#define END_CHR     ']'
#define CONDITION_S '('
#define CONDITION_E ')'
#define CONDITION_M ','
//UART Queue
#define TX1Q_LN 128

/****************************************************************************/
/**                                                                        **/
/**                        TYPEDEFS AND STRUCTURES                         **/
/**                                                                        **/
/****************************************************************************/


/****************************************************************************/
/**                                                                        **/
/**                      PROTOTYPES OF LOCAL FUNCTIONS                     **/
/**                                                                        **/
/****************************************************************************/
static void HardwareInit (void);
static void GlobalVarInit (void);
static void DynamicMemInit (void);
static void UARTQueueInit (void);
static int8u SendTx1 (int8u *strPtr);

//My Function
static void create_position_array(void);
static int8u ConvertStr2Int (int8u num);
/****************************************************************************/
/**                                                                        **/
/**                           EXPORTED VARIABLES                           **/
/**                                                                        **/
/****************************************************************************/


/****************************************************************************/
/**                                                                        **/
/**                            GLOBAL VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/
float Kp_x = 0.0f,Kp_y = 0.0f;
float Ki_x = 0.0f,Ki_y = 0.0f;
float Kd_x = 0.0f,Kd_y = 0.0f;
int16s e_x = 0,e_y = 0;
float int_e_x = 0,int_e_y = 0;
float u_x = 0,u_y = 0;
float control_x = 0,control_y = 0;
float p_e_x = 0,p_e_y = 0;
float tor = 2;
static int16u duty1;
int32s count_pulseX = 0; 
int32s count_pulseY = 0; 
float thetaX = 0;
float thetaY = 0;
int DistanceX = 0;
int DistanceY = 0;
int16u target_posX = 0;
int16u target_posY = 0;
int h;
static float current_posX = 0;
static float current_posY = 0;
int16u ReadPortB,B0,B1,B2,B3;
float radius = 12.20; //old radius = 11.92

int home_state = 1;
char move_state;
int x,y;
int send_end = 0;

static volatile Q8UX_STRUCT Tx1QCB;
static volatile int8u Tx1QArray[TX1Q_LN];
static volatile int8u *Tx1BuffPtr;
static volatile int16u TxBuffIdx;
static volatile TX1_STATUS Tx1Flag;
static volatile int16u Tx1FrameIn, Tx1FrameOut, Rx1FrameCount, RxCount,
                        Tx1QFullCount, Rx1QFullCount;

static volatile int8u *RxBuffPtr;
static volatile QPTRX_STRUCT Rx1QCB;
static volatile PTR_STRUCT Rx1BuffPtrArray[RX1Q_LN];
static volatile PTR_STRUCT DestPtrStruct;

static volatile int16u MemFail, MemCount;

static int position_x[50];
static int position_y[50];

/****************************************************************************/
/**                                                                        **/
/**                           EXPORTED FUNCTIONS                           **/
/**                                                                        **/
/****************************************************************************/
void drive_motor(int axis,int direction,int speed){
/***************************
	MotorLeft(CW) >> IN1  H
				  >> IN2  L
	MotorRight(CW) >> IN3  H
	               >> IN4  L
	Y_PLUS = Left(CW) + Right(CCW)
	X_PLUS = Left(CW) + Right(CW)
****************************/
	
	if(speed == 0 || direction == STOP){		
		if(axis == MOTOR_L){
			output_low(IN3);
			output_low(IN4);
		}
		else if(axis == MOTOR_R){
			output_low(IN1);
			output_low(IN2);
		}	
		set_pwm_duty(1, 400);
		set_pwm_duty(2, 400); 			
	}
	else if(axis == MOTOR_L){
		if(direction == CW){
			output_high(IN3);
			output_low(IN4);
			set_pwm_duty(1, speed);
			set_pwm_duty(2, speed);
		}
		else if(direction == CCW){
			output_low(IN3);
			output_high(IN4);
			set_pwm_duty(1, speed);
			set_pwm_duty(2, speed);
		}
	}
	else if(axis == MOTOR_R){
		if(direction == CW){
			output_high(IN1);    
			output_low(IN2);
			set_pwm_duty(1, speed);
			set_pwm_duty(2, speed);
		}
		else if(direction == CCW){
			output_low(IN1);    
			output_high(IN2);
			set_pwm_duty(1, speed);
			set_pwm_duty(2, speed);
		}
	}
	
}

void control_position(int axis, float target, float currentPosition) {
	if(axis == MOTOR_L){
		float targetX = target;
		float currentX = currentPosition;
		Kp_x = 20;         //old value = 20
		Ki_x = 0.00001;     //old value = 0.00001
		Kd_x = 8;       //old value = 8

		e_x = targetX - currentX;
		int_e_x = int_e_x + e_x;
		u_x = Kp_x*e_x + Ki_x*int_e_x + Kd_x*(e_x-p_e_x);
		/*x = currentX;
		printf("\r\nX = %d||Y = %d",x,y);*/
		p_e_x = e_x;  
		
		if (abs(e_x)>tor){
			if(abs(u_x) > 255){
				control_x = 255;	
			}
			else if(abs(u_x)<50){
				control_x = abs((50/255)*400);
			}else{
				control_x = abs((u_x/255)*400);
			}
			
			if (u_x>0){
				drive_motor(axis,CW,control_x);
			}
			else{
				drive_motor(axis,CCW,control_x);
			}
		}else{
				drive_motor(MOTOR_L,STOP,0);
				if(send_end == 0){
					printf("[e]");
					send_end = 1;
				}
		}
	}
	else if(axis == MOTOR_R){
		float targetY = target;
		float currentY = currentPosition;
		Kp_y = 20;          //old value = 29.5
		Ki_y = 0.000001;  //old value = 0.0001
		Kd_y = 0.0001;			 //old value = 13

		e_y = targetY - currentY;
		int_e_y = int_e_y + e_y;
		u_y = Kp_y*e_y + Ki_y*int_e_y + Kd_y*(e_y-p_e_y);
		/*y = currentY;
		printf("\r\nX = %d||Y = %d",x,y);*/
		p_e_y = e_y;  
		
		if (abs(e_y)>tor){
			if(abs(u_y) > 255){
				control_y = 255;
			}
			else if(abs(u_y) < 50){
				control_y = abs((50/255)*400);
			}else{
				control_y = abs((u_y/255)*400);
			}
			if (u_y>0){
				drive_motor(axis,CW,control_y);
			}
			else{
				drive_motor(axis,CCW,control_y);
			}
			
		}else{
			drive_motor(MOTOR_R,STOP,0);
			if(send_end == 0){
				printf("[e]");
				send_end = 1;
			}
		}
		
	}
	
}
void Encoder(void){
	thetaX = (count_pulseX*7.5)/64;
	thetaY = (count_pulseY*7.5)/64;
	current_posX = (thetaX*(22/7)*radius)/180;
	current_posY = (thetaY*(22/7)*radius)/180;
}

/****************************************************************************/
/**                                                                        **/
/**                             LOCAL FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/
int main (void)
 {		
	int8u errCode, sendTx1Count;

    DisableIntr ();
    HardwareInit ();
    GlobalVarInit ();
    DynamicMemInit ();
    UARTQueueInit ();
    EnableIntr ();
    for(;;){
        DisableIntr();
        QPtrXGet(&Rx1QCB, &DestPtrStruct, &errCode);
        if (errCode == Q_OK)
        {
            create_position_array();
			if(DestPtrStruct.blockPtr[0] == 'h'){ 
				/* HOME */
				disable_interrupts(INT_TIMER4);
				home_state = 1;
				clear_interrupt(INT_TIMER1);
				enable_interrupts(INT_TIMER1);
			}
			else if(DestPtrStruct.blockPtr[0] == 's'){
				/*  Backward after pick pen */
				move_state = 's';
				clear_interrupt(INT_TIMER4);
				enable_interrupts(INT_TIMER4);
			}
			else if(DestPtrStruct.blockPtr[0] == '1'){
				/* Left Pen : Red & Orange*/
				move_state = '1';
				clear_interrupt(INT_TIMER4);
				enable_interrupts(INT_TIMER4);
			}
			else if(DestPtrStruct.blockPtr[0] == '2'){
				/* Middle Pen : Green*/
				move_state = '2';
				clear_interrupt(INT_TIMER4);
				enable_interrupts(INT_TIMER4);
			}
			else if(DestPtrStruct.blockPtr[0] == '3'){
				/* Right Pen : Blue & Black*/
				move_state = '3';	
				clear_interrupt(INT_TIMER4);
				enable_interrupts(INT_TIMER4);
			}
        	EnableIntr ();	
            //sendTx1Count = SendTx1 (((int8u *)DestPtrStruct.blockPtr));
			printf("[e]");
            //putc('F');
            free ((void *)DestPtrStruct.blockPtr);
            MemCount--;
            //EnableIntr();
        }
        else
        {
            EnableIntr();
        }
    }
    return 0;
}

static void HardwareInit (void)
{
	setup_adc_ports(NO_ANALOGS);
	set_tris_a (get_tris_a () & 0xffe8); //1111 1111 1110 1000
	set_tris_b (get_tris_b () & 0xff3f); //1111 1111 0011 1111
	B0 = (ReadPortB & 0x0001);
	B1 = (ReadPortB & 0x0002) >> 1;
	B2 = (ReadPortB & 0x0004) >> 2;
	B3 = (ReadPortB & 0x0008) >> 3;

	setup_capture(1,CAPTURE_EE | INTERRUPT_EVERY_CAPTURE |CAPTURE_TIMER2);
	clear_interrupt(INT_IC1);
	enable_interrupts(INT_IC1);
	
	setup_capture(2,CAPTURE_EE | INTERRUPT_EVERY_CAPTURE |CAPTURE_TIMER2);
	clear_interrupt(INT_IC2);
	enable_interrupts(INT_IC2);

	setup_capture(3,CAPTURE_EE | INTERRUPT_EVERY_CAPTURE |CAPTURE_TIMER2);
	clear_interrupt(INT_IC3);
	enable_interrupts(INT_IC3);

	setup_capture(4,CAPTURE_EE | INTERRUPT_EVERY_CAPTURE |CAPTURE_TIMER2);
	clear_interrupt(INT_IC4);
	enable_interrupts(INT_IC4);

	clear_interrupt(INT_EXT1);
	enable_interrupts(INT_EXT1);
	
	clear_interrupt(INT_EXT2);
	enable_interrupts(INT_EXT2);

	duty1 = START_DUTY;
	set_compare_time(1, duty1, duty1);
	setup_compare(1, COMPARE_PWM | COMPARE_TIMER3);

	set_compare_time(2, duty1, duty1);
	setup_compare(2, COMPARE_PWM | COMPARE_TIMER3);
	
	set_timer1(0);
	setup_timer1(TMR_INTERNAL | TMR_DIV_BY_256, 6249); 	
	
	set_timer2(0);
	setup_timer2(TMR_INTERNAL | TMR_DIV_BY_1, 65535);

	set_timer3(0);
	setup_timer3(TMR_INTERNAL | TMR_DIV_BY_1, 400);

	set_timer4(0);
	setup_timer4 (TMR_INTERNAL | TMR_DIV_BY_256, 6249);

	set_timer5(0);
	setup_timer5 (TMR_INTERNAL | TMR_DIV_BY_256, 624);
	return;
}

static void GlobalVarInit (void)
{
    Tx1Flag = TX1_READY;
    TxBuffIdx = 0;
    Tx1FrameIn = 0;
    Tx1QFullCount = 0;
    Rx1FrameCount = 0;
    RxCount = 0;
    Rx1QFullCount = 0;
    MemFail = 0;
    MemCount = 0;
    return;
}
static void DynamicMemInit (void)
{
    //get mem block for first Rx1 buffer
    RxBuffPtr = (int8u *)malloc ((sizeof (int8u)) * RX_CMND_FRM_LN);
    if (RxBuffPtr != (int8u *)NULL)
    {
        MemCount++;
        clear_interrupt(INT_RDA);
        enable_interrupts (INT_RDA);
    }
    else
    {
        MemFail++;
    }
    return;
}
static void UARTQueueInit (void)
{
    QPtrXInit(&Rx1QCB, Rx1BuffPtrArray, RX1Q_LN);
    Q8UXInit(&Tx1QCB, Tx1QArray, TX1Q_LN);
    return;
}
static int8u SendTx1 (int8u *strPtr)//critical section
{
    int8u strLn;
    int8u strIdx;
    int8u qSpace;
    int8u errCode;
    int8u count;
    count = 0;
    strLn = strlen(strPtr);
    if (strLn != 0)
    {
        qSpace = TX1Q_LN - Q8UXCount (&Tx1QCB);
        if (qSpace >= (int16u)strLn)
        {
            for (strIdx = 0; strIdx < strLn; strIdx++)
            {
                Q8UXPut (&Tx1QCB, strPtr[strIdx], &errCode);
                count++;
            }
            if (Tx1Flag == TX1_READY)
            {
                Tx1Flag = TX1_BUSY; // Set Rx1 to Busy.
                TX1IF = 1;// Start Tx1 Interupt. 
                enable_interrupts(INT_TBE);
            }
        }
    }
    return count;
}

/****************************************************************************/

/****************************************************************************/
/**                               My Function                              **/
/****************************************************************************/
static void create_position_array(void)
{
    int16u c_state = 0;
    int16u s_state = 0;
    int16u sum_a = 0; 
    int16u sum_b = 0;
    int16u sum_c = 0;
	int16u sum_d = 0;
    int16u pos_r = 0;
    int *posY;
    int *posX;
    int16u Arr_state = 0;

    posY = position_y;
    posX = position_x;

    for(int8u i = 0; i < strlen(DestPtrStruct.blockPtr); i++)
    {
        if(DestPtrStruct.blockPtr[i] == CONDITION_S)
        {
            c_state = 1;
        }
        else if(DestPtrStruct.blockPtr[i] == CONDITION_M)
        {
            if(c_state == 1)
            {
                //Found x
                if(s_state==1){pos_r = sum_a;}
                if(s_state==2){pos_r = sum_a*10+sum_b;}
                if(s_state==3){pos_r = sum_a*100+sum_b*10+sum_c;}
				if(s_state==4){pos_r = sum_a*1000+sum_b*100+sum_c*10+sum_d;}
                printf("\r\n x == %d", pos_r);
				DistanceX = pos_r/10;
                *(posX+Arr_state) = pos_r;
                s_state = 0;
                c_state = 0;
            }
        }
        else if(DestPtrStruct.blockPtr[i] == CONDITION_E)
        {
            //Found y
            if(s_state==1){pos_r = sum_a;}
            if(s_state==2){pos_r = sum_a*10+sum_b;}
            if(s_state==3){pos_r = sum_a*100+sum_b*10+sum_c;}
			if(s_state==4){pos_r = sum_a*1000+sum_b*100+sum_c*10+sum_d;}
            printf("\r\n y == %d", pos_r);
			DistanceY = pos_r/10;
            *(posY+Arr_state) = pos_r;
            s_state = 0;
            Arr_state++;
			move_state = 'w';
			send_end = 0;
			clear_interrupt(INT_TIMER4);
			enable_interrupts(INT_TIMER4);
        }
        else
        {
            //Found numeric
            //printf("\r\nDestPtrStruct.blockPtr[i] = %c", DestPtrStruct.blockPtr[i]);
            if(s_state == 0){sum_a = ConvertStr2Int(DestPtrStruct.blockPtr[i]);}
            if(s_state == 1){sum_b = ConvertStr2Int(DestPtrStruct.blockPtr[i]);}
            if(s_state == 2){sum_c = ConvertStr2Int(DestPtrStruct.blockPtr[i]);}
            s_state++;
        }
    }
    return;
}        

static int8u ConvertStr2Int (int8u num)
{
	return num - '0';
}
/****************************************************************************/

/****************************************************************************/
/**                           Interrupt Functions                          **/
/****************************************************************************/

#INT_RDA //Rx1 interupt
void RDA1 (void)
{
    static FRAME_STATE FrameState = FRAME_WAIT;
    static int16u FrmIdx = 0;
    int8u Chr;
    int8u *errCode;

    Chr = getc(); // Read data from Rx1 register
    RxCount++;
    switch (FrameState) // State machine for build frame
    {
        case FRAME_WAIT: // waits for start char of new frame
            if (Chr == START_CHR)
            {
                //Get a start char
                //RxBuffPtr[FrmIdx] = Chr; // Save start char to frame
                //FrmIdx++;
                FrameState = FRAME_PROGRESS; // Change state to build frame
            }
            break;
        case FRAME_PROGRESS: // Build frame
            if ((FrmIdx == (RX_CMND_FRM_LN - 2)) && (Chr != END_CHR))
            {
                //Characters exceed frame lenght. Frame error.
                // Rejects data and black to wait new frame.
                FrmIdx = 0;
                FrameState = FRAME_WAIT;
            }
            else if (Chr == END_CHR)
            {
                //Get and char. Frame completes.
                //RxBuffPtr[FrmIdx] = Chr;
                //FrmIdx++;
                RxBuffPtr[FrmIdx] = 0;
                FrmIdx = 0;
                Rx1FrameCount++;
                //Sends Rx1 event to event queue.
                QPtrXPut (&Rx1QCB, (void *)RxBuffPtr, &errCode);
                if (errCode == Q_FULL)
                {
                    //EvQ full error. Free mem of data block.
                    free ((void *)RxBuffPtr);
                    MemCount--;
                    Rx1QFullCount++;
                }
                FrameState = FRAME_WAIT; // Back to wait for start char of new frame.
                // GEt mem block for better of new frame
                RxBuffPtr = (int8u *)malloc ((sizeof (int8u)) * RX_CMND_FRM_LN);
                if (RxBuffPtr == (int8u *)NULL)
                {
                    //Can not get mem block. Disable Rx1 interupt.
                    disable_interrupts(INT_RDA);
                    MemFail++;
                }
                else
                {
                    MemCount++;
                }
            }
            else
            {
                RxBuffPtr[FrmIdx] = Chr;
                FrmIdx++;
            }
            break;
        default:
            break;
    }
    return;
}

#INT_TBE // Tx1 interupt
void TBE1ISR (void)
{
    int8u destChr;
    Q_ERR errCode;
    Q8UXGet (&Tx1QCB, &destChr, &errCode);
    if (errCode == Q_OK)
    {
        putc(destChr);
    }
    else
    {
        disable_interrupts (INT_TBE);
        Tx1Flag = TX1_READY;
    }
    return;
}

#INT_TIMER1
void Home(void){
	if(home_state == 1){
		drive_motor(MOTOR_R,CCW,100);
		drive_motor(MOTOR_L,CCW,100);
	}
	else if(home_state == 2){
		drive_motor(MOTOR_R,CW,100);
		drive_motor(MOTOR_L,CCW,100);
	}
	else if(home_state == 3){
		home_state = 0;
		drive_motor(MOTOR_R,STOP,0);
		drive_motor(MOTOR_L,STOP,0);
		printf("[h]");
		disable_interrupts(INT_TIMER1);
	}
	return;
}

#INT_EXT1
void Initial_Y_axis(void){
	count_pulseX = 0;
	count_pulseY = 0;
	thetaY = 0;
	current_posY = 0;
	home_state = 3;
	return;
}
#INT_EXT2
void Initial_X_axis(void){
	count_pulseX = 0;
	count_pulseY = 0;
	thetaX = 0;
	current_posX = 0;
	home_state = 2;
	return;
}	

#INT_IC1
void Encoder1_A(void){
	ReadPortB = input_b() & 0x0003;  
	B0 = (ReadPortB & 0x0001);
	if((B0 ^ B1) == 0){
		count_pulseX--;
	}
	else{
		count_pulseX++;
	}
	//printf("\ncount_pulseX = %d",count_pulseX);
	return;
}

#INT_IC2
void Encoder1_B(void){
	ReadPortB = input_b() & 0x0003;  
	B1 = (ReadPortB & 0x0002 ) >> 1;
	if((B0 ^ B1) == 0){
		count_pulseX++;
	}
	else{
		count_pulseX--;
	}
	//printf("\r\ncount_pulseX = %d",count_pulseX);
	return;
}
	
#INT_IC3
void Encoder2_A(void){
	ReadPortB = input_b() & 0x000c; 
	B2 = (ReadPortB & 0x0004) >> 2; 
	if((B2 ^ B3) == 0){
		count_pulseY--;
	}
	else{
		count_pulseY++;
	}
	//printf("\r\ncount_pulseY = %d",count_pulseY);
	return;
}

#INT_IC4
void Encoder2_B(void){
	ReadPortB = input_b() & 0x000c;  
	B3 = (ReadPortB & 0x0008) >> 3;
	if((B2 ^ B3) == 0){
		count_pulseY++;
	}
	else{
		count_pulseY--;
	}
	//printf("\r\ncount_pulseY = %d",count_pulseY);
	return;
}

#INT_TIMER4
void Run_Motor(void){
	disable_interrupts(INT_TIMER5);
	Encoder();
	if(move_state == 'w'){
		/* MOVE COMMONLY */ 
		target_posX = 0.5*(DistanceX + DistanceY);
		target_posY = 0.5*(DistanceX - DistanceY);
		control_position(MOTOR_L,target_posX,current_posX);
		control_position(MOTOR_R,target_posY,current_posY);
	}
	else if(move_state == 's'){
		/* Backward a little bit */
		drive_motor(MOTOR_L,CCW,100);
		drive_motor(MOTOR_R,CW,100);
	}
	else if(move_state == '1'){
		/*To RED & ORANGE pen position*/
		control_position(MOTOR_L,RED_X,current_posX);
		control_position(MOTOR_R,RED_Y,current_posY);
	}
	else if(move_state == '2'){
		/*To GREEN pen position*/
		control_position(MOTOR_L,GREEN_X,current_posX);
		control_position(MOTOR_R,GREEN_Y,current_posY);
	}
	else if(move_state == '3'){
		/*To BLUE & BLACK pen position*/
		control_position(MOTOR_L,BLUE_X,current_posX);
		control_position(MOTOR_R,BLUE_Y,current_posY);
	}
	return;
}

#INT_TIMER5
void Run_Motor2(void){
	disable_interrupts(INT_TIMER4);
	return;
}


/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/**                                 EOF                                    **/
/**                                                                        **/
/****************************************************************************/