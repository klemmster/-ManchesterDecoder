/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 * 
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430G2xx3 Demo - DCO Calibration Constants Programmer
//
//  NOTE: THIS CODE REPLACES THE TI FACTORY-PROGRAMMED DCO CALIBRATION
//  CONSTANTS LOCATED IN INFOA WITH NEW VALUES. USE ONLY IF THE ORIGINAL
//  CONSTANTS ACCIDENTALLY GOT CORRUPTED OR ERASED.
//
//  Description: This code re-programs the G2xx2 DCO calibration constants.
//  A software FLL mechanism is used to set the DCO based on an external
//  32kHz reference clock. After each calibration, the values from the
//  clock system are read out and stored in a temporary variable. The final
//  frequency the DCO is set to is 1MHz, and this frequency is also used
//  during Flash programming of the constants. The program end is indicated
//  by the blinking LED.
//  ACLK = LFXT1/8 = 32768/8, MCLK = SMCLK = target DCO
//  //* External watch crystal installed on XIN XOUT is required for ACLK *//
//
//           MSP430G2xx3
//         ---------------
//     /|\|            XIN|-
//      | |               | 32kHz
//      --|RST        XOUT|-
//        |               |
//        |           P1.0|--> LED
//        |           P1.4|--> SMLCK = target DCO
//
//  D. Dang
//  Texas Instruments Inc.
//  May 2010
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 3.42A
//******************************************************************************
#include <msp430.h>
#include <stdint.h>

#define False 0x00;
#define True 0x01;

#define LOW_to_HIGH (0x0000)
#define HIGH_to_LOW (0x00FF)

volatile unsigned int i;

volatile unsigned char isSyncing;
volatile unsigned char isSynced;
volatile unsigned char currentBit;
volatile unsigned char nextBit;
volatile unsigned char parityIndex;

volatile unsigned char inBufferIndex;
volatile unsigned char dataIndex;
volatile unsigned int inBuffer[50];

volatile unsigned char expectShortEdge;
volatile unsigned char receivedHeader;

volatile unsigned char green_LED = BIT6;
volatile unsigned char red_LED = BIT0;
volatile unsigned char input_Pin = BIT1;
volatile unsigned char bitpos_input_pin = 1;

volatile unsigned int T2Max = 600; //T2 + Ttolerance;
volatile unsigned int T2Min = 400; //T2 - Ttolerance;

volatile unsigned int TMax = 400; //T + Ttolerance;
volatile unsigned int TMin = 200; //T - Ttolerance;

volatile uint32_t lastTagID = 0;

volatile uint16_t headerBits = 0; //Header 9 bits 1 == header done

#define RESET() { \
		isSyncing = False; \
		isSynced = False; \
		inBufferIndex = 0; \
		dataIndex = 0; \
		expectShortEdge = False; \
		receivedHeader = False; \
		headerBits = 0; \
		parityIndex = 0; \
		lastTagID = 0; \
		P1OUT &= ~BIT0; \
		P1OUT &= ~BIT6; \
} \

#define DELAY(delay){ \
		__delay_cycles(delay); \
}\

#define TOGGLE_LED(led){ \
		P1OUT ^= led; \
		DELAY(4000000); \
		P1OUT ^= led; \
		DELAY(4000000); \
}\

#define TOGGLE_LED_FAST(led){ \
		P1OUT ^= led; \
		DELAY(1000000); \
		P1OUT ^= led; \
		DELAY(1000000); \
}\

void setupPins(void)
{
	P1DIR = 0xFF;                             // P1.0,4 output
	P1OUT = 0x00;                             // Clear P1 output latches
	P1SEL = BIT4;                             // P1.4 SMCLK output

	P2DIR = (0xFF & ~input_Pin);
	P2REN = 0x00 | input_Pin;
	P2OUT = 0x00 | input_Pin;
	P2IES = (LOW_to_HIGH & input_Pin);
	P2IE = input_Pin;

	P3DIR = 0xFF;
	P3OUT = 0x00;
}

void nullRegisters(){
	DCOCTL = 0x00;
	BCSCTL1 = 0x00;
	BCSCTL2 = 0x00;
	TA0CTL = TACLR;
}

void setupClock(){
	  BCSCTL1 = CALBC1_8MHZ;					// Set DCO to 8MHz
	  DCOCTL = CALDCO_8MHZ;						// Set DCP to 8MHz
	  BCSCTL2 = SELM_0 | DIVM_0 | DIVS_2;		// Select DCO as MCLK; MCLK / 1 + SMCLK / 4

	  for (i = 0xfffe; i > 0; i--);             // Delay for clock stabilization

	  TOGGLE_LED(red_LED);

	  //IE1 |= OFIE;

	  do
	  {
		  IFG1 &= ~OFIFG;
		  for (i=0xFF; i>0; i--);
	  }while( (IFG1 & OFIFG) != 0);

	  //IE1 &= ~OFIE;

	  TOGGLE_LED_FAST(green_LED);
	  DELAY(1000);
	  TOGGLE_LED_FAST(green_LED);

	  TA0CTL = TASSEL_2 | ID_1 | MC_2;			// Enable capture timer /4 = 2uSecond
	  TA0CCTL0 = CAP | CCIS1 | CM_3;
}

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  nullRegisters();
  setupPins();
  setupClock();
  RESET();

  __bis_SR_register(GIE);       			// enable interrupts
}

//edge detect interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
	//Early escape if wrong pin interrupt
	if((P2IFG & input_Pin) == 0){
		P2IFG = 0;
		return;
	}
	TA0CCTL0 ^= CCIS0;						//Trigger time capture
	P2IFG &= ~input_Pin;					//Clear edge interrupt

	//Start Timer
	if(!isSyncing)
	{
		isSyncing = True;
		TA0R = 0x00;						// Set TimerCoutner 0
		return;
	}

	if ( (TA0CCR0 < TMin) || ( (TA0CCR0 > TMax) && (TA0CCR0 < T2Min) ) )
	{
		return;

	}
	P2IES ^= input_Pin;								//Flip edge detect direction
	P1OUT ^= BIT5;
	TA0R = 0;

	//Start Data Processing
	if(!isSynced){
		if (T2Min < TA0CCR0 && TA0CCR0 < T2Max){
			isSynced = True;
			//currentBit = (P2IN >> bitpos_input_pin) & 0x01;
			currentBit = (P2IES & input_Pin) >> bitpos_input_pin;
			return;
		}
	}
	if(isSynced){
		if (expectShortEdge)
		{
			expectShortEdge = False;
			if (TMin < TA0CCR0 && TA0CCR0 < TMax)
			{
				nextBit = currentBit;
			}else{
				RESET();
				return;
			}
		}
		else
		{
			if (TMin < TA0CCR0 && TA0CCR0 < TMax)
			{
				expectShortEdge = True;
				return;
			}else if (T2Min < TA0CCR0 && TA0CCR0 < T2Max)
			{
				nextBit = currentBit ^ 0x01;
			}else
			{
				RESET();
				return;
			}
		}
	} //End if(isSynced)

	currentBit = nextBit;
	if(!receivedHeader){
		headerBits = headerBits << 1;
		headerBits |= nextBit;
		headerBits &= 511;
		if (headerBits == 511){
			receivedHeader = True;
			inBufferIndex = 0;
		}
		return;
	}else{
		if(inBufferIndex < 50){
			inBuffer[inBufferIndex++] = nextBit;
			if(parityIndex++ == 4){
				parityIndex = 0;
				if(inBuffer[inBufferIndex-5] ^ inBuffer[inBufferIndex-4] ^ inBuffer[inBufferIndex-3] ^ inBuffer[inBufferIndex-2] ^ inBuffer[inBufferIndex-1]){
					RESET();
					return;
				}
				for(i=5; i>1; i--){
					lastTagID = lastTagID << 1;
					lastTagID |= inBuffer[inBufferIndex-i];
				}
			}

		}else{
			if(! (lastTagID == 0 || lastTagID == 1 )){
				if(lastTagID == 285708 || lastTagID == 16282433){
					TOGGLE_LED(green_LED);
				}else{
					TOGGLE_LED(red_LED);
				}
			}else{
				P1OUT = BIT0;
			}
			RESET();
			return;
		}
	}
}
