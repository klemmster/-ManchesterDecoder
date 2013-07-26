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

#define LOW_to_HIGH 0x00;
#define HIGH_to_LOW 0x02;

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

volatile unsigned int T = 128;	//Half Bit Rate
volatile unsigned int T2 = 256;  //Bit Rate = 0.000256 = 512 useconds 125kHz / 32

//volatile unsigned char Ttolerance = 1;
volatile unsigned int T2Max = 300; //T2 + Ttolerance;
volatile unsigned int T2Min = 200; //T2 - Ttolerance;

volatile unsigned int TMax = 200; //T + Ttolerance;
volatile unsigned int TMin = 103; //T - Ttolerance;

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

inline void startSync(void);
void setupPins(void);

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  //for (i = 0; i < 0xfffe; i++);             // Delay for XTAL stabilization

  RESET();
  BCSCTL1 = CALBC1_16MHZ;					// Set DCO to 16MHz
  DCOCTL = CALDCO_16MHZ;					// Set DCP to 16MHz

  BCSCTL2 = SELM_0 + DIVM_0 + DIVS_3;		// Select DCO as MCLK; MCLK / 1 + SMCLK / 8

  setupPins();

  __bis_SR_register(GIE);       			// enable interrupts
}

//edge detect interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
	TA0CCTL0 ^= CCIS0;						//Trigger time capture
	P2IFG &= ~BIT1;								//Clear edge interrupt

	//Start Timer
	if(!isSyncing)
	{
		isSyncing = True;
		TA0CTL |= TASSEL_2 | ID_2 | MC_2 | TACLR;			// Enable capture timer /4 = 2uSecond
		TA0R = 0;									// Set TimerCoutner 0
		TA0CCTL0 = CAP + /*SCS*/ + CCIS1 + CM_3;
		return;
	}

	if ( (TA0CCR0 < TMin) || ( (TA0CCR0 > TMax) && (TA0CCR0 < T2Min) ) )
	{
		return;

	}
	P2IES ^= BIT1;								//Flip edge detect direction
	TA0R = 0;
	//P1OUT ^= 0x01;                            // Toggle P1.0

	//Start Data Processing
	if(!isSynced){
		if (T2Min < TA0CCR0 && TA0CCR0 < T2Max){
			isSynced = True;
			currentBit = (P2IN >> 1) & 0x01;
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
//			P1OUT = 0x01;
			if(! (lastTagID == 0 | lastTagID == 1 )){
				if(lastTagID == 285708){
					P1OUT = BIT6;
				}else{
					P1OUT = BIT0;
				}
				__delay_cycles(20000000);
			}else{
				P1OUT = BIT0;
				__delay_cycles(20000000);
			}
			RESET();
			return;
		}
	}
}

void setupPins(void)
{
	P2DIR = 0x0;
	P2REN = 0x0;
	P2IES = LOW_to_HIGH;
	//P2OUT = 0;
	P2IE = BIT1;

	P1OUT = 0x00;                             // Clear P1 output latches
	P1SEL = 0x10;                             // P1.4 SMCLK output
	P1DIR = BIT0 | BIT6;                             // P1.0,4 output

}
