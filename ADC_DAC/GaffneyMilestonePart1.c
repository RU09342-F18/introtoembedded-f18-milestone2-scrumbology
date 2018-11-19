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
//   MSP430F552x Demo - ADC12, Sample A0, Set P1.0 if A0 > 0.5*AVcc
//
//   Description: A single sample is made on A0 with reference to AVcc.
//   Software sets ADC12SC to start sample and conversion - ADC12SC
//   automatically cleared at EOC. ADC12 internal oscillator times sample (16x)
//   and conversion. In Mainloop MSP430 waits in LPM0 to save power until ADC12
//   conversion complete, ADC12_ISR will force exit from LPM0 in Mainloop on
//   reti. If A0 > 0.5*AVcc, P1.0 set, else reset.
//
//                MSP430F552x
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//     Vin -->|P6.0/CB0/A0  P1.0|--> LED
//
//   Bhargavi Nisarga
//   Texas Instruments Inc.
//   April 2009
//   Built with CCSv4 and IAR Embedded Workbench Version: 4.21
//******************************************************************************

//OUR THERMISTER BROWN/BLACK/ORANGE = 3977 = B sub 25/85 value
//A = -14.63
//B = 4791.8
//C = -115334
//D = -3.73 * 10^6
//A1 = 3.354016 * 10 ^ -3
//B1 = 2.569850 * 10 ^ -4
//C1 = 2.620131 * 10 ^ -6
//D1 = 6.383091 * 10 ^ -8
#include <msp430.h>
#include <math.h>

float RT = 0;
float TEMP = 0;

int intTemp = 0;

int initializeADC(void){

    ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
    ADC12CTL1 = ADC12SHP;                     // Use sampling timer
    ADC12IE = 0x01;                           // Enable interrupt
    ADC12CTL0 |= ADC12ENC;
    P6SEL |= 0x01;                            // P6.0 ADC option select
    P1DIR |= 0x01;
}
int initializeUART(void){
    P4SEL |= BIT4+BIT5;                        // P3.4,5 = USCI_A0 TXD/RXD
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;                     // SMCLK
    UCA1BR0 = 6;                              // 1MHz 9600 (see User's Guide)
    UCA1BR1 = 0;                              // 1MHz 9600
    UCA1MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0,
                                              // over sampling
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;
}
int setTemp(void){
    //R = ((VOLTAGE*10000)/(4095 - VOLTAGE));
    RT = (10000*(3.3/4096)*ADC12MEM0)/(3.3 - (ADC12MEM0*(3.3/4096)));

    if(RT >= 5933 && RT <= 15698){ //RANGE 15 deg - 35 deg
         TEMP = -0.0022 * RT + 47.71;
    }else if(RT >= 1919 && RT < 5933){ //RANGE 40 deg - 65 deg
         TEMP = -0.0076 * RT + 78.917;
    }else{//(RT >= 677 && RT < 1919) //RANGE 70 deg - 100 deg
         TEMP = -0.0275 * RT + 116.24;
    }
    intTemp = (int) TEMP;

}

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    initializeADC();
    initializeUART();
         // LPM0, ADC12_ISR will force exit
    while (1)
    {
      ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
      __bis_SR_register(LPM0_bits + GIE);
      setTemp();

    }
}



#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    while(!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = intTemp;
    UCA1IFG &= ~BIT0;//~UCTXIFG;


}
//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void){

  switch(__even_in_range(ADC12IV,34))
  {
  case  6:                                  // Vector  6:  ADC12IFG0
      //VOLTAGE = ADC12MEM0; //Saves the resistance that is gotten from the Voltage divider
      if (ADC12MEM0 >= 0x7ff)                 // ADC12MEM = A0 > 0.5AVcc?
      {
          P1OUT |= BIT0;                        // P1.0 = 1
      }
      else
      {
        P1OUT &= ~BIT0;                       // P1.0 = 0
      }
    __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
  default: break;
  }
}
