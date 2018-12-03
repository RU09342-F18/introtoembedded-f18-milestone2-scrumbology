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

#include <msp430.h>
#include <math.h>

int isDataSent = 0;     //Variable that keeps the system paused until the first data is sent
float RT = 0;           //Variable that stores the resistance from the voltage divider
float currentTemp = 0;  //Variable that stores the current temp as a float
int goalTemp = 0;       //Variable that stores the goal temp to go to
int deltaTemp = 0;      //Variable that stores the difference between goal temp and current temp
int additionalCold = 0; //Variable that is used to make the fan spin a little faster the colder the goaltemp is
int intCurrentTemp = 0; //Variable that stores in value of current temp

void controlFan(){                              //used to set the PWM of the fan based on deltaTemp
    deltaTemp = goalTemp - intCurrentTemp;      //this initalizes the delta and also updates the value everytime the method is run

    if(intCurrentTemp < 46){                    //if the current temp is <= 45, make the fan blow a little harder
        additionalCold = 20;
    }else{                                      //else dont do anything
        additionalCold = 0;
    }

    if(deltaTemp < 0){                                   //needs to cool down, so more fan
        if((deltaTemp >= -75) && (deltaTemp <= -3)){     //big temp difference, use 100% duty cycle fan speed
            TA0CCR1 = 255;
        }else if((deltaTemp > -3) && (deltaTemp <= -2)){ //med temp difference, use 50% duty cycle fan speed
            TA0CCR1 = 150 + additionalCold;
        }else if((deltaTemp > -2) && (deltaTemp <= -1)){ //low temp difference, use 25% duty cycle fan speed
            TA0CCR1 = 70 + additionalCold;
        }else{                                           //really low temp difference, use 12.5% duty cycle fan speed
            TA0CCR1 = 70 + additionalCold;
        }
    }else if(deltaTemp > 0){                             //need to heat up, so no fan
        TA0CCR1 = 0;

    }else{                                               //change = 0
        TA0CCR1 = 0;                                     //do nothing
    }

}

void initializePWM() {                        //intialize the PWM using timer A0
   P1OUT &= ~BIT2;                            // initially sets the output to 0
   P1SEL |= BIT2;                             // sets P1.2 to output
   P1DIR |= BIT2;                             // sets P1.2 to TA CCR1 Capture

   TA0CTL = TASSEL_2 + ID_0 + MC_1 + TACLR;   // Configures TA0 to utilize SMCLK, an internal divider of 1, sets the clock to up mode, and initially clears the clock
   TA0CCR0 = 255;

   TA0CCR1 = 0;                               //used to change the duty cycle

   TA0CCTL1 = OUTMOD_7;                       // Sets TA0 to set/reset
}
int initializeADC(void){                      //initialize ADC
    ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
    ADC12CTL1 = ADC12SHP;                     // Use sampling timer
    ADC12IE = 0x01;                           // Enable interrupt
    ADC12CTL0 |= ADC12ENC;                    // Enable conversion
    P6SEL |= 0x01;                            // P6.0 ADC option select
    P1DIR |= 0x01;
}
int initializeUART(void){                     //initialize UART
    P4SEL |= BIT4+BIT5;                       // P4.4,5 = USCI_A0 TXD/RXD
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;                     // SMCLK
    UCA1BR0 = 6;                              // 1MHz 9600 (see User's Guide)
    UCA1BR1 = 0;                              // 1MHz 9600
    UCA1MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0,
                                              // over sampling
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                         // Enable Interupts
}
int setTemp(void){                            //Used to calculate currentTemp using ADC reading (voltage)
    RT = (10000*(3.3/4096)*ADC12MEM0)/(3.3 - (ADC12MEM0*(3.3/4096))); //Conversion from Analog value to digital value and from voltage to resistance

                                                        //used a linear representation of the Hart Equation to get different linear trend lines to base the conversion from resistance to temp on

    if(RT >= 5933 && RT <= 15698){                      //RANGE 15 deg - 35 deg
         currentTemp = -0.0022 * RT + 47.71;
    }else if(RT >= 1919 && RT < 5933){                  //RANGE 40 deg - 65 deg
         currentTemp = -0.0076 * RT + 78.917;
    }else{                  //(RT >= 677 && RT < 1919)  //RANGE 70 deg - 100 deg
         currentTemp = -0.0275 * RT + 116.24;
    }
    intCurrentTemp = (int) currentTemp;                 //casts as int to get int representation
}
int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;               // Stop WDT
    initializeADC();                        // Init ADC
    initializeUART();                       // Init UART
    initializePWM();                        // Init PWM
                                            // LPM0, ADC12_ISR will force exit
    while (1)
    {
      ADC12CTL0 |= ADC12SC;                 // Start sampling/conversion
      __bis_SR_register(LPM0_bits + GIE);   // Enable global interupts and Low Power Mode
      setTemp();                            //calls function to calculate temp from ADC reading
    }
}

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)          //UART interupt vector
{
    while(!(UCA1IFG & UCTXIFG));            //until these flags are triggered dont do anything
    UCA1TXBUF = intCurrentTemp;             //Transmit buffer gets the current temp as an int
    goalTemp = UCA1RXBUF;                   //goalTemp gets whatever is coming in from the recieve buffer
    isDataSent = 1;                         //sets this variable to 1 meaning goalTemp have been recieved
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)            //ADC interupt
{
  switch(__even_in_range(ADC12IV,34))       //Switch case statement for ADC12IV (interupt vector)
  {
  case  6:                                  // Vector  6:  ADC12IFG0
                                            //Saves the resistance that is gotten from the Voltage divider
      if(isDataSent == 1){                  //Condition for if data has been sent
          controlFan();                     //run controlFan()
      }

      if (ADC12MEM0 >= 0x7ff)               // ADC12MEM = A0 > 0.5AVcc?
      {
          P1OUT |= BIT0;                    // P1.0 = 1
      }
      else
      {
        P1OUT &= ~BIT0;                     // P1.0 = 0
      }
    __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
  default: break;
  }
}
