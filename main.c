#include <msp430.h>
#include <stdint.h>
#include "main.h"


#define NUMBERofDATA32 2

volatile unsigned char RXData = 0;
volatile unsigned char TXData;
volatile unsigned long setPoint = 43737400;//43.7374 Psi
volatile unsigned long Hyst = 500000;//0.5 Psi

volatile unsigned long runSum = 0;
volatile unsigned long avg = 0;



#pragma DATA_ALIGN(Filter_Buffer, 512);
uint8_t  Filter_Buffer[256];
uint16_t *  pFilter_Buffer_Head = Filter_Buffer;
uint8_t * ppFilter_Buffer_Head = (uint8_t *) &pFilter_Buffer_Head;



#pragma DATA_ALIGN(TransmitData, 4);
unsigned char TransmitData[4*NUMBERofDATA32];

unsigned char DataInfo[4];

HiLo32UnionAll result_A;
HiLo32UnionAll result_B;
HiLo16Union PressureRaw;
HiLo16Union TempRaw;

uint8_t indexmax = 8;
uint8_t index = 8;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop WDT

    // Configure GPIO for Timer
    P1DIR |= BIT0;                          //CS bit on P3.6
    P1OUT |= BIT0;

    P3DIR |= BIT6;
    P3OUT |= BIT6;

    //set DOUT direction on pin 2.5
    P2OUT &= ~BIT5;
    P2DIR |= BIT5;

    //P2OUT ^= BIT5; //toggle p2.5

    // Configure GPIO for SPI
    P5SEL1 &= ~(BIT0 | BIT1 | BIT2);        // USCI_B1 SCLK, MOSI, and MISO pin
    P5SEL0 |= (BIT0 | BIT1 | BIT2);
    PJSEL0 |= BIT4 | BIT5;                  // For XT1

    // Configure GPIO for UART
    P2SEL0 &= ~(BIT0 | BIT1);
    P2SEL1 |= (BIT0 | BIT1);                // USCI_A0 UART operation

    PJSEL0 |= BIT4 | BIT5;

     // Disable the GPIO power-on default high-impedance mode to activate
     // previously configured port settings
     PM5CTL0 &= ~LOCKLPM5;

     // Setup XT1
     CSCTL0_H = CSKEY_H;                     // Unlock CS registers
     CSCTL1 = DCOFSEL_6;                     // Set DCO to 8MHz
     CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK; // set ACLK = XT1; MCLK = DCO
     CSCTL3 = DIVA__1 | DIVS__2 | DIVM__2;   // Set all dividers
     CSCTL4 &= ~LFXTOFF;
     do
     {
         CSCTL5 &= ~LFXTOFFG;                // Clear XT1 fault flag
         SFRIFG1 &= ~OFIFG;
     } while (SFRIFG1 & OFIFG);              // Test oscillator fault flag
     CSCTL0_H = 0;


    // Configure USCI_B1 for SPI operation
    UCB1CTLW0 = UCSWRST;                    // **Put state machine in reset**
    UCB1CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB; // 3-pin, 8-bit SPI master
                                            // Clock polarity high, MSB
    UCB1CTLW0 |= UCSSEL__SMCLK;              // ACLK
    UCB1BRW = 20;                         // /2
    //UCB1MCTLW = 0;                          // No modulation
    UCB1CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
 //   UCB1IE |= UCRXIE;                       // Enable USCI_B1 RX interrupt
    TXData = 0x1;                           // Holds TX data


    // Startup clock system with max DCO setting ~8MHz
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL1 = DCOFSEL_3 | DCORSEL;           // Set DCO to 8MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
    CSCTL0_H = 0;                           // Lock CS registers


    // Configure USCI_A0 for UART mode
    UCA0CTLW0 = UCSWRST;                    // Put eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
            // Baud Rate calculation
            // 8000000/(16*9600) = 52.083
            // Fractional portion = 0.083
            // User's Guide Table 21-4: UCBRSx = 0x04
            // UCBRFx = int ( (52.083-52)*16) = 1

        // Baud Rate calculation
                // 8000000/(16*57600) = 8.680555555
                // Fractional portion = 0.680555555
                // User's Guide Table 21-4: UCBRSx = 0xD6
                // UCBRFx = int ( (0.680555555)*16) = 10

    // Baud Rate calculation
            // 8000000/(16*57600) = 8.680555555
            // Fractional portion = 0.680555555
            // User's Guide Table 21-4: UCBRSx = 0xB7
            // UCBRFx = int ( (0.680555555)*16) = 10


    UCA0BRW = 8;                           // 8000000/16/9600
    UCA0MCTLW = UCOS16 | UCBRF_10 | 0xB700;
    UCA0CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
  //  UCA0IE |= UCRXIE;                       // Enable USCI_A0 RX interrupt


    //TA0CCTL0 = CCIE;                        // TACCR0 interrupt enabled
    TA0CCR0 = 16000;
    TA0CTL = TASSEL__SMCLK | MC__UP;        // SMCLK, UP mode

    P3OUT |= BIT6; //keep CS high
    __bis_SR_register( GIE);

    P3OUT |= BIT6;
    __delay_cycles(10000000);



    result_B.uval32 = 0;


    while (1)
      {
        //This loop acts as a delay subroutine, and breaking it is exiting.

        if ( index < indexmax   )
        {

              if (!(UCA0STATW & UCBUSY)){
                  UCA0TXBUF = TransmitData[index];
                  //UCA0TXBUF = DataInfo[index];
                  index++;
              }
        }





        if (TA0CTL & TAIFG){
            TA0CTL &= ~TAIFG;
       //     UCB1IE |= UCTXIE;
         //   __bis_SR_register(LPM0_bits | GIE); // CPU off, enable interrupts
            //UCA0TXBUF = 'A';
            P3OUT &= ~BIT6;
            UCB1TXBUF = 0xff; __delay_cycles(180); PressureRaw.uval8.hi = UCB1RXBUF;
            UCB1TXBUF = 0xff; __delay_cycles(180); PressureRaw.uval8.lo = UCB1RXBUF;
            UCB1TXBUF = 0xff; __delay_cycles(180); TempRaw.uval8.hi = UCB1RXBUF;
            UCB1TXBUF = 0xff; __delay_cycles(180); TempRaw.uval8.lo = UCB1RXBUF;
            P3OUT |= BIT6;
           // __delay_cycles(1000000);

            //CALCULATIONS ON PRESSURE RAW
         //   PressureRaw.uval8.hi = DataInfo[0];
          //  PressureRaw.uval8.lo = DataInfo[1];

            PressureRaw.uval16  -= 1638;

////////////////////////////////////////////////////////////////////////////////////////////////
            runSum -= *pFilter_Buffer_Head;
            *pFilter_Buffer_Head = PressureRaw.uval16; (*ppFilter_Buffer_Head)++;(*ppFilter_Buffer_Head)++;
            runSum += PressureRaw.uval16;
            avg = runSum>>7; // divide by 128

////////////////////////////////////////////////////////////////////////////////////////////////

            MPYS = avg;
            OP2 = 4577;
            __no_operation();
            __no_operation();
            __no_operation();
            __no_operation();
            __no_operation();
            __no_operation();
            __no_operation();
            __no_operation();
            __no_operation();

            result_A.val16.hi = RESHI;
            result_A.val16.lo = RESLO;


            TempRaw.uval16 &=   ~(0x001F); // masking off last 5 bits
            //Offset
            TempRaw.uval16 -=16383;



            MPYS = TempRaw.uval16;
            OP2 = 3051;
            __no_operation();
            __no_operation();
            __no_operation();
            __no_operation();
            __no_operation();
            __no_operation();
            __no_operation();
            __no_operation();
            __no_operation();

            result_B.val16.hi = RESHI;
            result_B.val16.lo = RESLO;

//Temp
            TransmitData[7] = result_B.uval8.hi;
            TransmitData[6] = result_B.uval8.mh;
            TransmitData[5] = result_B.uval8.ml;
            TransmitData[4] = result_B.uval8.lo;



//Pressure
            TransmitData[3] = result_A.uval8.hi;
            TransmitData[2] = result_A.uval8.mh;
            TransmitData[1] = result_A.uval8.ml;
            TransmitData[0] = result_A.uval8.lo;
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

            if(result_A.uval32 >= setPoint){
                P2OUT = 0x00;
            }else if (result_A.uval32 <= (setPoint-Hyst)){
                P2OUT = 0x20;
            }

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
       //     * (uint32_t * ) (&TransmitData[0]) = result_A.uval32;
        //    * (uint32_t * ) (&TransmitData[0]) = result_B.uval32;

            index = 0;
            P1OUT ^= BIT0;                      // led operation
        }
      }
}

