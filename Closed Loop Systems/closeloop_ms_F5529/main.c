#include <msp430.h>
#include <math.h>

/*
 * Authors: Tiernan Cuesta and Kevin Purcell
 * Version 1
 * Closed Loop Systems - Embedded Systems
 * Controlling a fan to keep a system at a specified temperature
*/

// Variable Initializations
unsigned int ADC;
float resolution, logR, kelvin, temp;
char space = ' ';
volatile float target = 25;
volatile float diff;


void configurePWM() {
   // Sets P1.2 as the output pin
   P1DIR |= BIT2;                                       // Sets P1.2 as output driver for pwm for fan speed control
   P1SEL |= BIT2;                                       // Selects the port 1.2 as the timer A output
   TA0CTL = TASSEL_2 | MC_1 | TACLR;                    // Sets timerA_0 to SMCLK, up-mode, clears the register
   TA0CCR0 = 255;                                       // Sets CCR0 max pwm
   TA0CCR1 = 30;                                        // Sets CCR1 to initial value;
   TA0CCTL1 = OUTMOD_7;                                 // Output mode 7 for set/reset
}

void configureUARTLED()
{
   // Onboard LED for indicator that the ADC is sampling and the value is being sent to RealTerm
   P4DIR |= BIT7;
   P4OUT &= ~BIT7;
}

void configureUART() {
   P4SEL |= BIT5 | BIT4;                                // Enables RX and TX buffer
   UCA1CTL1 |= UCSWRST;                                 // Software reset enable
   UCA1CTL1 |= UCSSEL_1;                                // USCI clock source select - ACLK
   UCA1BR0 = 3;                                         // Baud rate clock divider1 for 9600 BR
   UCA1BR1 = 0;                                         // Baud rate clock divider2 for 9600 BR
   UCA1MCTL |= UCBRS_3 | UCBRF_0;                       // First and second stage modulation for higher accuracy baud rate
   UCA1IE |= UCTXIE;                                    // Enables Transfer buffer interrupt
   UCA1IE |= UCRXIE;                                    // Enables Receiver buffer interrupt
   UCA1IFG &= ~UCRXIFG;                                 // Clears Receiver buffer interrupt flag
}

void configureADC() {
   P6DIR &= ~BIT0;                                      // Sets P6.0 to input direction for ADC_12A input from voltage divider
   P6SEL |= BIT0;                                       // Sets P6.0 as the input for ADC12_A sample and conversion
   ADC12CTL2 = ADC12RES_2;                              // AD12_A resolution set to 12-bit
   ADC12CTL1 = ADC12SHP;                                // ADC12_A sample-and-hold pulse-mode select - SAMPCON signal is sourced from the sampling timer
   // ADC12_A Control Register 0 - 1024 cycles in a sampling period - Auto Trigger - Ref Volt off - Conversion overflow enable - Conversion enable - Start Sampling
   ADC12CTL0 = ADC12SHT1_15 | ADC12SHT0_15 | ADC12MSC | ADC12ON | ADC12TOVIE | ADC12ENC | ADC12SC;
   ADC12IE = ADC12IE0;                                  // Enables ADC12 interrupt
   ADC12IFG &= ~ADC12IFG0;                              // Clears ADC12 interrupt flag
}

int main(void)
{
   UCSCTL4 = SELA_0;                                    // Enables UART ACLK (32.768 kHz signal)
   WDTCTL = WDTPW | WDTHOLD;                            // Stop watchdog timer

   configurePWM();
   configureUARTLED();
   configureUART();
   configureADC();

   __bis_SR_register(GIE);                              // Enables Global Interrupt - ADC/UART interrupt support
   while (1);
}

// Helpful TI resource code - ADC vector interrupt protocol
#pragma vector=ADC12_VECTOR
__interrupt void newADC(void)
{
   switch (ADC12IV)
{
   case 6:
   {
        // Temperature calculations
        ADC = ADC12MEM0;                                // Sets the digital voltage value to float ADC for math
        resolution = (ADC * 10000.0) / (4095.0 - ADC);  // Calculates the nominal resistance of the thermistor
        logR = log(resolution / 10000.0);               // Extra math step for next line to be more simple - math provided by NTCLE100E3 thermistor datasheet
        // Calculates temperature reading in Kelvin - math provided by NTCLE100E3 thermistor datasheet
        kelvin = 1.0/(0.003354016 + 0.000256985 * logR + 0.000002620131 * logR * logR + 0.00000006383091 * logR * logR * logR);
        temp = (kelvin - 273.0);                        // Converts Kelvin to Celcius
        UCA1TXBUF = (int) temp;                         // Sends the temperature in Celcius to the UART buffer as an integer to be recorded on RealTerm
        P4OUT ^= BIT7;                                  // Toggles On-Board LED P4.7 as an indicator that the ADC sampled at that time
        ADC12CTL0 |= ADC12SC;                           // Triggers Analog to Digital Conversion

        diff = temp - target;                           // Calculates the difference in current temperature and target temperature of the thermistor
        volatile float P = diff * 75;                   // Proportion control constant
        // Proportion control functions
              if (P == 0)
              {
                  TA0CCR1 = 0;
              }

              else if (P >= 255)
              {
                 TA0CCR1 = 255;
              }

              else if (P < 0)
              {
                TA0CCR1 = 0;
              }
              else
              {
              TA0CCR1 = P;
              }
                break;
   }
   default:// Do nothing
 }
}
// UART interrupt vector protocol
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
  target = UCA1RXBUF;                                   // Receives value from UART and sets it to the new target temperature
}

