#include <msp430.h>
#include <stdio.h>
#include "main.h"

//volatile unsigned int buffer1[50];
//volatile unsigned int buffer2[50];
volatile int debugIndex = 0;

void Init_SYS(void) {
  WDTCTL = WDTPW + WDTHOLD; // stop WDT
  
  BCSCTL1 = CALBC1_16MHZ;   // set DCO 16 MHz
  DCOCTL  = CALDCO_16MHZ;   // set dco step and modulation
  BCSCTL3 |= LFXT1S_2;      // dco
  BCSCTL2 |= SELM_0;        // sel DCO = mclk 
}

void Init_I2C(char addr) {
  P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
  P1SEL2|= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
  UCB0CTL1 |= UCSWRST;                      // Enable SW reset
  UCB0CTL0 = UCMODE_3 + UCSYNC;             // I2C Slave, synchronous mode
  UCB0I2COA = addr;                         // Own Address is 048h
  UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
  UCB0I2CIE |= UCSTPIE + UCSTTIE;           // Enable STT and STP interrupt
  IE2 |= UCB0TXIE + UCB0RXIE;               // Enable RX interrupt
}

void Init_ServoPWM(void) {
  P2DIR |= PIN_S1 + PIN_S2; // Output p2.2 and p2.4                        
  P2SEL |= PIN_S1 + PIN_S2; // TA 1.1 on p2.2 and TA 1.2 on p2.4
  
  TA1CCR0 = PERIOD-1;  // PWM Period of TA 1
  TA1CCTL1 = OUTMOD_7 + CCIE; // CCR1 reset/set  
  TA1CCR1 = 0;         // BLUE duty cycle
  
  TA1CCTL2 = OUTMOD_7 + CCIE; // CCR2 reset/set 
  TA1CCR2 = 0;         // GREEN
  
  TA1CTL = TASSEL_2 + MC_1 + ID_3; // SMCLK, up mode TA 1
  TA1CCTL0 = CCIE;     // interrupt on TA1 overflow
}

void Init_irPWM(void) {
  P1DIR |= PIN_FAN; // Output p2.2 and p2.4                        
  P1SEL |= PIN_FAN; // TA 1.1 on p2.2 and TA 1.2 on p2.4
  
  TA0CCR0 = 10000-1;  // PWM Period of TA 1
  TA0CCTL1 = OUTMOD_7; // CCR1 reset/set  
  TA0CCR1 = 10000;         // BLUE duty cycle
  
  TA0CTL = TASSEL_2 + MC_1; // SMCLK, up mode TA 1
}

void Init_ADC(void) {
  // ADC10SSEL_0 :  ADC10OSC
  // ADC10DIV_0 : ADC10 Clock Divider Select 0. Roughly 5MHz
  // INCH_4 : highest channel we are going to sample
  // CONSEQ_1 :  Sequence of channels
  // SREF_1 : VR+ = VREF+ and VR- = AVSS
  // ADC10SHT_0 : Sample & Hold = 4 x ADC10CLKs
  // REF2_5V : ADC10 Ref 0:1.5V / 1:2.5V;
  ADC10CTL1 = ADC10SSEL_0 + ADC10DIV_0 + INCH_4;                      // AVcc/2
  ADC10CTL0 = SREF_0 + ADC10SHT_3 + REFON + ADC10ON;
  ADC10AE0  = BIT4;  // adc option for channels 3 and 4
  ADC10DTC1 = 0x01; // number of channels to be sampled
}

void Init_LED() {
  P2DIR |= BIT3 + BIT1;
  P1DIR |= BIT5;
}

void Init_Fan() {
  P1DIR |= BIT0;
}

void GetADC(void) {
  ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
  while (ADC10CTL1 & ADC10BUSY);          // ADC10BUSY?
  rawADC = (int)ADC10MEM;                   // ADC10MEM = A11 > 0.65?
}

int SaturateInteger(int value, int lowLimit, int highLimit) {
  if (value >= highLimit) return highLimit;
  else if (value <= lowLimit) return lowLimit;
  else return value;
}

unsigned int Filter(unsigned int sample){
    static unsigned long sum;

    sum -= sum/FILTER_CONST;
    sum += sample;

    return sum/FILTER_CONST;
}

void SwitchLED(int color) {
  switch(color) {
    case OFF:
      P2OUT |= BIT3;
      P2OUT |= BIT1;
      P1OUT |= BIT5;
      break;
    case RED:
      P2OUT &= ~BIT3;
      P2OUT |= BIT1;
      P1OUT |= BIT5;
      break;
    case GREEN:
      P2OUT |= BIT3;
      P2OUT |= BIT1;
      P1OUT &= ~BIT5;
      break;
    case BLUE:
      P2OUT |= BIT3;
      P2OUT &= ~BIT1;
      P1OUT |= BIT5;
      break;
    case YELLOW:
      P2OUT &= ~BIT3;
      P2OUT |= BIT1;
      P1OUT &= ~BIT5;
      break;
  }
}

void SwitchFan(int status) {
  if (status == 1) P1OUT |= BIT0;
  else P1OUT &= ~BIT0;
}

void __delay_cycles(const unsigned long int delay) {
  volatile long int r = delay;
  while (--r);
}

void delay_ms(unsigned int ms ) {
  unsigned int i;
  for (i = 0; i<= ms; i++)
    __delay_cycles(500); //Built-in function that suspends the execution for 500 cicles
}

int main(void) {
  Init_SYS();
  Init_I2C(0x4A);
  Init_irPWM();
  Init_ADC();
  Init_ServoPWM();
  Init_LED();
  Init_Fan();
  
  SwitchFan(1);

  S1REG = 3900; // S1 = vertical
  S2REG = 3000; // S2 = horizontal
  calculatedS1 = S1REG;
  calculatedS2 = S2REG;
  
  __bis_SR_register(GIE);
  
  while (1)
  {
    PRxData = (unsigned char *)RxBuffer;    // Start of RX buffer
    RXByteCtr = 0;                          // Clear RX byte count
    if (PRxData[0] == 0xAA) {
      checkSum = PRxData[5];
      checkSumCalc = PRxData[1] + PRxData[2] + PRxData[3] + PRxData[4];
      if (checkSum - (checkSumCalc&0xff) == 0) {
        S1Ref = SaturateInteger(((PRxData[1] & 0xff) << 8) | (PRxData[2] & 0xff), 0, 1000);
        S2Ref = SaturateInteger(((PRxData[3] & 0xff) << 8) | (PRxData[4] & 0xff), 0, 1000);
      }
      else {
        __no_operation();
      }
      
      if (S1Ref - S1Act <= 0) S1Increment = -resolution;//(S1Ref - S1Act) / resolution;
      else if (S1Ref - S1Act > 0) S1Increment = resolution;
      if (S2Ref - S2Act <= 0) S2Increment = -resolution;//(S2Ref - S2Act) / resolution;
      else if (S2Ref - S2Act > 0) S2Increment = resolution;

      __no_operation();
    }
    else if (PRxData[0] == 0xAB) {
      cpuTemp = (int) PRxData[1];
      if (cpuTemp >= 53) SwitchFan(1);
      else if (cpuTemp <= 48) SwitchFan(0);
      __no_operation();
    }
    else if (PRxData[0] == 0xAC) {
      batteryVoltageH = (int)batteryVoltage;
      batteryVoltageL = (int)((batteryVoltage-batteryVoltageH)*10);
      BatteryTxData[0] = (char)(batteryVoltageH & 0x0F) << 4 | (batteryVoltageL & 0x0F);
        
      //RX = 0;
      //TXByteCtr = 0;
      //PTxData = BatteryTxData;
    }
    else if (PRxData[0] == 0xAD) {
      ledInputRaw = PRxData[1];
      ledInputP = (ledInputRaw & 0xf0) >> 4;
      ledInputN = ~ledInputRaw & 0x0f;
      if ((ledInputP == ledInputN) && (ledInputP <= 10)){
         irREG = 10000-(1000*ledInputP);
         __no_operation();
      }
      __no_operation();
    }
    flag = 1;
    while(flag);
    //__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
                                            // Remain in LPM0 until master
                                            // finishes TX
    __no_operation();                       // Set breakpoint >>here<< and read
  }                                         // read out the RxData buffer
}

//------------------------------------------------------------------------------
// The USCI_B0 data ISR is used to move received data from the I2C master
// to the MSP430 memory.
//------------------------------------------------------------------------------
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
  char RxBuf;
  if(RX == 0){
    UCB0STAT &= ~(UCSTPIFG + UCSTTIFG);
    //IE2 &= ~UCB0RXIE;
    //delay_ms(1);
    UCB0TXBUF = *PTxData++;                   // Transmit data at address PTxData
    TXByteCtr++;                              // Increment TX byte counter
    //delay_ms(2);
    if (TXByteCtr > 0) RX = 1;
    //IE2 |= UCB0RXIE;
  }
  if(RX == 1){
    RxBuf = UCB0RXBUF;                   // Move RX data to address PRxData
    *PRxData++ = RxBuf;
    RXByteCtr++;                              // Increment RX byte count
    if ((RxBuf == 0xAC) && (RXByteCtr == 1)){
      RX = 0;
      TXByteCtr = 0;
      PTxData = BatteryTxData;
    }
//    if (PRxData[0] == 0xAC) {
//      TXByteCtr = 0;
//      //PTxData = [0x55];//BatteryTxData;
//      RX = 0;
//    }
  }
}

//------------------------------------------------------------------------------
// The USCI_B0 state ISR is used to wake up the CPU from LPM0 in order to
// process the received data in the main program. LPM0 is only exit in case
// of a (re-)start or stop condition when actual data was received.
//------------------------------------------------------------------------------
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void) {
  if(RX == 0){
    if (TXByteCtr) {                          // Check TX byte counter
      //__bic_SR_register_on_exit(CPUOFF);    // Exit LPM0 if data was
      flag = 0;
    }
  }                                           // transmitted
  if(RX == 1){                                // received
    if (RXByteCtr) {                          // Check RX byte counter
      //__bic_SR_register_on_exit(CPUOFF);    // Exit LPM0 if data was
      flag = 0;
    }
  }
  UCB0STAT &= ~(UCSTPIFG + UCSTTIFG);         // Clear interrupt flags
}                                          

// Timer1 A0 interrupt service routine
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A (void)
{
  //TA1CTL &= ~TA1IV_TAIFG;
  ITCounter++;
  
  if (ITCounter % 5 == 0) {
    GetADC();
    filteredADC = Filter(rawADC);
    batteryVoltage = (filteredADC * ADCFACTOR);
    
    //batteryVoltageH = (int)batteryVoltage;
    //batteryVoltageL = (int)((batteryVoltage-batteryVoltageH)*10);
    //BatteryTxData[0] = (char)(batteryVoltageH & 0x0F) << 4 | (batteryVoltageL & 0x0F);
    
    if (batteryVoltage < 3.0) LEDColor = BLUE;
    else if (batteryVoltage < 6.8) LEDColor = RED;
    else if (batteryVoltage < 7.2) LEDColor = YELLOW;
    else LEDColor = GREEN;
  }
  
  if (ITCounter % 5 == 0){
    if (HeartBeatCounter == 0) SwitchLED(LEDColor);
    else if (HeartBeatCounter == 3) SwitchLED(OFF);
    else if (HeartBeatCounter == 5) SwitchLED(LEDColor);
    else if (HeartBeatCounter == 8) SwitchLED(OFF);
    else if (HeartBeatCounter == 12) HeartBeatCounter = -1;
    HeartBeatCounter++;
  }
  
  if (ITCounter % 1 == 0){
    if (S1Act != S1Ref) {
      S1Act+=S1Increment;
    }
    if (((S1Act < S1Ref + S1Increment *2) && (S1Act > S1Ref - S1Increment*2)) || ((S1Act < S1Ref - S1Increment *2) && (S1Act > S1Ref + S1Increment*2))) {
      S1Act = S1Ref;
    }
    if (((S1Act <= S1Ref + resolution) && (S1Act >= S1Ref - resolution)) || ((S1Act <= S1Ref - resolution) && (S1Act >= S1Ref + resolution))) {
      S1Act = S1Ref;
    }

    if (S2Act != S2Ref) {
      S2Act+=S2Increment;
    }
    if (((S2Act < S2Ref + S2Increment *2) && (S2Act > S2Ref - S2Increment*2)) || ((S2Act < S2Ref - S2Increment *2) && (S2Act > S2Ref + S2Increment*2))) {
      S2Act = S2Ref;
    }
    if (((S2Act <= S2Ref + resolution) && (S2Act >= S2Ref - resolution)) || ((S2Act <= S2Ref - resolution) && (S2Act >= S2Ref + resolution))) {
      S2Act = S2Ref;
    }
    
    calculatedS1 = 2020 + (int)(S1Act * 2);
    calculatedS2 = 1700 + (int)(S2Act * 2.5);
  }
}

// Timer1 A0 interrupt service routine
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer_A_CCR1 (void)
{
  int IVValue = TA1IV;
  if (IVValue == TA1IV_TACCR1) {
    debugValue = IVValue;
    S1REG = calculatedS1;
  }
  else if (IVValue == TA1IV_TACCR2) {
    debugValue = IVValue;
    S2REG = calculatedS2;
  }
  else {
    debugValue = IVValue;
    __no_operation();
  }
  //switch(TA1IV) {
  //  case 2: S1REG = calculatedS1;
  //  case 4: S2REG = calculatedS2;
  //  case 7: __no_operation();
  //}

  //TA1CTL &= ~TAIFG;
}
  
#pragma vector= TRAPINT_VECTOR
__interrupt void TRAPINT_ISR(void)
{
  __no_operation();
}
  
  
  

  
  
  
  
  