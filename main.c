#include <msp430.h> 
#include <msp430fr5994.h>
#include <stdint.h>

// I2C control variables
int count = 0;
int I2Cmode;
int Calibrate = 0;
int Configure = 1;
int CollectData = 2;

// I2C data variables
int data_in = 0;
int pre_voltage = 0;

// ADC data variables
float pre_ADC_Value = 0;
float ADC_Value = 0;

// timer variables
int quarterSec = 0;
int z = 0;
int q = 0;
int resetHome = 0;
int b = 0;

// data
float pow1 = 0;
float pow2 = 0;
float pow3 = 0;
float pow4 = 0;
float volt1 = 0;
float volt2 = 0;
float volt3 = 0;
float volt4 = 0;

// calibration values
uint8_t calibrate_packet[] = {0x05, 0x43, 0x00};     // calibration packet for watt meter (0x05)
uint8_t config_packet[] = {0x00, 0x1c, 0x7f};        // configuration packet for power/current (0x00)
uint8_t power_packet[] = {0x02};                     // register address; voltage:(0x02), power:(0x03), current:(0x04)

// I2C Wattmeter addresses
uint8_t WATTMETER_1 = 0x45;         //[SUBSYSTEM NAME HERE]
uint8_t WATTMETER_2 = 0x44;         //[SUBSYSTEM NAME HERE]
uint8_t WATTMETER_3 = 0x41;         //[SUBSYSTEM NAME HERE]
uint8_t WATTMETER_4 = 0x40;         //[SUBSYSTEM NAME HERE]

// Initiate Internal DCO (Digital Crystal Oscillator) and Clocks
void initClockTo16MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL1 = DCOFSEL_0;                     // Set DCO to 1MHz

    // Set SMCLK = MCLK = DCO, ACLK = LFXTCLK (VLOCLK if unavailable)
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;

    // Per Device Errata set divider to 4 before changing frequency to
    // prevent out of spec operation from overshoot transient
    CSCTL3 = DIVA__4 | DIVS__4 | DIVM__4;   // Set all corresponding clk sources to divide by 4 for errata
    CSCTL1 = DCOFSEL_4 | DCORSEL;           // Set DCO to 16MHz

    // Delay by ~10us to let DCO settle. 60 cycles = 20 cycles buffer + (10us / (1/4MHz))
    __delay_cycles(60);
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers to 1 for 16MHz operation
    CSCTL0_H = 0;                           // Lock CS registers
}

// Initiate Pin Functionality
void initGPIO()
{
    // What are these? no one knows [FIX THIS]
    P1OUT &= ~0x0002;       // Clear P1.1 output latch for a defined power-on state
    P1DIR |= 0x0002;        // Set P1.1 to output direction
    P3DIR &= ~0x000F;       // set pins P3.0-P3.1 for input

    // I2C pins
    P7SEL0 |= BIT0 | BIT1;
    P7SEL1 &= ~(BIT0 | BIT1);

    // ADC PINS
    P3SEL0 |= BIT0 | BIT1 | BIT2 | BIT3;
    P3SEL1 |= BIT0 | BIT1 | BIT2 | BIT3;

    // UART pins
    P6SEL0 |= BIT0 | BIT1;
    P6SEL1 &= ~(BIT0 | BIT1);

    // sun timer clock pin
    P6DIR |= BIT2;
    P6OUT &= ~BIT2;

    // homing signal clock pin
    P6DIR |= BIT3;
    P6OUT &= ~BIT3;

    // Disable the GPIO power-on default high-impedance mode to activate
    PM5CTL0 &= ~LOCKLPM5;   //turn on I/O
}

// Initiate I2C Communication
void initI2C()
{
    UCB2CTLW0 = UCSWRST;                      // Enable SW reset
    UCB2CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC; // I2C master mode, SMCLK
    UCB2BRW = 160;                            // fSCL = SMCLK/160 = ~100kHz
    UCB2CTLW1 = UCASTP_2;                     // automatic STOP assertion
    UCB2CTLW0 &= ~UCSWRST;                    // Clear SW reset, resume operation
}

// Calibrate Wattmeter for [FIX THIS]
void calibrateWattmeter(uint8_t addr)
{
    UCB2I2CSA = addr;                       // Slave Address
    //UCB2TBCNT = sizeof(calibrate_packet);   // Write sizeof() bytes of data
    UCB2TBCNT = 3;
    UCB2CTLW0 |= UCTR;                      // I2C TX
    UCB2IE |= UCTXIE;                       // Local enable for TX IRQ
    __enable_interrupt();                   // Enable interrupt
    UCB2CTLW0 |= UCTXSTT;                   // Enable start condition
    while((UCB2IFG & UCSTPIFG) == 0){}      // Wait for stop flag bit
    UCB2IFG &= ~UCSTPIFG;                   // Clear stop flag
    count = 0;              //reset for next variable
}

// Configure Wattmeter for [FIX THIS]
void configWattmeter(uint8_t addr)
{
    UCB2I2CSA = addr;                           // Slave Address
    UCB2TBCNT = sizeof(config_packet);          // write/read bytes of data
    UCB2CTLW0 |= UCTR;                          // I2C TX
    UCB2IE |= UCTXIE;                         //local enable for TX IRQ
    __enable_interrupt();                       //enable maskables
    UCB2CTLW0 |= UCTXSTT;                      //start condition
   while((UCB2IFG & UCSTPIFG) == 0){}           // wait for stop flag bit
    UCB2IFG &= ~UCSTPIFG;                        // clear stop flag
    count = 0;              //reset for next variable
}

// I2C Write to Register with input Address (uint8_t addr)
void writeI2C(uint8_t addr)
{
    UCB2I2CSA = addr;                       // Slave Address
    UCB2TBCNT = sizeof(power_packet);       // Write sizeof() bytes of data
    UCB2CTLW0 |= UCTR;                      // I2C TX
    UCB2IE |= UCTXIE;                       // Local enable for TX IRQ
    __enable_interrupt();                   // Enable interrupt
    UCB2CTLW0 |= UCTXSTT;                   // Enable start condition
    while((UCB2IFG & UCSTPIFG) == 0){}      // Wait for stop flag bit
    UCB2IFG &= ~UCSTPIFG;                   // Clear stop flag
    count = 0;              //reset for next variable
}

// Calculate Voltage from measured value (pre_voltage)
float calcVolt(float pre_voltage)
{
    float volt = pre_voltage / 65535 * 32; // / 65535 * 32 for voltage;
    return volt;
}

// I2C Read from Register with input Address (addr)
float readI2C(uint8_t addr)
{
    UCB2I2CSA = addr;                       // Slave Address
    UCB2TBCNT = 2;                          // Read 2 bytes of data
    UCB2CTLW0 &= ~UCTR;                     // I2C RX
    UCB2IE |= UCRXIE;                       // Local enable for RX IRQ
    __enable_interrupt();                   // Enable interrupt
    UCB2CTLW0 |= UCTXSTT;                   // Enable start condition
    while((UCB2IFG & UCSTPIFG) == 0){}      // Wait for stop flag bit
    UCB2IFG &= ~UCSTPIFG;                   // Clear stop flag
    count = 0;              //reset for next variable
    float voltage = calcVolt(pre_voltage);        // calculate and return true value
    return voltage;
}

// Initiate ADC - Analog to Digital Converter
void initADC()
{
    ADC12CTL0 &= ~ADC12SHT0;                // Set conv clock cycle = 16
    ADC12CTL0 |= ADC12SHT0_2 | ADC12ON;     // Turn on ADC, ADC12SHT0 = 10
    ADC12CTL1 |= ADC12SSEL_2 | ADC12SHP;    // Chooses SMCLK as sample signal source
    ADC12CTL2 &= ~ADC12RES;                 // Clear resolution
    ADC12CTL2 |= ADC12RES_2;                // 12 bit resolution
}

// Calculate Voltage from measured ADC value (pre_ADC_Value)
float calcADC(float ADC)
{
    float ADC_volt = ADC / 4096 * 3.3 * 1.5;
    return ADC_volt;
}

// ADC read voltage from input pin (input_pin)
float readADC(uint8_t input_pin)
{
    ADC12MCTL0 &= ~0xffff;                  // Clear pin
    ADC12MCTL0 |= input_pin;                // ADC input pin
    //ADC12MCTL0 |= ADC12VRSEL_1; volt ref
    ADC12CTL0 |= ADC12ENC | ADC12SC;        // Enable/Start conversion
    while((ADC12IFGR0 & ADC12IFG0 ) == 0);  // Wait for stop flag bit
    pre_ADC_Value = ADC12MEM0;              // Voltage value collected from ADC
    ADC_Value = calcADC(pre_ADC_Value);     // calculate and return true voltage
    return ADC_Value;
}

// Initiate UART Communication
void initUART()
{
    UCA3CTLW0 |= UCSWRST;                   // Put UART into SW reset
    UCA3CTLW0 |= UCSSEL__SMCLK;             // Choose SMCLK for UART A1
    UCA3BRW = 104;                          // Pre-scalar 1MHz/(104)
    UCA3MCTLW |= UCOS16 | UCBRF3 | UCBRS5;  // ALL SETTINGS FOR 9600 baud
    UCA3CTLW0 &= ~UCSWRST;                  // Clear reset thing IDK
}

// UART send data (data)
void sendUART(float p1, float p2, float p3, float p4, float v1, float v2, float v3, float v4)
{
    uint8_t a[24];
    a[0] = 101;                 // label1
    a[1] = p1;                  // number before decimal
    a[2] = p1*100 - a[1]*100;   // number after decimal
    a[3] = 102;
    a[4] = p2;
    a[5] = p2*100 - a[4]*100;
    a[6] = 103;
    a[7] = p3;
    a[8] = p3*100 - a[7]*100;
    a[9] = 104;
    a[10] = p4;
    a[11] = p4*100 - a[10]*100;
    a[12] = 105;
    a[13] = v1;
    a[14] = v1*100 - a[13]*100;
    a[15] = 106;
    a[16] = v2;
    a[17] = v2*100 - a[16]*100;
    a[18] = 107;
    a[19] = v3;
    a[20] = v3*100 - a[19]*100;
    a[21] = 108;
    a[22] = v4;
    a[23] = v4*100 - a[22]*100;

    int i;
    int j;
    for(i = 0; i < 24; i++)
    {
        UCA3TXBUF = a[i];                       // Send data over UART
        for(j = 0; j<15000; j=j+1){}             // Delay loop
    }
    for(j = 0; j<20000; j=j+1){}                //whoops more delay loop
}

// Initiate TimerB0
void initTimerB0(void)
{
    TB0CTL |= TBCLR;                        // Reset timer
    TB0CTL |= TBSSEL__ACLK;                 // Select ACLCK
    TB0CTL |= MC__CONTINUOUS;               // Continuous mode
    TB0CTL |= CNTL_1;                       // 12-bit length
    TB0CTL |= TBIE;                         // Local enable for TB0 overflow
    __enable_interrupt();                   // Enable interrupt
    TB0CTL &= ~TBIFG;                       // Clear IRQ Flag
}

// Calculate Current from measured value (pre_voltage)
//float calcCurrent(float pre_voltage)
//{
//    float curr = pre_voltage / 65535 * 1;
//    return curr;
//}

// MAIN function
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	// Initiate Peripherals
	initClockTo16MHz();
	initGPIO();
	initI2C();
	initADC();
	initUART();
	initTimerB0();

	I2Cmode = Calibrate;                // Calibrate Wattmeter Mode
	//calibrateWattmeter(WATTMETER_1);
	//calibrateWattmeter(WATTMETER_2);
	//calibrateWattmeter(WATTMETER_3);
	//calibrateWattmeter(WATTMETER_4);

	I2Cmode = Configure;                // Configure Wattmeter Mode
	//configWattmeter(WATTMETER_1);
	//configWattmeter(WATTMETER_2);
	//confi gWattmeter(WATTMETER_3);
	//configWattmeter(WATTMETER_4);

	I2Cmode = CollectData;              // Collect Data Mode
	while(1)
	{
	    // read wattmeter data
	    writeI2C(WATTMETER_1);            // Wattmeter 1 //[Sensors]
	    pow1 = readI2C(WATTMETER_1);
	    writeI2C(WATTMETER_2);          // Wattmeter 2 //[SUBSYSTEM NAME HERE]
	    pow2 = readI2C(WATTMETER_2);
	    writeI2C(WATTMETER_3);          // Wattmeter 3 //[SUBSYSTEM NAME HERE]
	    pow3 = readI2C(WATTMETER_3);
	    writeI2C(WATTMETER_4);          // Wattmeter 4 //[SUBSYSTEM NAME HERE]
	    pow4 = readI2C(WATTMETER_4);

	    volt1 = readADC(ADC12INCH_12);            //ADC input = A12 (P3.0)
	    volt2 = readADC(ADC12INCH_13);            //ADC input = A13 (P3.1)
	    volt3 = readADC(ADC12INCH_14);            //ADC input = A13 (P3.2)
	    volt4 = readADC(ADC12INCH_15);            //ADC input = A13 (P3.3)

	    // test values [DELETE THIS]
	    // pow1 = 1.11;
	    // pow2 = 2.22;
	    // pow3 = 3.33;
	    // pow4 = 4.44;
	    // volt1 = 5.55;
	    // volt2 = 6.66;
	    // volt3 = 7.77;
	    // volt4 = 8.88;

	    sendUART(pow1, pow2, pow3, pow4, volt1, volt2, volt3, volt4);
	    __delay_cycles(10000);
	    P1OUT ^= 0x0002; // toggle P1.0 output (LED on/off)
	    __delay_cycles(5000); // delay for some time
	}
	return 0;
}

// ISR TimerB0 (5min track sun timer/Homing Signal)
#pragma vector = TIMER0_B1_VECTOR
__interrupt void ISR_TB0_Overflow(void)
{
    quarterSec = quarterSec + 1;
    /*if((volt1 < 0.01) && (volt2 < 0.01) && (volt3 < 0.01) && (volt4 < 0.01) && (z == 1))
    {
        resetHome = resetHome + 1;
        if((resetHome > 200) && (q < 1) && (b < 1))
        {
            P6OUT |= BIT3;
            resetHome = 0;
            q = 1;
            quarterSec = 0;
            b = 1;
        }
        if((quarterSec > 525) && (q >= 1))
        {
            P6OUT &= ~BIT3;
            resetHome = 0;
            q = 0;
            quarterSec = 0;
        }
        if((quarterSec > 1024) || (resetHome > 1024))
        {
            quarterSec = 0;
            resetHome = 0;
        }
    }
    else*/
    //{
        if(quarterSec >= 525)           //525->1min/ 2626->5min/
        {
            P6OUT |= BIT2;              // turn on signal
            z = 1;                      // signal is on
            quarterSec = 0;             // reset quarterSec timer (interrupt rate)
        }
        if((quarterSec >= 260) && (z == 1))     //30 sec
        {
            P6OUT &= ~BIT2;             // turn off signal
            z = 0;                      // signal is off
            quarterSec = 0;
        }
        b = 0;                          // Day mode
    //}
    TB0CTL &= ~TBIFG;                   // clear IRQ flag
}

// ISR I2C
#pragma vector = EUSCI_B2_VECTOR
__interrupt void EUSI_B2_I2C_ISR(void) //__interrupt void USCI_B2_ISR(void) //
{
    switch(UCB2IV)
    {
    case 0x16:          //ID 16: RXIFG
        data_in = UCB2RXBUF;
        if (count == 0)
        {
            pre_voltage = data_in << 8;
            count = count + 1;
        }
        if (count == 1)
        {
            pre_voltage = pre_voltage + data_in;
        }
        break;
    case 0x18:          //ID 18: TXIFG
        switch(I2Cmode)
        {
        case 0:
            if (count == (sizeof(calibrate_packet)-1))
            {
                UCB2TXBUF = calibrate_packet[count];
                count = 0;
            }
            else
            {
                UCB2TXBUF = calibrate_packet[count];
                count = count + 1;
            }
            break;
        case 1:
            if (count == (sizeof(config_packet)-1))
            {
               UCB2TXBUF = config_packet[count];
                count = 0;
            }
            else
            {
                UCB2TXBUF = config_packet[count];
                count = count + 1;
            }
            break;
        case 2:
            if (count == (sizeof(power_packet)-1))
            {
                UCB2TXBUF = power_packet[count];
                count = 0;
            }
            else
            {
                UCB2TXBUF = power_packet[count];
                count = count + 1;
            }
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

