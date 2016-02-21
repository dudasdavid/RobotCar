#define PERIOD       40000 //25Khz at 16Mhz clk, psuedo-period
#define FILTER_CONST 4
#define ADCFACTOR    0.0103

#define RED    1
#define GREEN  2
#define BLUE   3
#define YELLOW 4
#define OFF    0

//External hardware configuration
#define PIN_FAN BIT2	
#define PIN_S1	BIT4	
#define PIN_S2  BIT2

//Hardware configuration
#define S1REG 	TA1CCR1
#define S2REG 	TA1CCR2
#define irREG 	TA0CCR1

volatile int flag = 1;

volatile unsigned int calculatedS1 = 0;
volatile unsigned int calculatedS2 = 0;

volatile int debugValue = 0;

unsigned char *PRxData;                     // Pointer to RX data
unsigned char *PTxData;
volatile unsigned char RxBuffer[128];       // Allocate 128 byte of RAM

volatile unsigned char TXByteCtr, RXByteCtr;
volatile unsigned char RX = 1;

char SLV_Data = 0x11;

volatile double batteryVoltage = 0;
int batteryVoltageH;
int batteryVoltageL;

unsigned char ledInputRaw = 0;
unsigned char ledInputP = 0;
unsigned char ledInputN = 0;

unsigned char TxData[] =              // Table of data to transmit
  {0x00, 0x00};

unsigned char BatteryTxData[] = {0x00};            // Table of data to transmit

int checkSum;
int checkSumCalc;

int S1Ref = 800;
int S2Ref = 500;
int S1Act = 800;
int S2Act = 500;
int S1Increment = 0;
int S2Increment = 0;
int resolution = 25;

int ITCounter = 0;
int cpuTemp = 40;
int HeartBeatCounter = 0;
volatile unsigned int rawADC;
volatile unsigned int filteredADC = 0;
char buffer[10];

unsigned int kaka = 1023;

int LEDColor = BLUE;