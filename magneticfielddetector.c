#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>

#define SYSCLK 72000000L    // Internal oscillator frequency in Hz
#define BAUDRATE 115200L
#define F_SCK_MAX 2000000L  // Max SCK freq (Hz)

// Pins used by the SPI interface:
//  P0.0: SCK
//  P0.1: MISO
//  P0.2: MOSI
//  P0.3: SS*
#define ADC_CE P0_3

typedef struct {
    unsigned int left;
    unsigned int right;
    unsigned int mid;
} MagneticFieldData;


char _c51_external_startup (void)
{
    // Disable Watchdog with key sequence
    SFRPAGE = 0x00;
    WDTCN = 0xDE; //First key
    WDTCN = 0xAD; //Second key
  
    VDM0CN=0x80;       // enable VDD monitor
    RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD

    // Set clock to 72MHz
    SFRPAGE = 0x10;
    PFE0CN  = 0x20; 
    SFRPAGE = 0x00;
    
    CLKSEL = 0x00;
    CLKSEL = 0x00;
    while ((CLKSEL & 0x80) == 0);
    CLKSEL = 0x03;
    CLKSEL = 0x03;
    while ((CLKSEL & 0x80) == 0);

    P0MDOUT=0b_0001_1101; // SCK, MOSI, P0.3, TX0 are push-pull
    XBR0=0b_0000_0011;    // SPI0E=1, URT0E=1
    XBR1=0b_0000_0000;
    XBR2=0b_0100_0000;    // Enable crossbar and weak pull-ups

    // Configure Uart 0 for printf debugging
    SCON0 = 0x10;
    TH1 = 0x100-((SYSCLK/BAUDRATE)/(12L*2L));
    TL1 = TH1;      
    TMOD &= ~0xf0;  
    TMOD |=  0x20;                       
    TR1 = 1; 
    TI = 1;  

    // SPI initialization for MCP3008
    SPI0CKR = (SYSCLK/(2*F_SCK_MAX))-1;
    SPI0CFG = 0b_0100_0000; 
    SPI0CN0 = 0b_0000_0001; 
    
    return 0;
}

void Timer3us(unsigned char us)
{
    unsigned char i;               
    CKCON0|=0b_0100_0000;
    TMR3RL = (-(SYSCLK)/1000000L); 
    TMR3 = TMR3RL;                 
    TMR3CN0 = 0x04;                 
    for (i = 0; i < us; i++)       
    {
        while (!(TMR3CN0 & 0x80));  
        TMR3CN0 &= ~(0x80);         
    }
    TMR3CN0 = 0 ;                   
}

void waitms (unsigned int ms)
{
    unsigned int j;
    unsigned char k;
    for(j=0; j<ms; j++)
        for (k=0; k<4; k++) Timer3us(250);
}

void SPIWrite (unsigned char x)
{
   SPI0DAT=x;
   while(!SPIF);
   SPIF=0;
}

unsigned int volatile GetADC(char channel)
{
    unsigned int adc;
    ADC_CE=0; 
    
    SPIWrite(0x01);
    adc=SPI0DAT; 
    SPIWrite((channel*0x10)|0x80);  
    adc=((SPI0DAT & 0x03)*0x100);
    SPIWrite(0x55);
    adc+=SPI0DAT;
    
    ADC_CE=1; 
    return adc;
}

void read_magnetic_fields(MagneticFieldData *readings) 
{
    readings->left = GetADC(0);
    readings->right = GetADC(1);
    readings->mid = GetADC(2);
}

// CALL THIS FUNCTION FOR OTHER PARTS OF CODE
void update_sensor_display(void){
	char lcd_buffer[17];
	
	read_magnetic_fields(&fields);
	
	sprintf(lcd_buffer,"L:%04u R:%04u", fields.left, fields.right);
	
	sprintf(lcd_buffer, "Mid: %04u", fields.mid);
	
	printf("Left: %04u | Right: %04u | Mid: %04u\r", fields.left, fields.right, fields.mid);
		
}
