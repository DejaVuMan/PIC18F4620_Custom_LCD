/*
 * File:   newmain.c
 * Author: ADS
 *
 * Created on May 6, 2022, 8:29 PM
 */

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF // was ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF

#include <xc.h>
#include "pic18f4620.h"

// R/W Goes to Ground
#define LENA  PORTEbits.RE1 // E (Enable?)
#define LDAT  PORTEbits.RE2 // RS
#define LPORT PORTD
#define AINPUTS 0xffcf

#define L_ON	0x0F
#define L_OFF	0x08
#define L_CLR	0x01
#define L_L1	0x80
#define L_L2	0xC0
#define L_CR	0x0F		
#define L_NCR	0x0C	

#define L_CFG   0x38



// prototypes
void delay(unsigned int ms);

void delay(unsigned int ms)
{
    unsigned int i;
    unsigned char j;
    
 for (i =0; i< ms; i++)
 {
 
  for (j =0 ; j < 200; j++)
   {
      Nop();
      Nop();
      Nop();
      Nop();
      Nop();
   }
 }
}

void lcd_wr(unsigned char val)
{
  LPORT=val;
}

void lcd_cmd(unsigned char val)
{
    // E = LENA
    // RS = LDAT
	LENA=1;
        lcd_wr(val);
        LDAT=0;
        //delay(3);
        LENA=0;
        //delay(3);
	LENA=1;
}

void lcd_custom_command(unsigned char val)
{
    LDAT = val; // Send data to PORT as cmd for LCD
    LDAT = 0;
    LENA = 1;
    Nop();
    LENA = 0;
    delay(3);
}
 
void lcd_dat(unsigned char val)
{
	LENA=1;
        lcd_wr(val);
        LDAT=1;
        //delay(3);
        LENA=0;
        //delay(3);
	LENA=1;
}

void lcd_init(void)
{
	LENA=0;
	LDAT=0;
	delay(20);
	LENA=1;
	
	lcd_cmd(L_CFG); // 0x38
	delay(5);
	lcd_cmd(L_CFG); // 0x38
        delay(1);
	lcd_cmd(L_CFG); //Configure 0x38
	lcd_cmd(L_OFF); // 0x08
	lcd_cmd(L_ON); //Initialize with 0x0F
	lcd_cmd(L_CLR); //Clear with 0x01
	lcd_cmd(L_CFG); //Configure
    lcd_cmd(0x0C);// configure for no cursor
    lcd_cmd(L_L1);
}

void lcd_str(const char* str)
{
 unsigned char i=0;
  
 while(str[i] != 0 )
 {
   lcd_dat(str[i]);
   i++;
 }  
}

void lcd_custom_char(unsigned char loc, unsigned char * msg)
{
    if(loc < 8) // CGRAM can only store 8 custom symbols
    {
        unsigned char i;
        lcd_cmd(0x40+(loc*8));
        // Tells PIC18F to point at CGRAM address register
        for(i = 0; i < 8; i++)
        {
            lcd_dat(msg[i]); // write 8 bytes for character generation
        }
    }
}

void reverse_array(unsigned char* buffer) // Runtime O(N), Space O(1)
{
    unsigned int j = 7; unsigned int k = 0;
    while(j > k)
    {
        unsigned char temp = buffer[j];
        buffer[j] = buffer[k];
        buffer[k] = temp;
        j--;
        k++;
    }
}


void main(void) {
    /*
     * if we were very space conscious, we could use a function to reverse the
     * array of upper_left and upper_right for the lower portions when writing
     * to CGRAM so we dont have 4 arrays taking up 32 bytes in our program and
     * instead have 2 arrays taking up 16
     */
    unsigned char upper_left[8] =
    {0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0F, 0x0F};
    unsigned char upper_right[8] =
    {0x00, 0x00, 0x00, 0x00, 0x1C, 0x1E, 0x1E, 0x1E};
    
    unsigned char inverse_left[8] =
    {0x1F, 0x1F, 0x1F, 0x1F, 0x18, 0x10, 0x10, 0x10};
    unsigned char inverse_right[8] =
    {0x1F, 0x1F, 0x1F, 0x1F, 0x03, 0x01, 0x01, 0x01};
    
    
    
    //Inicjalizacja konwertera analogowo cyfrowego
    ADCON0=0x01;
    ADCON1=0x0B;
    ADCON2=0x01;
    
    TRISA=0xC3;
    TRISB=0x3F;   
    TRISC=0x01;
    TRISD=0x00;
    TRISE=0x00;
    
    lcd_init(); //Initialize LCD Disp
    lcd_cmd(L_CLR); //Clean old LCD display values
    
    unsigned int counter = 0;
    unsigned int inter;
    
    lcd_custom_char(0, upper_left); // write to CGRAM at index 0
    reverse_array(upper_left); // reverse array for lower portion
    lcd_custom_char(1, upper_left); 
    
    lcd_custom_char(2, upper_right);
    reverse_array(upper_right);
    lcd_custom_char(3, upper_right);
    
    lcd_custom_char(4, inverse_left);
    reverse_array(inverse_left);
    lcd_custom_char(5, inverse_left);
    
    lcd_custom_char(6, inverse_right);
    reverse_array(inverse_right);
    lcd_custom_char(7, inverse_right);
    
    while(counter < 5)
    {
       delay(1000);
       lcd_cmd(L_CLR);
       lcd_cmd(L_L1);
       lcd_str("  Good evening  ");
       
       delay(3000);
       lcd_cmd(L_CLR);
       lcd_cmd(L_L1);
       lcd_str(" Want to listen");
       lcd_cmd(L_L2);
       lcd_str("  To eurobeat?");
       
       delay(3000);
       
       lcd_cmd(L_CLR);
       
       for(inter = 0; inter < 5; inter++)
       {
            lcd_cmd(L_L1);
            lcd_str("   GO TO JAPAN");
            lcd_cmd(L_L2);
            lcd_str("  RIGHT NOW!!!");
       
            delay(300);
            lcd_cmd(L_CLR);
            delay(300);
       }
       
       lcd_cmd(L_CLR);
       
       for(inter = 0; inter < 5; inter++)
       {
            lcd_cmd(L_CLR);
            lcd_cmd(L_L1|(7)); // |(x) tells what position to put at
            lcd_dat(0);
            lcd_cmd(L_L2|(7));
            lcd_dat(1);
            lcd_cmd(L_L1|(8)); // |(x) tells what position to put at
            lcd_dat(2);
            lcd_cmd(L_L2|(8));
            lcd_dat(3);
            delay(300);
            
            lcd_cmd(L_CLR);
            
            char blacked[16] = {
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
            };
            
            lcd_cmd(L_L1);
            lcd_str(blacked);
            lcd_cmd(L_L2);
            lcd_str(blacked);
            
            
            lcd_cmd(L_L1|(7)); // |(x) tells what position to put at
            lcd_dat(4);
            lcd_cmd(L_L2|(7));
            lcd_dat(5);
            lcd_cmd(L_L1|(8)); // |(x) tells what position to put at
            lcd_dat(6);
            lcd_cmd(L_L2|(8));
            lcd_dat(7);
//            lcd_cmd(L_L2|(10));
//            lcd_dat(0xFF);
            delay(300);
       }
       counter ++;
    }
    return;
}
