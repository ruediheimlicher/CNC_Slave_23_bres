//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright Ruedi Heimlicher 2007. All rights reserved.
//


#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <stdlib.h>

#include "settings.h"
#include "lcd.c"
#include "adc.c"
#include "version.c"
#include "usb_rawhid.c"
//#include "ringbuffer.c"


// USB
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))


volatile uint8_t do_output=0;
static volatile uint8_t buffer[32]={};
static volatile uint8_t sendbuffer[32]={};

// begin Ringbuffer
#define RINGBUFFERTIEFE 4
#define READYBIT   0       // buffer kann Daten aufnehmen
#define FULLBIT   1        // Buffer ist voll
#define STARTBIT   2       // Buffer ist geladen
#define RINGBUFFERBIT 3    // Ringbuffer wird verwendet
#define LASTBIT   4         // Letzter Abschnitt  ist geladen
#define ENDBIT   5          // Letzter Abschnitt  ist abgearbeitet
#define STOPBIT   6        // Ablauf stoppen
#define FIRSTBIT   7

uint8_t CNCDaten[RINGBUFFERTIEFE][33];
uint8_t CDCStringArray[RINGBUFFERTIEFE];

//volatile uint8_t inposition= 0;
//volatile uint8_t outposition= 0;

volatile uint16_t abschnittnummer=0;
volatile uint16_t endposition= 0xFFFF;
volatile uint8_t ladeposition=0;

volatile uint8_t ringbufferstatus=0x00;   

uint16_t AbschnittCounter=0;
volatile uint8_t liniencounter= 0;
// end Ringbuffer
// end USB 

//#define SDAPIN		4
//#define SCLPIN		5

// SPI
#define OSZIPORT           PORTD
#define OSZIPORTDDR        DDRD
#define OSZIPORTPIN        PIND
#define OSZI_PULS_A        0
#define OSZI_PULS_B        1

#define OSZIALO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZIAHI OSZIPORT |= (1<<OSZI_PULS_A)
#define OSZIATOG OSZIPORT ^= (1<<OSZI_PULS_A)

#define OSZIBLO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZIBHI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZIBTOG OSZIPORT ^= (1<<OSZI_PULS_B)
// SPI



#define TIMER0_STARTWERT	0x40

#define LOOPLEDDDR          DDRF    //DDRD
#define LOOPLEDPORT         PORTF   //PORTD
#define LOOPLED             4       //6 

#define TASTENDDR           DDRF
#define TASTENPORT          PORTF
#define TASTENPIN          PINF

#define TASTE0				0   // HALT-PIN Motor A
#define TASTE1				1


#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define WDTBIT              7

// CNC12
#define CMD_PORT            PORTD   //    PORTB
#define CMD_DDR             DDRD    //    DDRB
#define CMD_PIN             PIND    //    PINB

#define LOAD_NEXT          5  // Bit fuer Laden des naechsten Abschnitts in der loop
#define LOAD_LAST          6  // Bit fuer Laden des letzten Abschnitts in der loop

// auf Stepperport 1
#define END_A0_PIN          6       //  PIN fuer Endanschlag A0 
#define END_B0_PIN          7 		//           Endanschlag B0 


// Auf Stepperport 2
#define END_C0_PIN          6       //  PIN fuer Endanschlag C0 
#define END_D0_PIN          7 		//           Endanschlag D0 


#define RICHTUNG_A	0
#define RICHTUNG_B	1
#define RICHTUNG_C	2
#define RICHTUNG_D	3

// Seite 1

#define STEPPERPORT_1	PORTC
#define STEPPERDDR_1    DDRC
#define STEPPERPIN_1    PINC

#define MA_STEP         0
#define MA_RI           1
#define MA_EN           2

#define MB_STEP         3
#define MB_RI           4
#define MB_EN           5

#define END_A0          6           // Bit fuer Endanschlag bei A0
#define END_B0          7           // Bit fuer Endanschlag bei A1


// Seite 2

#define STEPPERPORT_2      PORTB
#define STEPPERDDR_2       DDRB
#define STEPPERPIN_2       PINB

#define MC_STEP            0           // PIN auf Stepperport 2
#define MC_RI              1
#define MC_EN              2

#define MD_STEP            3           // PIN auf Stepperport 2
#define MD_RI              4
#define MD_EN              5

#define END_C0             6           // Anschlagstatus:  Bit fuer Endanschlag bei C0
#define END_D0             7           // Anschlagstatus:  Bit fuer Endanschlag bei D0


#define HALT_PIN           0

#define COUNT_A				0 // 4		// Motorstatus:   Schritte von Motor A zaehlen
#define COUNT_B				1 // 5		// Motorstatus:   Schritte von Motor B zaehlen

#define COUNT_C				2 // 4		// Motorstatus:   Schritte von Motor C zaehlen
#define COUNT_D				3 // 5		// Motorstatus:   Schritte von Motor D zaehlen



#define DC                  7    // DC ON: HI
#define STROM               4    // Stepperstrom Generell ON: LO

#define GO_HOME           7     // Bit fuer befehl beginn home auf cncstatus
#define DC_DIVIDER         1      // teilt die pwm-Frequenz in ISR

volatile uint8_t timer0startwert=TIMER0_STARTWERT;
#define USB_DATENBREITE 32
//volatile uint8_t rxbuffer[USB_DATENBREITE];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[USB_DATENBREITE];

//uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit

void delay_ms(unsigned int ms);

volatile uint8_t           cncstatus=0x00;

volatile uint8_t           usbstatus=0x00;
static volatile uint8_t    motorstatus=0x00;
volatile uint8_t           anschlagstatus=0x00;
volatile uint8_t           anschlagcounter=0;

#define USB_SEND  0 

volatile uint8_t status=0;

volatile uint8_t           PWM=0;
static volatile uint8_t    pwmposition=0;
static volatile uint8_t    pwmdivider=0;


volatile char SPI_data='0';
volatile char SPI_dataArray[32];

// CNC

volatile uint16_t CounterA=0;			// Zaehler fuer Delay von Motor A 
volatile uint16_t CounterB=0;			// Zaehler fuer Delay von Motor B
volatile uint16_t CounterC=0;			// Zaehler fuer Delay von Motor C 
volatile uint16_t CounterD=0;			// Zaehler fuer Delay von Motor D

volatile uint16_t DelayA=24;			// Delay von Motor A 
volatile uint16_t DelayB=24;			// Delay von Motor B 
volatile uint16_t DelayC=24;			// Delay von Motor C 
volatile uint16_t DelayD=24;			// Delay von Motor D 

volatile uint16_t StepCounterA=0;	// Zaehler fuer Schritte von Motor A 
volatile uint16_t StepCounterB=0;	// Zaehler fuer Schritte von Motor B
volatile uint16_t StepCounterC=0;	// Zaehler fuer Schritte von Motor C 
volatile uint16_t StepCounterD=0;	// Zaehler fuer Schritte von Motor D

volatile uint8_t richtung=0;

volatile uint8_t homestatus=0;

volatile uint8_t           timerstatus=0;

// bresenham start
uint8_t motorsteps = 48;
uint8_t micro = 1;
volatile uint8_t           bresenhamstatus=0x00; // relevanter motor, in Abschnittladen:bres gesetzt

volatile uint16_t           bresenham_errAB = 0; // error A,B
volatile uint16_t           bresenham_e2AB = 0; // check A,B

volatile uint16_t           bresenham_errCD = 0;
volatile uint16_t           bresenham_e2CD = 0;

volatile uint16_t           StepStartA = 0; // startwert am Anfang des Abschnittes
volatile uint16_t           StepStartC = 0;

// Seite A
volatile int16_t xA, yA, tA, dxA, dyA, incxA, incyA, pdxA, pdyA, ddxA, ddyA, deltaslowdirectionA, deltafastdirectionA, errA;

volatile uint16_t deltafastdelayA = 0; // aktueller delay 
volatile uint16_t bres_delayA = 0; // steps fuer fastdirection
volatile uint16_t bres_counterA = 0; // zaehler fuer fastdirection

//Seite B
volatile int16_t xB, yB, tB, dxB, dyB, incxB, incyB, pdxB, pdyB, ddxB, ddyB, deltaslowdirectionB, deltafastdirectionB, errB;

volatile uint16_t deltafastdelayB = 0; // aktueller delay 
volatile uint16_t bres_delayB = 0; // steps fuer fastdirection
volatile uint16_t bres_counterB = 0; // zaehler fuer fastdirection

volatile uint16_t          ramptimerintervall = TIMERINTERVALL;

volatile uint8_t           rampstatus=0;
//volatile uint8_t           RampZeit = RAMPZEIT;
//volatile uint8_t           RampFaktor = RAMPFAKTOR;
volatile uint32_t          rampstepstart=0; // Stepcounter am Anfang
//volatile uint32_t          ramptimercounter=0;  // laufender counter  fuer Rampanpassung
//volatile uint32_t          //ramptimerdelay = 100;  // Takt fuer Rampanpassung
uint8_t                    rampschritt = 2;
volatile uint16_t          rampbreite = 0;  // anzahl Schritte der Ramp. Wird beim Start bestimmt und fuer das Ende verwendet

volatile uint32_t          rampendstep = 0; // Beginn der Endramp. Wird in Abschnittladen bestimmt
volatile uint16_t          timerintervall = TIMERINTERVALL;
volatile uint16_t          timerintervall_SLOW = 0; // Intervall klein
volatile uint16_t          timerintervall_FAST = 0; // Intervall gross

uint8_t richtungstatus = 0;
uint8_t oldrichtungstatus = 0;
#define AXNEG  0
#define AYNEG  1
#define BXNEG  4
#define BYNEG  5

int16_t lastdax = 0; // letzte Werte fuer schritte x, y. Fuer berechnung gradient
int16_t lastday = 0;


uint16_t errpos = 0;
// bresenham end

// https://www.cprogramcoding.com/p/box-sizing-border-box_549.html
int max(int num1, int num2)
{
    return (num1 > num2 ) ? num1 : num2;
}


void startTimer2(void)
{
   timerstatus |= (1<<TIMER_ON);
   //timer2
   TCNT2   = 0; 
   //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode 
   TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8 
   //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt 
   TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt 
   //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64 
   TCCR2A = 0x00;
   
   sei();
}

void stopTimer2(void)
{
   TCCR2B = 0;
   timerstatus &= ~(1<<TIMER_ON);
}

void slaveinit(void)
{
	//OSZIPORTDDR |= (1<<PULS);	// Output
	//OSZIPORT |= (1<<PULS);		// HI
	
	LOOPLEDDDR |=(1<<LOOPLED);
	LOOPLEDPORT |= (1<<LOOPLED);	// HI

	STEPPERDDR_1 |= (1<<MA_STEP);
	STEPPERPORT_1 |= (1<<MA_STEP);	// HI
	
	STEPPERDDR_1 |= (1 << MA_RI);
	STEPPERPORT_1 |= (1 << MA_RI);	// HI
   
	STEPPERDDR_1 |= (1 << MA_EN);
	STEPPERPORT_1 |= (1 << MA_EN);	// HI
	
	STEPPERDDR_1 |= (1 << MB_STEP);
	STEPPERPORT_1 |= (1 << MB_STEP); // HI
	
	STEPPERDDR_1 |= (1 << MB_RI);
	STEPPERPORT_1 |= (1 << MB_RI);	// HI
	
	STEPPERDDR_1 |= (1 << MB_EN);
	STEPPERPORT_1 &= ~(1 << MB_EN); // LO
   
   //Seite 2
	STEPPERDDR_2 |= (1<<MC_STEP);
	STEPPERPORT_2 |= (1<<MC_STEP);	// HI
	
	STEPPERDDR_2 |= (1 << MC_RI);
	STEPPERPORT_1 |= (1 << MC_RI);	// HI
   
	STEPPERDDR_2 |= (1 << MC_EN);
	STEPPERPORT_2 |= (1 << MC_EN);	// HI
	
	STEPPERDDR_2 |= (1 << MD_STEP);
	STEPPERPORT_2 |= (1 << MD_STEP); // HI
	
	STEPPERDDR_2 |= (1 << MD_RI);
	STEPPERPORT_2 |= (1 << MD_RI);	// HI
	
	STEPPERDDR_2 |= (1 << MD_EN);
   STEPPERPORT_2 |= (1 << MD_EN);	// HI
   
	
	//Pin 0 von   als Ausgang fuer OSZI
	OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
    OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
    OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
    OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
	

	TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang fŸr Taste 0
	TASTENPORT |= (1<<TASTE0);	//Pull-up

//	DDRB &= ~(1<<PORTB1);	//Bit 1 von PORT B als Eingang fŸr Taste 1
//	PORTB |= (1<<PORTB1);	//Pull-up
	

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);		//Pin 4 von PORT D als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT D als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD

	//Versuch mit init von CNC 12
//	CMD_DDR &= ~(1<<END_A0_PIN);			//	Bit 0 von PORT B als Eingang fŸr Endanschlag A0
//	CMD_PORT |= (1<<END_A0_PIN);			// Pull-up

//   CMD_DDR &= ~(1<<END_B0_PIN);			//	Bit 1 von PORT B als Eingang fŸr Endanschlag A1
//	CMD_PORT |= (1<<END_B0_PIN);			// Pull-up

//	CMD_DDR &= ~(1<<PORTB1);			// Bit 1 von PORT B als Eingang fŸr Taste 1
//	CMD_PORT |= (1<<PORTB1);			//	Pull-up
	
   DDRD |= (1<<PORTD6);
  PORTD |= (1<<PORTD6);
   
   
   // Anschlaege
   
   STEPPERDDR_1 &= ~(1<<END_A0_PIN);			//	Eingang fŸr Endanschlag A0
	STEPPERPORT_1 |= (1<<END_A0_PIN);			// Pull-up
   
	STEPPERDDR_1 &= ~(1<<END_B0_PIN);			//	Eingang fŸr Endanschlag B0
	STEPPERPORT_1 |= (1<<END_B0_PIN);			// Pull-up
   
   
   STEPPERDDR_2 &= ~(1<<END_C0_PIN);			//	Eingang fŸr Endanschlag C0
	STEPPERPORT_2 |= (1<<END_C0_PIN);			// Pull-up
   
   STEPPERDDR_2 &= ~(1<<END_D0_PIN);			//	Eingang fŸr Endanschlag D0
	STEPPERPORT_2 |= (1<<END_D0_PIN);			// Pull-up
   

   
   
   CMD_DDR |= (1<<DC);                       // DC-PWM-Ausgang
   CMD_PORT &= ~(1<<DC);                      // LO
   
   CMD_DDR |= (1<<STROM);                    // Stepperstrom-Ausgang, Active HI
   CMD_PORT |= (1<<STROM);                   // HI
}




void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

void timer0 (void) 
{ 
// Timer fuer Exp
	//TCCR0 |= (1<<CS01);						// clock	/8
	//TCCR0 |= (1<<CS01)|(1<<CS02);			// clock	/64
	TCCR0B |= (1<<CS02)| (1<<CS02);			// clock	/256
	//TCCR0 |= (1<<CS00)|(1<<CS02);			// clock /1024
	TCCR0B |= (1 << CS10); // Set up timer 
	OCR0A = 0x2;
	
	//TIFR |= (1<<TOV0);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<TOIE0);							//Overflow Interrupt aktivieren
	TCNT0 = TIMER0_STARTWERT;					//RŸcksetzen des Timers

}

// in startTimer2 verchoben
/*
void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);							//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
	TCCR2 |= (1<<CS20)|(1<<CS21);				//Takt /64	Intervall 64 us

	TCCR2 |= (1<<WGM21);							//	ClearTimerOnCompareMatch CTC

	//OC2 akt
//	TCCR2 |= (1<<COM20);						//	OC2 Pin zuruecksetzen bei CTC


	TIFR |= (1<<TOV2);							//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<OCIE2);							//CTC Interrupt aktivieren

	TCNT2 = 0x00;									//Zaehler zuruecksetzen
	
	OCR2 = wert;									//Setzen des Compare Registers auf Servoimpulsdauer
} 
*/

volatile uint16_t timer2Counter=0; 

ISR (TIMER2_OVF_vect) 
{ 
	timer2Counter +=1;
   
   if (PWM) // Draht soll heiss sein. 
   {
   }
   else
   {
      pwmposition =0;
   }

	if (timer2Counter >= 14) 
	{
       
      if (CounterA)  CounterA-=1;
      if (CounterB) 	CounterB-=1;
      if (CounterC)  CounterC-=1;
      if (CounterD)  CounterD-=1;
      
      if (timerstatus & (1<<TIMER_ON))
      {
         // OSZI_A_LO(); // 100us
         
         if (bres_delayA)
         {
            bres_delayA-=1;
         }
         
         if (bres_delayB)
         {
            bres_delayB-=1;
         }
         
         
         
      } 
     
      
      
      if (PWM)
      {
         pwmposition ++;
      }
      else
      {
         pwmposition =0;
      }

		timer2Counter = 0; 
        //OSZIBTOG ;
	} 
	TCNT2 = 10;							// ergibt 2 kHz fuer Timertakt
}

/*
ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
{
//		lcd_clr_line(1);
//		lcd_puts("Timer2 Comp\0");
		TCCR20=0;
}
*/



uint8_t  AbschnittLaden_4M(const uint8_t* AbschnittDaten) // 22us
{
   stopTimer2();
   lcd_gotoxy(15,0);
   lcd_puts("    ");
   
	uint8_t returnwert=0;
// MARK: mark Reihenfolge der Daten
	/*			
	 Reihenfolge der Daten:
    0    schritteax lb
    1    schritteax hb
    2    schritteay lb
    3    schritteay hb
    
    4    delayax lb
    5    delayax hb
    6    delayay lb
    7    delayay hb
    
    8    schrittebx lb
    9    schrittebx hb
    10    schritteby lb
    11    schritteby hb
    
    12    delaybx lb
    13    delaybx hb
    14    delayby lb
    15    delayby hb
    
    
    16   (8)    code
    17   (9)    position // Beschreibung der Lage im Schnittpolygon:first, last, ...
    18   (10)   indexh     // Nummer des Abschnitts
    19   (11)   indexl   
    
    20     pwm
    
    21   motorstatus // relevanter Motor fuer Abschnitt
    
    22   Strom ON
	 */			
	int lage = 0;
   //   lage = AbschnittDaten[9]; // Start: 1, innerhalb: 0, Ende: 2
   lage = AbschnittDaten[17]; // Start: 1, innerhalb: 0, Ende: 2
	if (lage & 0x01)
   {
      returnwert=1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert=2;
   }
   richtung=0;
   
	// Motor A
	STEPPERPORT_1 &= ~(1<<MA_EN); // Pololu ON
	
	uint8_t dataL=0;
	uint8_t dataH=0;
	
	uint8_t delayL=0;
	uint8_t delayH=0;
	
	dataL=AbschnittDaten[0];
	dataH=AbschnittDaten[1];
	
	//lcd_gotoxy(17,0);
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_A); // Rueckwarts
		STEPPERPORT_1 &= ~(1<< MA_RI); // PIN fuer Treiber stellen
		//lcd_putc('r');
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_A);
		STEPPERPORT_1 |= (1<< MA_RI);
		//lcd_putc('v');	// Vorwaerts
	}
	
	dataH &= (0x7F);
	StepCounterA = dataH;		// HByte
	StepCounterA <<= 8;        // shift 8
	StepCounterA += dataL;     // +LByte
	
	delayL=AbschnittDaten[4];
	delayH=AbschnittDaten[5];
	
	
	DelayA = delayH;
	DelayA <<= 8;
	DelayA += delayL;
	
   CounterA = DelayA;
   
	// Motor B
	//CounterB=0;
	STEPPERPORT_1 &= ~(1<<MB_EN);	// Pololu ON
   
	dataL=AbschnittDaten[2];
	dataH=AbschnittDaten[3];
	//lcd_gotoxy(19,1);
   
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_B); // Rueckwarts
		STEPPERPORT_1 &= ~(1<< MB_RI);
		//lcd_putc('r');
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_B);
		STEPPERPORT_1 |= (1<< MB_RI);
		//lcd_putc('v');
	}
	
	dataH &= (0x7F);
	StepCounterB = dataH;		// HByte
	StepCounterB <<= 8;		// shift 8
	StepCounterB += dataL;	// +LByte
	
	DelayB = (AbschnittDaten[7]<<8)+ AbschnittDaten[6];
   
   CounterB = DelayB;
   
	// Motor C
	STEPPERPORT_2 &= ~(1<<MC_EN); // Pololu ON
	//CounterC=0;
	dataL=0;
	dataH=0;
	
	delayL=0;
	delayH=0;
	
	dataL=AbschnittDaten[8];
	dataH=AbschnittDaten[9];
   
	//richtung=0;
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_C); // Rueckwarts
		STEPPERPORT_2 &= ~(1<< MC_RI);
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_C);
		STEPPERPORT_2 |= (1<< MC_RI);
	}
	
	dataH &= (0x7F);
	StepCounterC = dataH;		// HByte
	StepCounterC <<= 8;		// shift 8
	StepCounterC += dataL;	// +LByte
   
	
	delayL=AbschnittDaten[12];
	delayH=AbschnittDaten[13];
   
	DelayC = delayH;
	DelayC <<=8;
	DelayC += delayL;
   
   CounterC = DelayC;
   
   // Motor D
	STEPPERPORT_2 &= ~(1<<MD_EN); // Pololu ON
	//CounterD=0;
	dataL=0;
	dataH=0;
	
	delayL = 0;
	delayH = 0;
	
	dataL = AbschnittDaten[10];
	dataH = AbschnittDaten[11];
   
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_D); // Rueckwarts
		STEPPERPORT_2 &= ~(1<< MD_RI);
		//lcd_putc('r');
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_D);
		STEPPERPORT_2 |= (1<< MD_RI);
	}
	
	dataH &= (0x7F);
	StepCounterD= dataH;		// HByte
	StepCounterD <<= 8;		// shift 8
	StepCounterD += dataL;	// +LByte
	
	delayL=AbschnittDaten[14];
	delayH=AbschnittDaten[15];
   
	DelayD = delayH;
	DelayD <<= 8;
	DelayD += delayL;
   
   CounterD = DelayD;
   
   // pwm-rate
   PWM = AbschnittDaten[20];
   
   // motorstatus: welcher Motor ist relevant
   motorstatus = AbschnittDaten[21];
   
   startTimer2();
   
   /*
   lcd_gotoxy(0,0);
   /*
   lcd_putc('A');
   lcd_putint12(StepCounterA);
   lcd_putc(' ');
   lcd_putint12(CounterA);
   lcd_gotoxy(0,1);
   lcd_putc('B');
   lcd_putint12(StepCounterB);
   lcd_putc(' ');
   lcd_putint12(CounterA);
   lcd_gotoxy(0,2);
   lcd_putc('C');
   lcd_putint12(StepCounterC);
   lcd_putc(' ');
   lcd_putint12(CounterC);
   lcd_gotoxy(0,3);
   lcd_putc('D');
   lcd_putint12(StepCounterD);
   lcd_putc(' ');
   lcd_putint12(CounterD);
*/
   return returnwert;
   
   // Nicht mehr verwendet, wird in Stepper berechnet
   if (StepCounterA > StepCounterB) 
   {
      if (StepCounterA > StepCounterC)
      {
         if (StepCounterA > StepCounterD) // A max
         {
            motorstatus |= (1<<COUNT_A);
            lcd_putc('A');
         }
         else //A>B A>C D>A
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
         
      }//A>C
      else // A>B A<C: A weg, B weg
      {
         if (StepCounterC > StepCounterD)
         {
            motorstatus |= (1<<COUNT_C);
            //lcd_putc('C');
         }
         else // A>B A<C D>C B weg, 
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
         
         
      }
   }// A>B
   
   
   else // B>A A weg
   {
      if (StepCounterB > StepCounterC) // C weg
      {
         if (StepCounterB > StepCounterD) // D weg
         {
            motorstatus |= (1<<COUNT_B);
            //lcd_putc('B');
         }
         else
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
      }
      else // B<C B weg
      {  
         if (StepCounterC > StepCounterD) // D weg
         {
            motorstatus |= (1<<COUNT_C);
            //lcd_putc('C');
         }
         else // D>C C weg
         {
            motorstatus |= (1<<COUNT_D);
            //lcd_putc('D');
         }
         
      }
   }
   
   //OSZIAHI;
   return returnwert;
}

uint8_t  AbschnittLaden(const uint8_t* AbschnittDaten)
{
   
	uint8_t returnwert=0;
	/*			
	 Reihenfolge der Daten:
	 schrittexl"
	 schrittexh"
	 schritteyl"
	 schritteyh"
	 delayxl"
	 delayxh"
	 delayyl"
	 delayyh"
	 code"
    position   Beschreibung der Lage im Schnittpolygon: first, last
    
    
	 */			
	int lage = 0;
   lage = AbschnittDaten[9]; // Start: 1, innerhalb: 0, Ende: 2
	if (lage & 0x01)
   {
      returnwert=1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert=2;
   }
   uint8_t dataL=0;
	uint8_t dataH=0;
	
	uint8_t delayL=0;
	uint8_t delayH=0;

   
	// Motor A

//	STEPPERPORT_1 &= ~(1<<MA_EN); // Pololu ON
   

	
	dataL=AbschnittDaten[0];
	dataH=AbschnittDaten[1];
	
	//lcd_gotoxy(18,0);
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_A); // Rueckwarts
		STEPPERPORT_1 &= ~(1<< MA_RI);
 
		
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_A);
		STEPPERPORT_1 |= (1<< MA_RI);
		//lcd_putc('v');	// Vorwaerts
   }   
	
	dataH &= (0x7F);
	StepCounterA= dataH;		// HByte
	StepCounterA <<= 8;		// shift 8
	StepCounterA += dataL;	// +LByte
   
	
	delayL=AbschnittDaten[4];
	delayH=AbschnittDaten[5];
	
	
	DelayA = delayH;
	DelayA <<=8;
	DelayA += delayL;
   
   
//   CounterA = DelayA;
   
	// Motor B
	//STEPPERPORT_1 |= (1<<MB_EN);
//	STEPPERPORT_1 &= ~(1<<MB_EN);	// Pololu ON
   
	dataL=AbschnittDaten[2];
	dataH=AbschnittDaten[3];
  
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_B); // Rueckwarts
		STEPPERPORT_1 &= ~(1<< MB_RI);
   }
	else 
	{
		richtung &= ~(1<<RICHTUNG_B);
		STEPPERPORT_1 |= (1<< MB_RI);
	
   
   }
	
	dataH &= (0x7F);
	StepCounterB= dataH;		// HByte
	StepCounterB <<= 8;		// shift 8
	StepCounterB += dataL;	// +LByte
	
	DelayB = (AbschnittDaten[7]<<8)+ AbschnittDaten[6];
   
//   CounterB = DelayB;
   
   
   if (StepCounterA > StepCounterB) // Hoeherer Wert setzt relevante Zaehlvariable
   {
      motorstatus |= (1 << COUNT_A); // Schritte von Motor A zaehlen 
      motorstatus &= ~(1 << COUNT_B);// Bit von Motor B zuruecksetzen
      //lcd_putc('A');
   }
   else
   {
      motorstatus |= (1 << COUNT_B); // Schritte von Motor B zaehlen 
      motorstatus &= ~(1 << COUNT_A);// Bit von Motor A zuruecksetzen
      //lcd_putc('B');
   }
   
   
 //  return returnwert;
 //
   // Motor C
//	STEPPERPORT_2 &= ~(1<<MC_EN); // Pololu ON
	//CounterC=0;
//	dataL=0;
//	dataH=0;
	
//	delayL=0;
//	delayH=0;
	
	dataL=AbschnittDaten[8];
	dataH=AbschnittDaten[9];
   
	//richtung=0;
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_C); // Rueckwarts
		STEPPERPORT_2 &= ~(1<< MC_RI);
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_C);
		STEPPERPORT_2 |= (1<< MC_RI);
	}
	
	dataH &= (0x7F);
	StepCounterC = dataH;		// HByte
	StepCounterC <<= 8;		// shift 8
	StepCounterC += dataL;	// +LByte
   
	
	delayL=AbschnittDaten[12];
	delayH=AbschnittDaten[13];
   
	DelayC = delayH;
	DelayC <<=8;
	DelayC += delayL;
   
//   CounterC = DelayC;
   
   // Motor D
//	STEPPERPORT_2 &= ~(1<<MD_EN); // Pololu ON
	//CounterD=0;
//	dataL=0;
//	dataH=0;
	
//	delayL = 0;
//	delayH = 0;
	
	dataL = AbschnittDaten[10];
	dataH = AbschnittDaten[11];
   
	if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
	{
		richtung |= (1<<RICHTUNG_D); // Rueckwarts
		STEPPERPORT_2 &= ~(1<< MD_RI);
		//lcd_putc('r');
	}
	else 
	{
		richtung &= ~(1<<RICHTUNG_D);
		STEPPERPORT_2 |= (1<< MD_RI);
	}
	
	dataH &= (0x7F);
	StepCounterD= dataH;		// HByte
	StepCounterD <<= 8;		// shift 8
	StepCounterD += dataL;	// +LByte
	
	delayL=AbschnittDaten[14];
	delayH=AbschnittDaten[15];
   
	DelayD = delayH;
	DelayD <<= 8;
	DelayD += delayL;
   
   CounterA = DelayA;
   CounterB = DelayB;
   CounterC = DelayC;
   CounterD = DelayD;


  	STEPPERPORT_1 &= ~(1<<MA_EN); // Pololu ON
   STEPPERPORT_1 &= ~(1<<MB_EN); // Pololu ON
   STEPPERPORT_2 &= ~(1<<MC_EN); // Pololu ON
   STEPPERPORT_2 &= ~(1<<MD_EN); // Pololu ON
   
     return returnwert;
}


// MARK: mark AbschnittLaden_bres
uint8_t  AbschnittLaden_bres(const uint8_t* AbschnittDaten) // 22us
{
   stopTimer2();
 //  lcd_gotoxy(15,0);
 //  lcd_puts("    ");
   
   uint8_t returnwert=0;

   /*         
    Reihenfolge der Daten:
    0    schritteax lb
    1    schritteax hb
    2    schritteay lb
    3    schritteay hb
    
    4    delayax lb
    5    delayax hb
    6    delayay lb
    7    delayay hb
    
    8    schrittebx lb
    9    schrittebx hb
    10    schritteby lb
    11    schritteby hb
    
    12    delaybx lb
    13    delaybx hb
    14    delayby lb
    15    delayby hb
    
    
    16   (8)    code
    17   (9)    position // Beschreibung der Lage im Schnittpolygon:first, last, ...
    18   (10)   indexh     // Nummer des Abschnitts
    19   (11)   indexl   
    
    20     pwm
    
    21   motorstatus // relevanter Motor fuer Abschnitt
    
    22   zoomfaktor
    
    25   steps
    26   micro
    
    */         
   
   motorsteps = AbschnittDaten[25];
   
   micro = AbschnittDaten[26];
   
   uint16_t index = (AbschnittDaten[18] << 8) | AbschnittDaten[19];
   
   if (AbschnittDaten[35] == 1)
   {
      // Serial.printf("+++ +++ +++ \t\t\t index: %d AbschnittLaden_bres WENDEPUNKT \n",index);
      rampstatus |=(1<<RAMPOKBIT);
   }

   
   
   // pwm-rate
   PWM = AbschnittDaten[20];
   //// Serial.printf("AbschnittLaden_4M steps: %d micro: %d PWM: %d\n",steps,micro,PWM);
   //// Serial.printf("AbschnittLaden_bres start \n");
   //**   analogWrite(DC_PWM, PWM);
   
   //// Serial.printf("AbschnittLaden_bres AbschnittDaten eingang index: %d\n", index);
   
   
   /*
   for(uint8_t i=0;i<27;i++) // 5 us ohne printf, 10ms mit printf
   { 
      // Serial.printf("%d \t",AbschnittDaten[i]);
   }
   // Serial.printf("\n");
   
 //  // Serial.printf("AbschnittLaden_4M steps: %d micro: %d\n",motorsteps,micro);
   
   //   lage = AbschnittDaten[9]; // Start: 1, innerhalb: 0, Ende: 2
    */
   
   int lage = 0;
   lage = AbschnittDaten[17]; // Start: 1, innerhalb: 0, Ende: 2
  // // Serial.printf("******* *********   AbschnittLaden_bres lage: %d\n",lage);
  // // Serial.printf("AbschnittLaden_bres lage: %d\n",lage);
   if (lage & 0x01) // erstes Element
   {
      returnwert=1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert=2;
   }
   
   richtung=0;
   
   // Motor A
   //digitalWriteFast(MA_EN,LOW); // Pololu ON
   STEPPERPORT_1 &= ~(1<<MA_EN); // Pololu ON
   uint8_t dataL=0;
   uint8_t dataH=0;
   
   uint8_t delayL=0;
   uint8_t delayH=0;
   
   dataL=AbschnittDaten[0];
   dataH=AbschnittDaten[1];
   
   //lcd_gotoxy(17,0);
   int8_t vz = 1;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_A); // Rueckwarts
      //digitalWriteFast(MA_RI, LOW); // PIN fuer Treiber stellen
      STEPPERPORT_1 &= ~(1<< MA_RI); // PIN fuer Treiber stellen
      vz = -1;
      //lcd_putc('r');
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_A);
      //digitalWriteFast(MA_RI, HIGH);
      STEPPERPORT_1 |= (1<< MA_RI);
      //lcd_putc('v');   // Vorwaerts
   }
   
   dataH &= (0x7F); // bit 8 entfernen
   StepCounterA = dataL | (dataH << 8);      //    

   int16_t newdax =  StepCounterA * vz;
   StepCounterA *= micro;
   StepStartA = StepCounterA;
      
   delayL=AbschnittDaten[4];
   delayH=AbschnittDaten[5];
   
   DelayA = delayL | (delayH << 8);
   
   CounterA = DelayA;
   
   // Motor B
   //CounterB=0;
   //digitalWriteFast(MB_EN,LOW);   // Pololu ON
   STEPPERPORT_1 &= ~(1<<MB_EN);   // Pololu ON
   dataL=AbschnittDaten[2];
   dataH=AbschnittDaten[3];
   //lcd_gotoxy(19,1);
   vz = 1;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_B); // Rueckwarts
      //digitalWriteFast(MB_RI, LOW);      //lcd_putc('r');
      STEPPERPORT_1 &= ~(1<< MB_RI);
      vz = -1;
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_B);
      //digitalWriteFast(MB_RI, HIGH);
      STEPPERPORT_1 |= (1<< MB_RI);
   }
   
   dataH &= (0x7F);
    
   StepCounterB = dataL | (dataH <<8);
   int16_t newday = StepCounterB * vz;
   
   StepCounterB *= micro;
   
   
    DelayB = (AbschnittDaten[7]<<8) | AbschnittDaten[6];
   
   
   // Serial.printf("\nAbschnittLaden_bres index: %d StepCounterA  : %d DelayA: %d StepCounterB: %d DelayB: %d\n",index,StepCounterA, DelayA, StepCounterB, DelayB);

   
   CounterB = DelayB;
   
   
    // Motor C
   //digitalWriteFast(MC_EN,LOW);    // Pololu ON
   STEPPERPORT_2 &= ~(1<<MC_EN); // Pololu ON
   //CounterC=0;
   dataL=0;
   dataH=0;
   
   delayL=0;
   delayH=0;
   
   dataL=AbschnittDaten[8];
   dataH=AbschnittDaten[9];
   //// Serial.printf("AbschnittLaden_4M C datah: %d\n",dataH);
   //richtung=0;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_C); // Rueckwarts
      //digitalWriteFast(MC_RI, LOW);
      STEPPERPORT_2 &= ~(1<< MC_RI);
      //// Serial.printf("AbschnittLaden_4M C negativ\n");
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_C);
      //digitalWriteFast(MA_RI, HIGH);
      STEPPERPORT_2 |= (1<< MC_RI);
      //// Serial.printf("AbschnittLaden_4M C positiv\n");
   }
   
   dataH &= (0x7F);
//   StepCounterC = dataH;      // HByte
//   StepCounterC <<= 8;      // shift 8
//   StepCounterC += dataL;   // +LByte
   
   StepCounterC = dataL | (dataH << 8);
   StepCounterC  *= micro;
   
   StepStartC = StepCounterC;
   delayL=AbschnittDaten[12];
   delayH=AbschnittDaten[13];
   
//   DelayC = delayH;
//   DelayC <<=8;
//   DelayC += delayL;
   
   DelayC = delayL | (delayH <<8);
   
   CounterC = DelayC;

   // Motor D
   //digitalWriteFast(MD_EN,LOW);   //CounterD=0;
   STEPPERPORT_2 &= ~(1<<MD_EN); // Pololu ON
   dataL=0;
   dataH=0;
   
   delayL = 0;
   delayH = 0;
   
   dataL = AbschnittDaten[10];
   dataH = AbschnittDaten[11];
   //// Serial.printf("AbschnittLaden_4M D datah: %d\n",dataH);
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_D); // Rueckwarts
      //digitalWriteFast(MD_RI, LOW);
      richtung |= (1<<RICHTUNG_D); // Rueckwarts
      //// Serial.printf("AbschnittLaden_4M D negativ\n");
      //lcd_putc('r');
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_D);
      //digitalWriteFast(MD_RI, HIGH);
      STEPPERPORT_2 |= (1<< MD_RI);
      //// Serial.printf("AbschnittLaden_4M D positiv\n");
   }
   
   dataH &= (0x7F);
   
   StepCounterD = dataL | (dataH << 8); 
   StepCounterD  *= micro;
   
   delayL=AbschnittDaten[14];
   delayH=AbschnittDaten[15];
   
   DelayD = delayL | (delayH <<8);
   
   //// Serial.printf("AbschnittLaden_4M StepCounterD: %d DelayD: %d\n",StepCounterD,DelayD);
   CounterD = DelayD;
   
   //  ****
   //  Bresenham
   //  ***
   //// Serial.printf("AbschnittLaden_bres vor bresenham: StepCounterA: %d StepCounterB: %d\n",StepCounterA,StepCounterB);
   deltafastdirectionA = 0;
   deltaslowdirectionA = 0;
   deltafastdirectionB = 0;
   deltaslowdirectionB = 0;
   deltafastdelayA = 0;
   deltafastdelayB = 0;

   // bresenham Seite A
   
   // relevanten Motor setzen
   if (StepCounterA > StepCounterB)
   {
      //
      pdxA = 1;
      pdyA = 0;
      ddxA = 1;
      ddyA = 1;
      deltaslowdirectionA = StepCounterB;
      deltafastdirectionA = StepCounterA;
      deltafastdelayA = DelayA;
 //     // Serial.printf("AbschnittLaden_bres  A > B\n");
   }
   else
   {
      //
      pdxA = 0;
      pdyA = 1;
      ddxA = 1;
      ddyA = 1;
      deltaslowdirectionA = StepCounterA;
      deltafastdirectionA = StepCounterB;
      deltafastdelayA = DelayB;
 //     // Serial.printf("AbschnittLaden_bres  A < B\n");
   }
   // aktuelle Werte einsetzen
   bres_delayA = deltafastdelayA; // aktueller delay in fastdir
   bres_counterA = deltafastdirectionA; // aktueller counter fuer steps
   
   if(rampstatus & (1<<RAMPOKBIT))
   {
      // Serial.printf("AbschnittLaden_bres index: %d set RAMPSTARTBIT\n",index);
      rampstatus |= (1<<RAMPSTARTBIT);
      errpos = 0;
      ramptimerintervall += (ramptimerintervall/4*3);
      
      //delayTimer.update(ramptimerintervall);
   }
   
   xA = StepCounterA; // 
   yA = StepCounterB;

   errA = deltafastdirectionA/2;
   
  // // Serial.printf("AbschnittLaden_bres deltafastdirectionA: %d deltaslowdirectionA: %d  deltafastdelayA: %d errA: %d bres_counterA: %d bres_delayA: %d\n",deltafastdirectionA,deltaslowdirectionA, deltafastdelayA,errA,bres_counterA,bres_delayA);

   // bresenham Seite B
   
    
   
   // relevanten Motor setzen
   if (StepCounterC > StepCounterD)
   {
      //
      pdxB = 1;
      pdyB = 0;
      ddxB = 1;
      ddyB = 1;
      deltaslowdirectionB = StepCounterD;
      deltafastdirectionB = StepCounterC;
      deltafastdelayB = DelayC;
      //// Serial.printf("AbschnittLaden_bres  C > D\n");
   }
   else
   {
      //
      pdxB = 0;
      pdyB = 1;
      ddxB = 1;
      ddyB = 1;
      deltaslowdirectionB = StepCounterC;
      deltafastdirectionB = StepCounterD;
      deltafastdelayB = DelayD;
      //// Serial.printf("AbschnittLaden_bres  C < D\n");
   }
   // aktuelle Werte einsetzen
   bres_delayB = deltafastdelayB; // aktueller delay in fastdir
   bres_counterB = deltafastdirectionB; // aktueller counter fuer steps
   
   xB = StepCounterC; // 
   yB = StepCounterD;

   errB = deltafastdirectionB/2;
   
     {
   
   timerintervall_FAST = TIMERINTERVALL;
   //  OSZI_B_LO();
   }
   
   
   // motorstatus: welcher Motor ist relevant
   motorstatus = AbschnittDaten[21];
   
   // richtung change
#pragma mark Richtung change
   
  // rampstatus |=(1<<RAMPOKBIT);

   
   startTimer2();
   
   //// Serial.printf("\nAbschnittLaden_bres end aktuellelage: %d \n",returnwert);
     return returnwert;
 
}



void AnschlagVonMotor(const uint8_t motor) // Schlitten ist am Anschlag
{
   //NSLog(@"AnschlagVonMotor: %d anschlagstatus am Beginn: %d",motor, anschlagstatus);
   if (richtung & (1<<(RICHTUNG_A + motor))) // Richtung war auf Anschlag A0 zu         
   {
      anschlagcounter ++;
       // MARK: END_A0 + motor
      if (!(anschlagstatus &(1<< (END_A0 + motor)))) // Bit noch nicht gesetzt
      {
         cli();
         PWM = 0;
         anschlagstatus |= (1<< (END_A0 + motor));      // Bit fuer Anschlag A0+motor setzen
         //anschlagstatus |= (1<< (END_A0 + motor + 4)); 
   
        // NSLog(@"anschlagstatus gesetzt: %d cncstatus: %d" anschlagstatus, cncstatus);
         //cncstatus |=  (1<<GO_HOME);
         if (cncstatus & (1<<GO_HOME)) // nur eigene Seite abstellen
         {
   // ********************************* Start HOME *****************
            // Zuerst kommt der Schlitten am Anschalg A oder C an
            
            lcd_gotoxy(15,0);
            lcd_puts("home");
           // Zuerst horizonal auf Anschlag
            switch (motor) // Stepperport 1
            {
               case 0:
               {
                  
               }
                                    
            }//switch motor
            //lcd_gotoxy(10,1);
            //lcd_putc('L');
            //lcd_putint2(ladeposition);
            
            sendbuffer[0]=0xB5 + motor; // HOME Ankunft melden
            cncstatus |= (1<<motor); 
            
             
            if (motor<2) // Stepperport 1
            {
               //lcd_gotoxy(0,2);
               
               //lcd_puts("P1 M");
               //lcd_putint1(motor);

               STEPPERPORT_1 |= (1<<(MA_EN + motor));     // Motor 0 ODER 1 OFF // andere Richtung kommt anschliessend von master
               
               if (anschlagstatus &(1<< END_A0)) // Anschlag von Motor A               
               {
                  //lcd_gotoxy(6,3);
                  //lcd_puts("A0");
                  StepCounterA=0; 
                  StepCounterB=0; 
                  if (anschlagstatus &(1<< END_B0)) // Anschlag von Motor B, NACH Motor A
                  {
                     //lcd_gotoxy(8,3);
                     //lcd_puts("B0");
                     //StepCounterB=0; 
                  }
               }
               // 
               //StepCounterB=0; 
                //             CounterA=0xFFFF;
                //             CounterB=0xFFFF;
               
            }
            else // Stepperport 2
            {
               //lcd_gotoxy(0,3);
               //lcd_puts("P2 M");
               //lcd_putint1(motor);
               

               STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
               //STEPPERPORT_1 |= (1<<(MA_EN + motor - 2)); // Paralleler Motor 0,1 OFF
               
               if (anschlagstatus &(1<< END_C0)) // Anschlag von Motor C               
               {
                  //lcd_gotoxy(6,3);
                  //lcd_puts("C0");
                  StepCounterC=0; 
                  StepCounterD=0;
                  if (anschlagstatus &(1<< END_D0)) // Anschlag von Motor D, NACH Motor C
                  {
                     //lcd_gotoxy(8,3);
                     //lcd_puts("D0");
                     StepCounterD=0; 
                  }
               }
              
              // StepCounterD=0;
               //               CounterC=0xFFFF;
               //               CounterD=0xFFFF;
               
            }
    //        cncstatus &= ~(1<<GO_HOME);
            //ladeposition=0;
    //        AbschnittCounter++;
            //sendbuffer[0]=0xEA;
            
    //        lcd_putc('C');
    //        lcd_putint(cncstatus);

 //           lcd_puthex(STEPPERPIN_1);
//            lcd_puthex(STEPPERPIN_2);
            
            sendbuffer[7]=abschnittnummer; // lo
            sendbuffer[8]=ladeposition;
            sendbuffer[22] = cncstatus;
            sendbuffer[19] = anschlagstatus;
            
            sendbuffer[23] = (StepCounterA & 0xFF0)>>8;
            sendbuffer[24] = StepCounterA & 0x00FF;
            sendbuffer[25] = (StepCounterB & 0xFF0)>>8;
            sendbuffer[26] = StepCounterB & 0x00FF;
            sendbuffer[27] = (StepCounterC & 0xFF0)>>8;
            sendbuffer[28] = StepCounterC & 0x00FF;
            sendbuffer[29] = (StepCounterD & 0xFF0)>>8;
            sendbuffer[30] = StepCounterD & 0x00FF;
            lcd_gotoxy(0,0);
            lcd_puts("code ");
            lcd_gotoxy(6+motor,0);
            lcd_puthex(sendbuffer[0]);
 //           usb_rawhid_send((void*)sendbuffer, 50); // 220518 diff
            sei();
            
  //          cncstatus &= ~(1<<GO_HOME);
            
            
            
    //        richtung &= ~(1<<(RICHTUNG_A + motor)); // Richtung umschalten // 220518 diff
// ********************************* End HOME *****************
         } // end HOME
         else           // beide Seiten abstellen, Vorgang unterbrechen
         {    
            lcd_gotoxy(10,0);
            lcd_puts("both");
            cncstatus=0;
            sendbuffer[0]=0xA5 + motor;
            
            if (motor<2) // Stepperport 1
            {
               STEPPERPORT_1 |= (1<<(MA_EN + motor));     // Motor 0,1 OFF
               STEPPERPORT_2 |= (1<<(MA_EN + motor + 2)); // Paralleler Motor 2,3 OFF
            }
            else // Stepperport 2
            {
               STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
               STEPPERPORT_1 |= (1<<(MA_EN + motor - 2)); // Paralleler Motor 0,1 OFF
            }
            
            // Alles abstellen
            StepCounterA=0;
            StepCounterB=0;
            StepCounterC=0;
            StepCounterD=0;
            
            /*
            CounterA = 0;
            CounterB = 0;
            CounterC = 0;
            CounterD = 0;
            */
            ladeposition=0;
            motorstatus=0;
             
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
            sendbuffer[22] = cncstatus;
            lcd_gotoxy(0,0);
            lcd_puts("code ");
            lcd_gotoxy(6+motor,0);
            lcd_puthex(sendbuffer[0]);

            usb_rawhid_send((void*)sendbuffer, 50);
            sei();
             richtung &= ~(1<<(RICHTUNG_A + motor)); // Richtung umschalten
           
         } // both
         
         sei();
      } // END_A0 + motor
      else
      {
         
      }
      
   } // richtung war auf Anschlag zu
   else  // richtung ist von Anschlag weg
   {
       if (!(anschlagstatus &(1<< (END_A0 + motor))))
      {
         anschlagstatus &= ~(1<< (END_A0 + motor)); // Bit fuer Anschlag X0 zuruecksetzen
      }
      else
      {
      }
      
   }
   
}

void gohome(void)
{
   /*         
    Reihenfolge der Daten:
    0    schritteax lb
    1    schritteax hb
    2    schritteay lb
    3    schritteay hb
    
    4    delayax lb
    5    delayax hb
    6    delayay lb
    7    delayay hb
    
    8    schrittebx lb
    9    schrittebx hb
    10    schritteby lb
    11    schritteby hb
    
    12    delaybx lb
    13    delaybx hb
    14    delayby lb
    15    delayby hb
    
    
    16   (8)    code
    17   (9)    position // Beschreibung der Lage im Schnittpolygon:first, last, ...
    18   (10)   indexh     // Nummer des Abschnitts
    19   (11)   indexl   
    
    20     pwm
    
    21   motorstatus // relevanter Motor fuer Abschnitt
    */         
   homestatus = 0;
   motorstatus = 0;
   richtung |= (1<<RICHTUNG_A ); // horizontaler Anschlag A
   richtung |= (1<<RICHTUNG_C ); // horizontaler Anschlag C
   cncstatus |= (1<<GO_HOME);
   
   StepCounterA=0;
   StepCounterB=0;
   StepCounterC=0;
   StepCounterD=0;
   
   CounterA=0xFFFF;
   CounterB=0xFFFF;
   CounterC=0xFFFF;
   CounterD=0xFFFF;
   uint8_t i=0, k=0;
   for (k=0;k<RINGBUFFERTIEFE;k++)
   {
      for(i=0;i<USB_DATENBREITE;i++)
      {
         CNCDaten[k][i]=0;  
      }
   }
   
   CNCDaten[0][0] = 14;// schritteax lb
   CNCDaten[0][1] = 136;// schritteax hb, negativ: 8 mit bit 7
   CNCDaten[0][4] = 14;// delayax lb
   CNCDaten[0][8] = 14;// schrittebx lb
   CNCDaten[0][9] = 136;// schrittebx hb, negativ: 4 mit bit 7
   CNCDaten[0][12] = 14;// delaybx lb
   CNCDaten[0][16] = 0xF0; // code F0
   CNCDaten[0][17] = 2; // position Beschreibung der Lage im Schnittpolygon:first, last, ...
   CNCDaten[1][19] = 0; // indexl
   CNCDaten[0][21] = 1; // motorstatus // relevanter Motor fuer Abschnitt
   
   CNCDaten[1][2] = 80;// schrittebx lb
   CNCDaten[1][3] = 193;// schrittebx hb
   CNCDaten[1][6] = 14;// delayay lb
   CNCDaten[1][10] = 80;// schritteby lb
   CNCDaten[1][11] = 193;// schritteby hb
   CNCDaten[1][14] = 14; // delayby lb
   CNCDaten[1][16] = 0xF0; // code F0
   CNCDaten[1][17] = 2;// position Beschreibung der Lage im Schnittpolygon:first, last, ...
   CNCDaten[1][19] = 1; // indexl
   CNCDaten[1][21] = 1;
   
   uint8_t pos=AbschnittLaden_4M(CNCDaten[0]); 
   
   richtung |= (1<<RICHTUNG_A ); // horizontaler Anschlag A
   richtung |= (1<<RICHTUNG_C ); // horizontaler Anschlag C
   cncstatus |= (1<<GO_HOME);
   lcd_gotoxy(10,0);
   lcd_puthex(motorstatus);
   motorstatus |= (1<< COUNT_A);
   motorstatus |= (1<< COUNT_C);
   lcd_puthex(motorstatus);
   endposition=abschnittnummer; // nur fuer home
}

// nicht verwendet
void AbschnittEndVonMotor(const uint8_t derstatus) // 0 - 3 fuer A - D   52 us
{
   uint8_t motor = (derstatus & 0x30)>>4;
   //uint8_t motor = derstatus ;
   //motor=0;
   //   STEPPERPORT_1 |= (1<<(MA_EN + motor));					// Motor A... OFF
   
   
   if (motor < 2)
   {
      //    STEPPERPORT_1 |= (1<<(MA_EN + motor));
      //    StepCounterA=0;
      //    StepCounterB=0;
   }
   else
   {
      //   STEPPERPORT_2 |= (1<<(MA_EN + motor -2)); // MC_EN ist = MA_EN, aber motor ist 3
      //   StepCounterC=0;
      //   StepCounterD=0;
      
   }
   
   sendbuffer[16]=StepCounterA & 0xFF;
   sendbuffer[17]=StepCounterB & 0xFF;
   sendbuffer[18]=StepCounterC & 0xFF;
   sendbuffer[19]=StepCounterD & 0xFF;
   sendbuffer[20]=derstatus;
   sendbuffer[21]=motor;
   sendbuffer[22]=motorstatus;
   
   //   STEPPERPORT_1 |= (1<<(MA_EN + motor));
   if (StepCounterA)
   {
      StepCounterA=1;
      
   }
   StepCounterA=0;
   
   if (StepCounterB)
   {
      StepCounterB=1;
   }
   
   StepCounterB=0;
   
   //CounterA=0xFFFF;
   //CounterA=0;
   
   CounterB=0xFFFF;
   //CounterB=0;
   
   //   STEPPERPORT_2 |= (1<<(MA_EN + motor -2));
   if (StepCounterC)
   {
      StepCounterC=1;
   }
   StepCounterC=0;
   
   if (StepCounterD)
   {
      StepCounterD=1;
   }
   StepCounterD=0;
   
   CounterC=0xFFFF;
   //CounterC=0;
   
   CounterD=0xFFFF;
   //CounterD=0;
   
   OSZIBLO;
   if (abschnittnummer==endposition) // Serie fertig
   {  
      lcd_gotoxy(0,3);
       lcd_putc('E');
      ringbufferstatus = 0;
      anschlagstatus=0;
      anschlagstatus &= ~(1<< (END_A0 + motor)); // Bit fuer Anschlag Ax zuruecksetzen
      motorstatus=0;
      //sendbuffer[0]=0xAA + motor;
      sendbuffer[0]=0xAA + motor;
      
      sendbuffer[1]=abschnittnummer;
      sendbuffer[5]=abschnittnummer;
      sendbuffer[6]=ladeposition;
      sendbuffer[7]=cncstatus;
      usb_rawhid_send((void*)sendbuffer, 50);
      sei();

      ladeposition=0;
      
      STEPPERPORT_1 |= (1<<MA_EN);
      STEPPERPORT_1 |= (1<<MB_EN);
      STEPPERPORT_2 |= (1<<MC_EN);
      STEPPERPORT_2 |= (1<<MD_EN);
   }
   else 
   { 
      
      uint8_t aktuellelage=0;
      {
         uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
         aktuelleladeposition &= 0x03; // Position im Ringbuffer
         // aktuellen Abschnitt laden
         
         aktuellelage = AbschnittLaden_4M((uint8_t*)CNCDaten[aktuelleladeposition]); //gibt Lage zurueck: 1: Anfang, 2: Ende, 0; innerhalb
         uint8_t aktuellermotor = motor;
         aktuellermotor <<=6;
         cncstatus |= aktuellermotor;
         
         if (aktuellelage==2) // war letzter Abschnitt
         {
            endposition=abschnittnummer; // letzter Abschnitt zu fahren
            
            cncstatus |= (1<<LOAD_LAST);
            
            // Neu: letzen Abschnitt melden
            sendbuffer[0]=0xD0;
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
            usb_rawhid_send((void*)sendbuffer, 50);
            sei();
            // end neu
         }  
         else
         {
            cncstatus |= (1<<LOAD_NEXT);
            // neuen Abschnitt abrufen
            
            sendbuffer[5]=abschnittnummer;
            sendbuffer[6]=ladeposition;
            sendbuffer[0]=0xA0 + motor;
            usb_rawhid_send((void*)sendbuffer, 50);
            sei();
         }
         
         ladeposition++; // Position im Ringbuffer
         
      }
      AbschnittCounter++;
      //OSZIALO;
      OSZIBHI;
   }
   
}



uint8_t RingbufferLaden(const uint8_t outpos )
{
   uint8_t lage=0;
   return lage;
}

// MARK: mark - main
int main (void) 
{
    int8_t r;

uint16_t count=0;
    
   // set for 16 MHz clock
   CPU_PRESCALE(0);
    
   // Initialize the USB, and then wait for the host to set configuration.
   // If the Teensy is powered without a PC connected to the USB port,
   // this will wait forever.
   usb_init();
   while (!usb_configured()) /* wait */ ;
    
   // Wait an extra second for the PC's operating system to load drivers
   // and do whatever it does to actually be ready for input
   _delay_ms(1000);

   //in Start-loop in while
   //init_twi_slave (SLAVE_ADRESSE);
   sei();
   
   
   slaveinit();
      
   /* initialize the LCD */
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

   lcd_puts("Guten Tag\0");
   delay_ms(1000);
   lcd_cls();
   //lcd_puts("READY\0");
   lcd_puts("V: \0");
   lcd_puts(VERSION);
   //lcd_clr_line(1);

   uint8_t Tastenwert=0;
   uint8_t TastaturCount=0;
   
   uint16_t TastenStatus=0;
   uint16_t Tastencount=0;
   uint16_t Tastenprellen=0x01F;
   //timer0();
   
   //initADC(TASTATURPIN);
   //wdt_enable(WDTO_2S);
   
   uint16_t loopcount0=0;
   uint8_t loopcount1=0;

   
   
   
   /*
   Bit 0: 1 wenn wdt ausgelšst wurde
    
     */ 
   uint8_t i=0;
   
   //timer2
   TCNT2   = 0; 
//   TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode 
   TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8 
//   TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt 
   TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt 
   //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64 
    TCCR2A = 0x00;

     sei();
   
   PWM = 0;
   
   char* versionstring = (char*) malloc(4);
   strncpy(versionstring, VERSION+9, 3);
   versionstring[3]='\0';
   volatile uint16_t versionint = atoi(versionstring);
   volatile uint8_t versionintl = versionint & 0x00FF;
   //versionint >>=8;
   volatile uint8_t versioninth = (versionint & 0xFF00)>>8;
   lcd_clr_line(0);
   
// MARK: mark while     
   while (1)
   {
      
      //OSZIBLO;
      //Blinkanzeige
      loopcount0+=1;
      if (loopcount0==0x8FFF)
      {
         loopcount0=0;
         loopcount1+=1;
         LOOPLEDPORT ^=(1<<LOOPLED);
         PORTD ^= (1<<PORTD6);
         
         lcd_gotoxy(18,3);
         lcd_puthex(loopcount1);
   //      lcd_putint1(richtung);
   //      lcd_putc(' ');
   //      lcd_putint(anschlagcounter);
   /*      
         lcd_gotoxy(0,3);
         lcd_putc('H');
         lcd_puthex(homestatus);
         lcd_putc(' ');
         lcd_putc('M');
         lcd_puthex(motorstatus);
    */     
         //
         //STEPPERPORT_1 ^= (1 << MA_STEP);
         //PORTD ^= (1<<0);
         //lcd_gotoxy(18,1);
         //lcd_puthex(loopcount1);
         //timer0();
      
         if (motorstatus > 0)
         {
          //  sendbuffer[0]=0xEB;
          //  usb_rawhid_send((void*)sendbuffer, 50);
         }
      
      } // if loopcount
      
      /**   HOT   ***********************/
      /*
       pwmposition wird in der ISR incrementiert. Wenn pwmposition > ist als der eingestellte Wert PWM, wird der Impuls wieder abgeschaltet. Nach dem Overflow wird wieder eingeschaltet
       */
      
      
      if (PWM) // Draht soll heiss sein, PWM >0. 
      {
         
         if (pwmposition > PWM) // > DC OFF, PIN ist LO
         {
            CMD_PORT &= ~(1<<DC);
            //OSZIAHI ;
         }
         else                    // > DC ON, PIN ist HI
         {
            CMD_PORT |= (1<<DC); // DC ON
            //OSZIALO ;
            
         }
         
      }
      else
      {
         CMD_PORT &= ~(1<<DC); // Draht ausschalten
      }

      
// MARK: start_usb
       /**   Begin USB-routinen   ***********************/
      
        // Start USB
      //lcd_putc('u');
      r = usb_rawhid_recv((void*)buffer, 0);
      if (r > 0) 
      {
         //OSZIBHI;
         cli(); 
         
         uint8_t code = 0x00;
         code = buffer[16];
         
         
         switch (code)
         {   
               
            case 0xE0: // Man: Alles stoppen
            {
               ringbufferstatus = 0;
               motorstatus=0;
               anschlagstatus = 0;
               cncstatus = 0;
               sendbuffer[0]=0xE1;
               
               sendbuffer[5]=abschnittnummer;
               sendbuffer[6]=ladeposition;
               usb_rawhid_send((void*)sendbuffer, 50);
               sei();
               sendbuffer[0]=0x00;
               sendbuffer[5]=0x00;
               sendbuffer[6]=0x00;
               ladeposition=0;
               endposition=0xFFFF;
               
               AbschnittCounter=0;
               PWM = sendbuffer[20];
               CMD_PORT &= ~(1<<DC);
               
               
               StepCounterA=0;
               StepCounterB=0;
               StepCounterC=0;
               StepCounterD=0;
               
               CounterA=0;
               CounterB=0;
               CounterC=0;
               CounterD=0;
               
               STEPPERPORT_1 |= (1<<MA_EN); // Pololu OFF
               STEPPERPORT_1 |= (1<<MB_EN); // Pololu OFF
               STEPPERPORT_2 |= (1<<MC_EN); // Pololu OFF
               STEPPERPORT_2 |= (1<<MD_EN); // Pololu OFF
 
               lcd_gotoxy(0,1);
               lcd_puts("HALT\0");
               
            }break;
               
               
            case 0xE2: // DC ON_OFF: Temperatur Schneiddraht setzen
            {
               PWM = buffer[20];
               if (PWM==0)
               {
                  CMD_PORT &= ~(1<<DC);
               }
               
                
            }break;
               
               
            case 0xE4: // Stepperstrom ON_OFF
            {
               
               if (buffer[8]) // 
               {
                  CMD_PORT |= (1<<STROM); // ON
                  PWM = buffer[20];
               }
               else
               {
                  CMD_PORT &= ~(1<<STROM); // OFF
                  PWM = 0;
               }
               
               if (PWM==0)
               {
                  CMD_PORT &= ~(1<<DC);
               }
               
                 
            }break;
               
            case 0xE6:  // mousup
            {
               CounterA = 0;
               CounterB = 0;
               CounterC = 0;
               CounterD = 0;
               ringbufferstatus = 0;
               cncstatus=0;
               motorstatus=0;
               StepCounterA=0;
               StepCounterB=0;
               StepCounterC=0;
               StepCounterD=0;
              
               AbschnittCounter=0;
               
            }break;
               
            case 0xF1: // reset
            {
               uint8_t i=0, k=0;
               for (k=0;k<RINGBUFFERTIEFE;k++)
               {
                  for(i=0;i<USB_DATENBREITE;i++)
                  {
                     CNCDaten[k][i]=0;  
                  }
               }
               
               ringbufferstatus = 0;
               motorstatus=0;
               anschlagstatus = 0;
               
               cncstatus = 0;
               ladeposition=0;
               endposition=0xFFFF;
               
               AbschnittCounter=0;
               PWM = 0;
               CMD_PORT &= ~(1<<DC);
               
               
               StepCounterA=0;
               StepCounterB=0;
               StepCounterC=0;
               StepCounterD=0;
               
               CounterA=0;
               CounterB=0;
               CounterC=0;
               CounterD=0;
               
               lcd_gotoxy(14,0);
               lcd_puts("reset\0");
               //cli();
               //usb_init();
               /*
                while (!usb_configured()) // wait  ;
                
                // Wait an extra second for the PC's operating system to load drivers
                // and do whatever it does to actually be ready for input
                _delay_ms(1000);
                */
               //sei();
               //sendbuffer[0]=0xF2;
               //usb_rawhid_send((void*)sendbuffer, 50);
               //sendbuffer[0]=0x00;
               
            }break;
              
               // 220516: nicht mehr gesetzt, zeigt Endmarkierung sonst nicht an
// MARK: F0
            case 0xF0:// cncstatus fuer go_home setzen
               {
                  lcd_cls();
                  lcd_gotoxy(0,0);
                  lcd_puts("HOME");
                  
                //  gohome();
                //  break;
                  abschnittnummer = 0; // diff 220520
                  
                  ladeposition=0;
                  endposition=0xFFFF;
                  cncstatus = 0;
                  motorstatus = 0;
                  ringbufferstatus=0x00;
                  anschlagstatus=0;
                  ringbufferstatus |= (1<<FIRSTBIT);
        //          ringbufferstatus |= (1<<STARTBIT); // diff 220520

                  AbschnittCounter=0;
                  //sendbuffer[8]= versionintl;
                  //sendbuffer[8]= versioninth;

                  sendbuffer[0]=0xF0;
                  //sendbuffer[0]=0x45;
                  cncstatus |= (1<<GO_HOME); // Bit fuer go_home setzen
                  sendbuffer[22] = cncstatus;
                  
                  ringbufferstatus |= (1<<LASTBIT);
                  // Daten vom buffer in CNCDaten laden
                  {
                     uint8_t pos=0;
                     pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
                     //if (abschnittnummer>8)
                     {
                        //lcd_putint1(pos);
                     }
                     uint8_t i=0;
                     for(i=0;i<USB_DATENBREITE;i++)
                     {
                        if (i<5)
                        {
                         //  lcd_puthex(buffer[i]);
                        }
                        CNCDaten[pos][i]=buffer[i];  
                     }
                     
                  }
                  startTimer2();
                  
                  // F0 melden
      //            usb_rawhid_send((void*)sendbuffer, 50);
                 
                  sei();

               }break;
               
// MARK: mark default
            default:
            {
               // Abschnittnummer bestimmen
               uint8_t indexh=buffer[18];
               uint8_t indexl=buffer[19];
               uint8_t position = buffer[17];
               
               
               abschnittnummer= indexh<<8;
               abschnittnummer += indexl;
               sendbuffer[0]=0x33;
               sendbuffer[5]=abschnittnummer;
               sendbuffer[6]=buffer[16];
               
    //           sendbuffer[8]= versionintl;
    //           sendbuffer[9]= versioninth;
               
               /* in Mill32
                // Lage:
                
                uint8_t lage = buffer[25];
                // lage im Ablauf: 
                // 1: erster Abschnitt
                // 2: letzter Abschnitt
                // 0: innerer Abschnitt

                */
               
              
 //              usb_rawhid_send((void*)sendbuffer, 50); // nicht jedes Paket melden
               
               if (abschnittnummer==0)
               {
                  //anschlagstatus &= ~(1<< END_A0); // 220518 diff
                //  lcd_clr_line(1);
                  cli();
                  /*
                  uint8_t i=0,k=0;
                  for (k=0;k<RINGBUFFERTIEFE;k++)
                  {
                     for(i=0;i<USB_DATENBREITE;i++)
                     {
                        CNCDaten[k][i]=0;  
                     }
                  }
                   */
                  //CNCDaten = {};
                  
                  
                  ladeposition=0;
                  endposition=0xFFFF;
                  cncstatus = 0;
                  motorstatus = 0;
                  ringbufferstatus=0x00;
                  anschlagstatus=0;
                  ringbufferstatus |= (1<<FIRSTBIT);
                  AbschnittCounter=0;
                  //sendbuffer[8]= versionintl;
                  //sendbuffer[8]= versioninth;
                  sendbuffer[5]=0x00;
                  
                  //in teensy3.2: timerintervall
  //                sendbuffer[8] = (TIMERINTERVALL & 0xFF00)>>8;
  //                sendbuffer[9] = (TIMERINTERVALL & 0x00FF);
                  sendbuffer[0]=0xD1;
                   
          //         usb_rawhid_send((void*)sendbuffer, 50);
                  startTimer2();
                  sei();
                  
               }
               else
               {
                  
               }
               
               //             if (buffer[9]& 0x02)// letzter Abschnitt
               
               if (buffer[17]& 0x02)// letzter Abschnitt
               {
                  ringbufferstatus |= (1<<LASTBIT);
                  if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
                  {
                     endposition=abschnittnummer; // erster ist letzter Abschnitt
                  }
               }
               
               
               
               // Daten vom buffer in CNCDaten laden
               {
                  uint8_t pos=(abschnittnummer);
                  pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
                  //if (abschnittnummer>8)
                  {
                     //lcd_putint1(pos);
                  }
                  uint8_t i=0;
                  for(i=0;i<USB_DATENBREITE;i++)
                  {
                     CNCDaten[pos][i]=buffer[i];  
                  }
                  
               }
               
               
               // Erster Abschnitt, naechsten Abschnitt laden
               if ((abschnittnummer == 0)&&(endposition))
               {
                  {
                     //lcd_putc('*');
                     // Version zurueckmelden
                     
                     int versionl, versionh;
                     
                     //versionl=VERSION & 0xFF;
                     //versionh=((VERSION >> 8) & 0xFF);

                     
                     
                     
                     
                     sendbuffer[5]=abschnittnummer;
                     sendbuffer[6]=ladeposition;
                     sendbuffer[0]=0xAF;
                     usb_rawhid_send((void*)sendbuffer, 50);
                     sei();
                     //  sendbuffer[0]=0x00;
                     //  sendbuffer[5]=0x00;
                     //  sendbuffer[6]=0x00;
                     
                     
                  }  
               }
               
               ringbufferstatus &= ~(1<<FIRSTBIT);
               
               // Ringbuffer ist voll oder  letzter Abschnitt schon erreicht
               //if ((abschnittnummer >= 2)||(ringbufferstatus & (1<<LASTBIT)))                {
               if ((abschnittnummer ==1 )||((abschnittnummer ==0 )&&(ringbufferstatus & (1<<LASTBIT)))) 
               {
                  {
                     ringbufferstatus &= ~(1<<LASTBIT);
                     ringbufferstatus |= (1<<STARTBIT);
                     
                  }
               }
               
            } // default
               
         } // switch code
         code=0;
         sei();
         
      } // r>0, neue Daten
      
      /**   End USB-routinen   ***********************/
      
      /**   Start CNC-routinen   ***********************/
      
      if (ringbufferstatus & (1<<STARTBIT)) // Buffer ist geladen, Abschnitt 0 laden
      {
         cli();
         ringbufferstatus &= ~(1<<STARTBIT);         
         ladeposition=0;
         AbschnittCounter=0;
         richtungstatus = 0; // neubeginn, set back
         oldrichtungstatus = 0;
         // Abschnitt 0 laden
         uint8_t l = sizeof(CNCDaten[ladeposition]);
         uint8_t micro = CNCDaten[ladeposition][26];

         // Ersten Abschnitt laden
         for(i=0;i<USB_DATENBREITE;i++)
         {
  //          CNCDaten[0][i]=0;  
         }

         //uint8_t pos=AbschnittLaden_4M(CNCDaten[0]); 
         uint8_t pos=AbschnittLaden_bres(CNCDaten[0]); // erster Wert im Ringbuffer
         
         lcd_gotoxy(0,0);
         lcd_putc('A');
         lcd_putint(pos);
         ladeposition++;
         if (pos==2) // nur ein Abschnitt
         {
            ringbufferstatus |=(1<<ENDBIT);
            ringbufferstatus |=(1<<LASTBIT);
         }
         
         AbschnittCounter+=1;
         sei();
      }
       
  
//// MARK: mark Anschlag
      // MARK: Anschlag
      // ********************
      // * Anschlag Motor A *
      // ********************
      
      if ((STEPPERPIN_1 & (1<< END_A0_PIN)) ) // Eingang ist HI, Schlitten nicht am Anschlag A0
      {
         if (anschlagstatus &(1<< END_A0))
         {
            anschlagstatus &= ~(1<< END_A0); // Bit fuer Anschlag A0 zuruecksetzen
            lcd_gotoxy(12,2);
            lcd_puts("  ");

         }
      }
      else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag A0
      {    
         lcd_gotoxy(12,2);
         lcd_putc('A');
        lcd_putc('0');

          AnschlagVonMotor(0);
      }
      
      
      // **************************************
      // * Anschlag Motor B *
      // **************************************
      // Anschlag B0
      if ((STEPPERPIN_1 & (1<< END_B0_PIN)) ) // Schlitten nicht am Anschlag B0
      {
         if (anschlagstatus &(1<< END_B0))
         {
            anschlagstatus &= ~(1<< END_B0); // Bit fuer Anschlag B0 zuruecksetzen
            lcd_gotoxy(16,2);
            lcd_puts("  ");

         }
      }
      else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag B0
      {
        lcd_gotoxy(16,2);
        lcd_putc('B');
       lcd_putc('0');

         AnschlagVonMotor(1);
      } // end Anschlag B0
      
      // End Anschlag B
      
      
      // ********************
      // * Anschlag Motor C *
      // ********************
      
      // Anschlag C0
      if ((STEPPERPIN_2 & (1<< END_C0_PIN)) ) // Eingang ist HI, Schlitten nicht am Anschlag C0
      {
         if (anschlagstatus &(1<< END_C0))
         {
            anschlagstatus &= ~(1<< END_C0); // Bit fuer Anschlag C0 zuruecksetzen
            lcd_gotoxy(12,3);
            lcd_puts("  ");


         }         
      }
      else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag C0
      {
         lcd_gotoxy(12,3);
         lcd_putc('C');
         lcd_putc('0');
 
         AnschlagVonMotor(2);
      }
      
      // ***************
      // * Anschlag Motor D *
      // ***************
      
      // Anschlag D0
      if ((STEPPERPIN_2 & (1<< END_D0_PIN)) ) // Schlitten nicht am Anschlag D0
      {
         if (anschlagstatus &(1<< END_D0))
         {
            anschlagstatus &= ~(1<< END_D0); // Bit fuer Anschlag D0 zuruecksetzen
            lcd_gotoxy(16,3);
            lcd_puts("  ");

        }
      }
      else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag D0
      {
         lcd_gotoxy(16,3);
         lcd_putc('D');
         lcd_putc('0');
         AnschlagVonMotor(3);
      }

// MARK: Motor A  B  
      
      // Anschlag schon registriert
      // Begin Motor A
      // **************************************
      // * Motor A B *
      // **************************************
      if (deltafastdirectionA > 0) // Bewegung auf Seite A vorhanden
      {
         
         // Es hat noch Steps, CounterA ist abgezaehlt (DelayA bestimmt Impulsabstand fuer Steps), kein Anschlag 
         if ((bres_counterA > 0)  && (bres_delayA == 0) &&((!(anschlagstatus & (1<< END_A0))) && (!(anschlagstatus & (1<< END_B0)))) )        
         {
            // start ramp
            
            if (rampstatus & (1<<RAMPSTARTBIT))
            {
               if (ramptimerintervall > timerintervall_FAST) // noch nicht auf max speed
               {
                  //errarray[errpos++] = ramptimerintervall;
                  //  Serial.printf("start ramptimerintervall: %d\n",ramptimerintervall);
                  if(rampstatus & (1<<RAMPOKBIT))
                  {
                     ramptimerintervall -= RAMPSCHRITT;
                     
                     //        delayTimer.update(ramptimerintervall);
                     //rampbreite++;
                     
                  }
                  
               }
               else
               {
                  //OSZI_B_HI();
                  //errarray[errpos++] = 1000;
                  rampstatus &= ~(1<<RAMPSTARTBIT);
                  rampendstep = rampstepstart - max(StepCounterA, StepCounterB);
                  rampstatus |= (1<<RAMPENDBIT);
                  rampstatus |= (1<<RAMPEND0BIT);
                  //Serial.printf("end rampstepstart: %d rampendstep: %d ramptimerintervall: %d timerintervall: %d\n",rampstepstart,rampendstep, ramptimerintervall,timerintervall);
                  //Serial.printf("end ramp\n");
                  rampstatus &= ~(1<<RAMPOKBIT);
               }
            }
            // end ramp
            
            //      noInterrupts();
            //
            // Aktualisierung Fehlerterm
            errA -= deltaslowdirectionA;
            
            bres_counterA -= 1; // steps abarbeiten
            
            if (bres_counterA < 4)
            {
               //Serial.printf("bres_counterA: %d xA: %d yA: %d\n",bres_counterA, xA, yA);
            }
            if (errA < 0)
            {
               //Fehlerterm wieder positiv (>=0) machen
               errA += deltafastdirectionA;
               // Schritt in langsame Richtung, Diagonalschritt
               xA -= ddxA;
               yA -= ddyA;
               if (xA)
               {
                  if (ddxA && xA)// Motor A soll steppen
                  {
                     STEPPERPORT_1 &= ~(1 << MA_STEP);
                     //digitalWriteFast(MA_STEP,LOW);
                  }
               }
               if (yA)
               {
                  if (ddyA && yA)// Motor B soll steppen
                  {
                     //digitalWriteFast(MB_STEP,LOW);
                     STEPPERPORT_1 &= ~(1 << MB_STEP);
                  }
               }
               // Impuls A und B starten
               //Serial.printf("Motor A diagonal\t");
            }
            else 
            {
               // Schritt in schnelle Richtung, Parallelschritt
               if (xA) // noch Schritte da
               {
                  xA -= pdxA;
               }
               if (yA) 
               {
                  yA -= pdyA;
               }
               
               if (xA) // noch Schritte Motor A
               {
                  if (pdxA && xA)// Motor A soll steppen
                  {
                     STEPPERPORT_1 &= ~(1 << MA_STEP);
                     //digitalWriteFast(MA_STEP,LOW);
                  }
               }
               if (yA) // noch Schritte Motor B
               {
                  if (pdyA && yA)// Motor B soll steppen
                  {
                     //digitalWriteFast(MB_STEP,LOW);
                     STEPPERPORT_1 &= ~(1 << MB_STEP);
                  }
               }
               
               //Serial.printf("Motor A parallel\t");
            }
            bres_delayA = deltafastdelayA;
            // CounterA zuruecksetzen fuer neuen Impuls
            
            
            // Wenn StepCounterA jetzt nach decrement abgelaufen und relevant: next Datenpaket abrufen
            if ((bres_counterA == 0 ) )    // relevanter counter abgelaufen
            {
               //Serial.printf("Motor AB bres_counterA ist null\n");
               if ((abschnittnummer==endposition)) // Ablauf fertig
               {  
                  //Serial.printf("*** *** *** *** *** *** Motor AB abschnittnummer==endposition xA: %d yA: %d cncstatus: %d\n",xA, yA, cncstatus);
                  if (cncstatus & (1<<GO_HOME))
                  {
                     homestatus |= (1<<COUNT_A);
                  }
                  
                  //        cli();
                  //Serial.printf("Motor A endpos > BD\n");
                  ringbufferstatus = 0;
                  // home: 
                  motorstatus &= ~(1<< COUNT_A);
                  motorstatus=0;
                  
                  sendbuffer[0]=0xBD;
                  sendbuffer[5]=abschnittnummer;
                  sendbuffer[6]=ladeposition;
                  //    sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                  sendbuffer[22] = cncstatus;
                  //Serial.printf("*** *** *** *** *** BD 1\n");
                  usb_rawhid_send((void*)sendbuffer, 0);
                  ladeposition=0;
                  
                  //analogWrite(DC_PWM, 0);
                  
                  cncstatus=0;
                  /*
                   for (uint16_t i=0;i<255;i++)
                   {
                   
                   Serial.printf("%d\t%d \n",i,errarray[i]);
                   }
                   */
                  //      sei();
               }
               else 
               {
                  uint8_t aktuellelage=0; // Lage innerhalb der Abschnittserie: Start: 1, Innerhalb: 0, Ende: 2
                  
                  uint8_t aktuelleladeposition=(ladeposition & 0x00FF); // 8 bit
                  aktuelleladeposition &= 0x03;
                  
                  // aktuellen Abschnitt laden
                  //_delay_us(5);
                  
                  //Serial.printf("axh: %d \t",CNCDaten[aktuelleladeposition][1]);
                  uint8_t axh = CNCDaten[aktuelleladeposition][1];
                  //Serial.printf("axh: %d \t",axh);
                  if (axh < 128)
                  {
                     //Serial.printf("richtung x positiv\n");
                  }
                  else 
                  {
                     //Serial.printf("richtung x positiv\n");
                  }
                  
                  //     Serial.printf("Motor AB: aktuellelage code vor: %d\nAbschnittdaten vor Funktion: \n",CNCDaten[aktuelleladeposition][17]);
                  for(uint8_t i=0;i<27;i++) // 5 us ohne printf, 10ms mit printf
                  { 
                     //  Serial.printf("%d \t",CNCDaten[aktuelleladeposition][i]);
                  }
                  //Serial.printf("\n");
                  
                  
                  
                  aktuellelage = AbschnittLaden_bres(CNCDaten[aktuelleladeposition]); 
                  
                  
                  //Serial.printf("deltafastdirectionA: %d Motor AB: ladeposition: %d aktuellelage: %d ",deltafastdirectionA, ladeposition,aktuellelage);
                  if (aktuellelage==2) // war letzter Abschnitt
                  {
                     //Serial.printf("Motor AB:  war letzter Abschnitt xA: %d yA: %d abschnittnummer: %d\n",xA, yA, abschnittnummer);
                     
                     endposition=abschnittnummer; // letzter Abschnitt
                     
                     // Neu: letzten Abschnitt melden
                     sendbuffer[0]=0xD0;
                     sendbuffer[5]=abschnittnummer;
                     sendbuffer[6]=ladeposition;
                     //sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                     sendbuffer[22] = cncstatus;
                     usb_rawhid_send((void*)sendbuffer, 0);
                     // sei();
                     
                  }
                  else
                  {
                     // neuen Abschnitt abrufen
                     //Serial.printf("Motor AB neuen Abschnitt abrufen xA: %d yA: %d abschnittnummer: %d\n",xA, yA, abschnittnummer);
                     sendbuffer[5]=abschnittnummer;
                     sendbuffer[6]=ladeposition;
                     //sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                     sendbuffer[22] = cncstatus;
                     // TODO : ev.  A0 setzen
                     sendbuffer[0]=0xA1;
                     usb_rawhid_send((void*)sendbuffer, 0);
                  }
                  
                  ladeposition++;
                  
                  AbschnittCounter++;
                  
               }
            }
            
            //       interrupts();
         }
         else  //if (bres_counterA == 0)        //if ((StepCounterA ==0)  && (StepCounterB ==0))
         {
            //OSZI_A_LO();
            //if (digitalReadFast(MA_STEP) == 0) //100 ns
            if ((STEPPERPORT_1 &(1 << MA_STEP)) == 0)
            {
               //Serial.printf("step beenden\n"); 
               //digitalWriteFast(MA_STEP,HIGH);
               STEPPERPORT_1 |= (1 << MA_STEP);
            }
            
            //if (digitalReadFast(MB_STEP) == 0) //100 ns
            if ((STEPPERPORT_1 &(1 << MB_STEP)) == 0)
            {
               
               //digitalWriteFast(MB_STEP,HIGH);
               STEPPERPORT_1 |= (1 << MB_STEP);
            }
            if ((xA == 0)  && (yA == 0))
            {
               //if (digitalReadFast(MA_EN) == 0)
               if ((STEPPERPORT_1 &(1 << MB_EN)) == 0)
               {
                  // Motoren ausschalten
                  //Serial.printf("Motor A ausschalten\n"); 
                  //digitalWriteFast(MA_EN,HIGH);
                  STEPPERPORT_1 |= (1 << MA_EN);
               }
               //if (digitalReadFast(MB_EN) == 0)
               if ((STEPPERPORT_1 &(1 << MB_EN)) == 0)
               {
                  // Motoren ausschalten
                  //Serial.printf("Motor B ausschalten\n"); 
                  //digitalWriteFast(MB_EN,HIGH);
                  STEPPERPORT_1 |= (1 << MA_EN);
               }
               
               
            }
            
            
            //OSZI_A_HI();
            //      interrupts();
         }
         
      } // if deltafastdirectionA > 0
   
      // End Motor B
      
      // Begin Motor C
// MARK: mark Motor C
      // **************************************
      // * Motor C *
      // **************************************
      
      // Es hat noch Steps, CounterC ist abgezaehlt (DelayA bestimmt Impulsabstand fuer Steps)
      if (StepCounterC &&(CounterC == 0) &&(!(anschlagstatus & (1<< END_C0))))//||(cncstatus & (1<< END_D0)))))//   
      {
         cli();
         // Impuls starten
         STEPPERPORT_2 &= ~(1<<MC_STEP);   // Impuls an Motor C LO -> ON
         CounterC=DelayC;                     // CounterA zuruecksetzen fuer neuen Impuls
          
         StepCounterC--;
         
         // Wenn StepCounterC abgelaufen und relevant: next Datenpaket abrufen
         if (StepCounterC ==0 && (motorstatus & (1<< COUNT_C)))    // Motor C ist relevant fuer Stepcount 
         {
            lcd_gotoxy(12,1);
            lcd_puts("C end");
//            STEPPERPORT_2 |= (1<<MC_EN);                          // Motor C OFF
            //StepCounterD=0; 
            // Begin Ringbuffer-Stuff
            //if (ringbufferstatus & (1<<ENDBIT))
            if (abschnittnummer==endposition)
            {  
               lcd_puts("CA");
               if (cncstatus & (1<<GO_HOME))
               {
               homestatus |= (1<<COUNT_C);
               }

               cli();
               ringbufferstatus = 0;
               cncstatus=0;
               // home: 
               motorstatus &= ~(1<< COUNT_C);
               motorstatus = 0;
               //
               
               
               //
               sendbuffer[0]=0xBD;
               sendbuffer[5]=abschnittnummer;
               sendbuffer[6]=ladeposition;
               sendbuffer[22] = cncstatus;
               usb_rawhid_send((void*)sendbuffer, 50);
               ladeposition=0;
               sei();
               
            }
            else 
            { 
               lcd_puts("CB");
               uint8_t aktuellelage=0; // Lage innerhalb der Abschnittserie: Start: 1, Innerhalb: 0, Ende: 2
               uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
               aktuelleladeposition &= 0x03;
               
               // aktuellen Abschnitt laden
               
               if (ladeposition>8)
               {
                  //lcd_putint1(ladeposition);
               }
               //aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
               aktuellelage = AbschnittLaden_bres(CNCDaten[aktuelleladeposition]);
               if (aktuellelage==2) // war letzter Abschnitt
               {
                  endposition=abschnittnummer; // letzter Abschnitt
                  // Neu: letzten Abschnitt melden
                  sendbuffer[0]=0xD0;
                  sendbuffer[5]=abschnittnummer;
                  sendbuffer[6]=ladeposition;
                  sendbuffer[22] = cncstatus;
                  
                  usb_rawhid_send((void*)sendbuffer, 50);
                  sei();
                  
               }  
               else
               {
                  // neuen Abschnitt abrufen
                  sendbuffer[5]=abschnittnummer;
                  sendbuffer[6]=ladeposition;
                  sendbuffer[22] = cncstatus;
                  sendbuffer[0]=0xA2;
                  usb_rawhid_send((void*)sendbuffer, 50);  
                  
               }
               
               ladeposition++;
               
               
               
               
               if (aktuellelage==2)
               {
                  //ringbufferstatus |= (1<<ENDBIT);
               }
               AbschnittCounter++;
               
            }
            
         }
         
         
      }
      else
      {
   
         STEPPERPORT_2 |= (1<<MC_STEP);               // Impuls an Motor C HI -> OFF
         
         if (StepCounterC ==0)                     // Keine Steps mehr fuer Motor C
         {
   
            STEPPERPORT_2 |= (1<<MC_EN);                     // Motor C OFF
         }
      }
         sei();
      // MARK: mark Motor D
        // **************************************
      // * Motor D *
      // **************************************
      
      if (StepCounterD && (CounterD == 0)&&(!(anschlagstatus & (1<< END_D0))))
      {
         cli();
         
         STEPPERPORT_2 &= ~(1<<MD_STEP);               // Impuls an Motor D LO: ON
         CounterD= DelayD;
         StepCounterD--;
         
         if (StepCounterD ==0 && (motorstatus & (1<< COUNT_D))) // Motor D ist relevant fuer Stepcount 
         {
            lcd_gotoxy(16,1);
            lcd_puts("D end");
            //StepCounterC=0;
            // Begin Ringbuffer-Stuff
            if (abschnittnummer==endposition)
            {  
               lcd_puts("DA");
               if (cncstatus & (1<<GO_HOME))
               {
               homestatus |= (1<<COUNT_D);
               }

               cli();
               
               ringbufferstatus = 0;
               cncstatus=0;
               motorstatus &= ~(1<< COUNT_D);
               
               motorstatus = 0;
               sendbuffer[0]=0xBD;
               sendbuffer[5]=abschnittnummer;
               sendbuffer[6]=ladeposition;
               
               sendbuffer[8] = cncstatus;
               usb_rawhid_send((void*)sendbuffer, 50);
               ladeposition=0;
               sei();
            }
            else 
            { 
               lcd_puts("DB");
               uint8_t aktuellelage=0;
               {
                  uint8_t aktuelleladeposition=(ladeposition & 0x00FF);
                  aktuelleladeposition &= 0x03;
                  
                  // aktuellen Abschnitt laden
                  
                  //aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
                  aktuellelage = AbschnittLaden_bres(CNCDaten[aktuelleladeposition]);
                  if (aktuellelage==2) // war letzter Abschnitt
                  {
                     endposition=abschnittnummer; // letzter Abschnitt
                     // Neu: letzten Abschnitt melden
                     sendbuffer[0]=0xD0;
                     sendbuffer[5]=abschnittnummer;
                     sendbuffer[6]=ladeposition;
                     sendbuffer[8] = cncstatus;
                     usb_rawhid_send((void*)sendbuffer, 50);
                     sei();

                  }  
                  else
                  {
                     // neuen Abschnitt abruffen
                     sendbuffer[5]=abschnittnummer;
                     sendbuffer[6]=ladeposition;
                     sendbuffer[8] = cncstatus;
                     sendbuffer[0]=0xA3;
                     usb_rawhid_send((void*)sendbuffer, 50);
                     
                  }
                  
                  ladeposition++;
                  
               }
               if (aktuellelage==2)
               {
                  //ringbufferstatus |= (1<<ENDBIT);
               }
               AbschnittCounter++;
               
            }
         }
         
         
         sei();
      }
      else// if (CounterB)
      {
         STEPPERPORT_2 |= (1<<MD_STEP);
         if (StepCounterD ==0)                     // Keine Steps mehr fuer Motor D
         {
            STEPPERPORT_2 |= (1<<MD_EN);               // Motor D OFF
            
         }
         
         
         
      }
      sei(); 
      // End Motor D
      
   
      /**   Ende CNC-routinen   ***********************/
      
      
      /* **** rx_buffer abfragen **************** */
      //rxdata=0;
      
// MARK: mark Tasten      
      //   Daten von USB vorhanden
       // rxdata
      
      //lcd_gotoxy(16,0);
        //lcd_putint(StepCounterA & 0x00FF);
      
      if (!(TASTENPIN & (1<<TASTE0))) // Taste 0
      {
         //lcd_gotoxy(8,1);
         //lcd_puts("T0 Down\0");
         
         if (!(TastenStatus & (1<<TASTE0))) //Taste 0 war noch nicht gedrueckt
         {
            //RingD2(5);
            TastenStatus |= (1<<TASTE0);
            
            Tastencount=0;
            //lcd_gotoxy(0,1);
            //lcd_puts("P0 \0");
            //lcd_putint(TastenStatus);
            //delay_ms(800);
         }
         else
         {
            
            
            Tastencount +=1;
            //lcd_gotoxy(7,1);
            //lcd_puts("TC \0");
            //lcd_putint(Tastencount);
            
            if (Tastencount >= Tastenprellen)
            {
               
               Tastencount=0;
               if (TastenStatus & (1<<TASTE0))
               {
                  //sendbuffer[0]=loopcount1;
                  //sendbuffer[1]=0xAB;
                  //usbstatus |= (1<<USB_SEND);
                  //lcd_gotoxy(2,1);
                  //lcd_putc('1');

                  //usb_rawhid_send((void*)sendbuffer, 50);
               }
               TastenStatus &= ~(1<<TASTE0);
               //lcd_gotoxy(3,1);
               //lcd_puts("ON \0");
               //delay_ms(400);
               //lcd_gotoxy(3,1);
              // lcd_puts("  \0");
               //lcd_putint(TastenStatus);
               
               
            }
         }//else
         
      }   // Taste 0
      
         
      
      if (!(TASTENPIN & (1<<TASTE1))) // Taste 1
      {
         //lcd_gotoxy(12,1);
         //lcd_puts("T1 Down\0");
         
         if (! (TastenStatus & (1<<TASTE1))) //Taste 1 war nicht nicht gedrueckt
         {
            TastenStatus |= (1<<TASTE1);
            Tastencount=0;
            //lcd_gotoxy(3,1);
            //lcd_puts("P1 \0");
            //lcd_putint(Servoimpulsdauer);
            //delay_ms(800);
            
         }
         else
         {
            //lcd_gotoxy(3,1);
            //lcd_puts("       \0");
            
            Tastencount +=1;
            if (Tastencount >= Tastenprellen)
            {
               
               
               Tastencount=0;
               TastenStatus &= ~(1<<TASTE1);
               
            }
         }//   else
         
      } // Taste 1
      
      /* ******************** */
      //      initADC(TASTATURPIN);
      //      Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
      
      Tastenwert=0;
      
      //lcd_gotoxy(3,1);
      //lcd_putint(Tastenwert);
   
      //OSZIBHI;
      if (usbstatus & (1<< USB_SEND))
      {
         //lcd_gotoxy(10,1);
         //lcd_puthex(AbschnittCounter);
         //sendbuffer[3]= AbschnittCounter;
         //usb_rawhid_send((void*)sendbuffer, 50);
         //sendbuffer[0]=0;
         //sendbuffer[5]=0;
         //sendbuffer[6]=0;
         //usbstatus &= ~(1<< USB_SEND);
         
      }

   }//while
   //free (sendbuffer);

// return 0;
}
