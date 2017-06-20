/*
 * Simple DDS Signal Generator
 * 
 * 2017/6/20 by morecat_lab
 * 
 * based on http://interface.khm.de/index.php/lab/interfaces-advanced/arduino-dds-sinewave-generator/
 * KHM 2009 /  Martin Nawrath
 * Kunsthochschule fuer Medien Koeln
 * Academy of Media Arts Cologne
 * 
 * [spec]
 * SIN1 ,SQU11 50Hz - 2KHz, with auto phase change
 * SIN2 ,SQU12 50Hz - 2KHz, with manual phase change
 * SQU1 1Hz - 8MHz
 * SQU2 40Hz - 8MHz
 * 
 */

#include "avr/pgmspace.h"
#include "Arduino.h"
#include "U8glib.h"

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI
// PWM OUT
#define SINOUT1 11 // OC2A (PB3) require filter
#define SINOUT2 3  // OC2B (PD3) require filter
// DIGITAL OUT
#define SQUOUT1 9  // OC1A (PB1)
#define SQUOUT2 6  // OC0A (PD6)
#define SQUOUT11 7 //    - (PD7)
#define SQUOUT12 8 //    - (PB0)
// for UI
#define ENCODER_A 4
#define ENCODER_B 5
#define ENCODER_SW 2
#define LED 13


// table of 256 sine values / one sine period / stored in flash memory
const unsigned char sine256[] PROGMEM  = {
  127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,217,219,221,223,225,227,229,231,233,234,236,238,239,240,
  242,243,244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,254,254,254,254,253,253,253,252,252,251,250,249,249,248,247,245,244,243,242,240,239,238,236,234,233,231,229,227,225,223,
  221,219,217,215,212,210,208,205,203,200,198,195,192,190,187,184,181,178,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,78,
  76,73,70,67,64,62,59,56,54,51,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,16,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,16,18,20,21,23,25,27,29,31,
  33,35,37,39,42,44,46,49,51,54,56,59,62,64,67,70,73,76,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124

};
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

char buf[32];

// #define REFCLK (31372.549)  // =16MHz / 510
#define REFCLK (31376.6)      // measured

// variables used inside interrupt service declared as voilatile
volatile byte icnt1;             // var inside interrupt
volatile byte ms;              // counter incremented all 1ms

volatile unsigned long phaccu_a;   // pahse accumulator
volatile unsigned long tword_a;  // dds tuning word m for OC2A
volatile byte icnt_a;              // var inside interrupt
volatile char autoShift_a = 0;  // auto shift

volatile unsigned long phaccu_b;   // pahse accumulator
volatile unsigned long tword_b;  // dds tuning word m for OC2B
volatile byte icnt_b;            // var inside interrupt

// for menu system
byte menu_mode = 0;  // main menu = 0, edit menu = 1
byte menu_cursor = 0;  // location of menu
byte menu_edit;  // edit wave numner
char dispChannel[5][5] = { "SIN1", "SIN2", "SQU1", "SQU2" };
byte freqIndex[4] = { 7, 8, 8, 6 };  // initial setting of frequency (1KHz)
byte freqIndexMax[4] = {10, 10, 21, 19};

// frequency table of sin out
struct sin_freqtab_t {
  unsigned long tick;
  char name[10];
} sin_freqtab [10] = {
  pow(2,32)*50.0/REFCLK, "50Hz",
  pow(2,32)*100.0/REFCLK, "100Hz",
  pow(2,32)*200.0/REFCLK, "200Hz",
  pow(2,32)*300.0/REFCLK, "300Hz",
  pow(2,32)*416.66666/REFCLK, "417Hz",
  pow(2,32)*500.0/REFCLK, "500Hz",
  pow(2,32)*666.666/REFCLK, "667Hz",
  pow(2,32)*1000.0/REFCLK, "1,000Hz",
  pow(2,32)*1250.0/REFCLK, "1,250Hz",
  pow(2,32)*2000.0/REFCLK, "2,000Hz"
};

// frequency table of squ out (timer 1)
struct squ_freqtab1_t {
  unsigned char tccr1b;
  unsigned int ocr1a;    // 16bit counter
  char name[10];
} squ_freqtab1[21] = {
  0x0c, 31249, "1Hz",    // Prescaler = 1/256
  0x0b, 12499, "10Hz",    // Prescaler = 1/64
  0x0a, 25999, "40Hz",    // Prescaler = 1/8
  0x0a, 19999, "50Hz",    // Prescaler = 1/8
  0x0a, 16665, "60Hz*",   // Prescaler = 1/8
  0x0a, 9999,  "100Hz",   // Prescaler = 1/8
  0x09, 39999, "200Hz",   // Prescaler = 1/1
  0x09, 19999, "400Hz",
  0x09, 7999,  "1KHz",
  0x09, 3999,  "2KHz",
  0x09, 1999,  "4KHz",
  0x09, 799,   "10KHz",
  0x09, 399,   "20KHz",
  0x09, 199,   "40KHz",
  0x09, 79,    "100KHz",
  0x09, 39,    "200KHz",
  0x09, 19,    "400KHz",
  0x09, 7,     "1MHz",
  0x09, 3,     "2MHz",
  0x09, 1,     "4MHz",
  0x09, 0,     "8MHz",
};

// frequency table of squ out (timer 0)
struct squ_freqtab0_t {
  unsigned char tccr0b;
  unsigned char ocr0a;    // 8bit counter
  char name[10];
} squ_freqtab0[19] = {
  0x05,194,   "40Hz*",  // prescaler = 1/1024
  0x05,155,   "50Hz*",  // prescaler = 1/1024
  0x05,129,   "60Hz*",  // prescaler = 1/1024
  0x05,77,    "100Hz*",  // prescaler = 1/1024
  0x04,155,   "200Hz*",  // prescaler = 1/254
  0x04,77,    "400Hz*",  // prescaler = 1/254
  0x03,124,   "1KHz",   // prescaler = 1/64
  0x03, 61,   "2KHz*",  // prescaler = 1/64
  0x02,249,   "4KHz",   // prescaler = 1/8
  0x02, 99,   "10KHz",  // prescaler = 1/8
  0x02, 49,   "20KHz",  // prescaler = 1/8
  0x01, 199,  "40KHz",  // prescaler = 1/1
  0x01, 79,   "100KHz",
  0x01, 39,   "200KHz",
  0x01, 19,   "400KHz",
  0x01, 7,    "1MHz",
  0x01, 3,    "2MHz",
  0x01, 1,    "4MHz",
  0x01, 0,    "8MHz",
};
void draw(void) {
  u8g.setFont(u8g_font_unifontr);// Width = 8, Height = 16
  if (menu_mode == 0) {
    u8g.drawFrame(0, menu_cursor * 16, 128, 16);
    sprintf(buf, "%s: %s", dispChannel[0], sin_freqtab[freqIndex[0]].name);
    u8g.drawStr(0, 14, buf);
    sprintf(buf, "%s: %s", dispChannel[1], sin_freqtab[freqIndex[1]].name);
    u8g.drawStr(0, 30, buf);
    sprintf(buf, "%s: %s", dispChannel[2], squ_freqtab1[freqIndex[2]].name);
    u8g.drawStr(0, 46, buf);
    sprintf(buf, "%s: %s", dispChannel[3], squ_freqtab0[freqIndex[3]].name);
    u8g.drawStr(0, 62, buf);
  } else if (menu_mode == 1 || menu_mode == 2) {
    u8g.setColorIndex(1);
    u8g.drawBox(0, 0, 128, 16);
    u8g.setColorIndex(0);
    sprintf(buf, "CHANGE -> %s", dispChannel[menu_edit]);
    u8g.drawStr(0, 14, buf);
    u8g.setColorIndex(1);
    if (menu_edit < 2) {
      sprintf(buf, "FREQ: %s", sin_freqtab[freqIndex[menu_edit]].name);
    } else if (menu_edit == 2) {
      sprintf(buf, "FREQ: %s", squ_freqtab1[freqIndex[menu_edit]].name);
    } else if (menu_edit == 3) {
      sprintf(buf, "FREQ: %s", squ_freqtab0[freqIndex[menu_edit]].name);
    }
    u8g.drawStr(0, 30, buf);
    if (menu_edit == 0) { // only SIN1 autoshift mode
      sprintf(buf, "PHASE: AUTO(%02d)", autoShift_a);
      u8g.drawStr(0, 46, buf); 
    } else if (menu_edit == 1) {// only SIN2 manual shift
      u8g.drawStr(0, 46, "PHASE: < - >");  
    } else {
      u8g.drawStr(0, 46, "PHASE: FIXED");  // otherwise SIN2
    }
    u8g.drawStr(0, 62, "RETURN");    
    if (menu_mode == 1) {
      u8g.drawFrame(0, menu_cursor * 16, 128, 16);
    } else {
      u8g.drawFrame(6 * 8, menu_cursor * 16, 128 - 6 * 8, 16);  // drow a box on NUMBER FIELD
    }
  }
}

void menu_select() {  // move cursor by encoder
  char m;
  if ((m = read_enc()) == 0) {
    return;
  }
  if (menu_mode == 0) {  // move menu_cursor 0 to 3
    if (m == 1 && menu_cursor < 3) {
      menu_cursor++; 
    } else if ( m == -1 && menu_cursor > 0) {
      menu_cursor--;
    }
  } else if (menu_mode == 1) { // move menu cursor 1 to 3
    if (m == 1 && menu_cursor < 3) {
      menu_cursor++;
    } else if (m == -1 && menu_cursor > 1) {
      menu_cursor--;
    }
  } else if (menu_mode == 2) { //
    if (menu_cursor == 1) { // frequeny change
      if (m == 1 && ((freqIndex[menu_edit] + 1) <  freqIndexMax[menu_edit])) {
        freqIndex[menu_edit]++;
      } else if ( m == -1 && freqIndex[menu_edit] > 0) {
        freqIndex[menu_edit]--;
      }
    } else if (menu_cursor == 2) { // phase change
      if (menu_edit == 0) {  // only SIN1
        if (m == 1 & autoShift_a < 10) {
          autoShift_a++;
        } else if (m == -1 && autoShift_a > -10) {
          autoShift_a--;
        }
      } else if (menu_edit == 1) {  // only SIN2
        if (m == 1) {
          shift_SinPhase(1, 1);
        } else if (m == -1) {
          shift_SinPhase(1, -1);
        }
      }
    }
  }
}

void setup()
{
  // sets the digital pin as output
  pinMode(LED, OUTPUT);      
  pinMode(SINOUT1, OUTPUT);
  pinMode(SINOUT2, OUTPUT);

  pinMode(SQUOUT1, OUTPUT);    // D9 OC1A (PB1)
  pinMode(SQUOUT2, OUTPUT);    // D6 OC0A (PD6)
  pinMode(SQUOUT11, OUTPUT);   // D7 (PD7)
  pinMode(SQUOUT12, OUTPUT);   // D8 (PB0)

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  
  cbi (TIMSK0,TOIE0);              // disable Timer0 !!! delay() is now not available

  Setup_timer2();
  setup_SineFreq();
  setup_SquFreq();
}


void loop() {
  static char changed = 0;
  while(1) {
    if (ms > 50) {                 // wait fou 50m second for auto rotate
      ms=0;
      if (autoShift_a != 0) {
        shift_SinPhase(0, autoShift_a);
      }
    }
    if (digitalRead(ENCODER_SW) == 0 && changed == 0) {  // push encoder
      changed = 1;
      switch (menu_mode) {
        case 0:
          menu_mode =1;
          menu_edit = menu_cursor;
          menu_cursor = 1;  // move to line 1
          break;
        case 1:
          if (menu_cursor == 3) {  // return
            menu_mode = 0;
            menu_cursor = menu_edit;  // return to main mode
          } else if (menu_cursor == 1) {
            menu_mode = 2;    // edit number
          } else if (menu_cursor == 2 && menu_edit == 0) {
            menu_mode = 2;    // SIN1 Phase
          } else if (menu_cursor == 2 && menu_edit == 1) {
            menu_mode = 2;    // SIN2 Phase
          }
          break;
        case 2:
          menu_mode = 1;
          if (menu_edit == 0 || menu_edit == 1) {
            if (menu_cursor == 1) {
              setup_SineFreq();
            }
          } else if ( menu_edit == 2 || menu_edit == 3) {
            setup_SquFreq();
          }
          break;
        default:
          break;
        }
    } else {
      changed = 0;
    }

    // update OLED
    u8g.firstPage();  
    do {
      draw();
    } while( u8g.nextPage() );
    
  }
}

char read_enc(void){
  static unsigned char p;
  p = (p << 2 ) + ((PIND >> 4) & 0x3);
  p &= 0x0f;
  if (p == 0x7) {
    return -1;
  } else if (p == 0xd) {
    return 1;
  }
  return 0;  // status not changed
}

void setup_SineFreq() {
  cbi (TIMSK2,TOIE2);               // disable Time2 Interrupt
  tword_a = sin_freqtab[freqIndex[0]].tick;  
  tword_b = sin_freqtab[freqIndex[1]].tick;
  phaccu_a = 0;  
//  phaccu_b = 0xfc000000;            // fine tune phase
//  phaccu_b = 0xf0000000;            // fine tune phase
  phaccu_b = 0;            // = phase
  sbi (TIMSK2,TOIE2);               // enable Timer2 Interrupt 
}


void shift_SinPhase(byte ch, byte angle) {
  cbi (TIMSK2,TOIE2);               // disable Time2 Interrupt
  if (ch == 0) {
    phaccu_a = phaccu_a + (angle * 0x1000000); // shift 1/256 degree
  } else {
    phaccu_b = phaccu_b + (angle * 0x1000000); // shift 1/256 degree
  }
  sbi (TIMSK2,TOIE2);               // enable Timer2 Interrupt
  
}

void setup_SquFreq() {
  // Toggle OC1A on compare match, Mode 8 (CTC)
  TCCR1A = (0 << COM1A1) | (1 << COM1A0) | (0 << WGM11) | (0 << WGM10);
  TCCR1B= squ_freqtab1[freqIndex[2]].tccr1b;
  OCR1A = squ_freqtab1[freqIndex[2]].ocr1a;
  TCNT1  = 0; // initialize counter value to 0

  // Toggle OC0A on compare match, Mode 8 (CTC)
  TCCR0A = (0 << COM0A1) | (1 << COM0A0) | (1 << WGM01) | (0 << WGM00);
  TCCR0B= squ_freqtab0[freqIndex[3]].tccr0b;
  OCR0A = squ_freqtab0[freqIndex[3]].ocr0a;
  TCNT0 = 0;
}

//******************************************************************
// timer2 setup
void Setup_timer2() {
  // set prscaler to 1, PWM mode to phase correct PWM,  16000000/510 = 31372.55 Hz clock
  TCCR2A = (1 << COM2A1) | (0 << COM2A0) | ( 1 << COM2B1) | ( 0 << COM2B0) | ( 0 << WGM21) | ( 1 << WGM20);
  // Timer2 Clock Prescaler to : 1 =>  16000000/510 = 31372.55 Hz clock
  TCCR2B = (0 << WGM22) | (0 << CS22) | ( 0 << CS21) | ( 1 << CS20);
}


//******************************************************************
// Timer2 Interrupt Service at 31372,550 KHz = 32uSec
// this is the timebase REFCLOCK for the DDS generator
// FOUT = (M (REFCLK)) / (2 exp 32)
ISR(TIMER2_OVF_vect) {

  // for SIN1
  phaccu_a=phaccu_a+tword_a; // soft DDS, phase accu with 32 bits
  icnt_a=phaccu_a >> 24;     // use upper 8 bits for phase accu as frequency information
                         // read value fron ROM sine table and send to PWM DAC
  OCR2A=pgm_read_byte_near(sine256 + icnt_a);

  // for SIN2
  phaccu_b=phaccu_b+tword_b; // soft DDS, phase accu with 32 bits
  icnt_b = phaccu_b >> 24;     // use upper 8 bits for phase accu as frequency information
                         // read value fron ROM sine table and send to PWM DAC
  OCR2B=pgm_read_byte_near(sine256 + icnt_b);

  // other waveforms
  //  OCR2B = icnt_b; // lamp
  //  OCR2B = 256 - icnt_b; // inv lamp
  //  OCR2B = (icnt_b > 128) ? icnt_b * 2 : 256 - (icnt_b * 2); // sawtooth

  if (OCR2A < 0x80) {  // output digital by PWM info
    cbi(PORTD,7);
  } else {
    sbi(PORTD,7);
  }

  if (OCR2B < 0x80) {  // output digital by PWM info
    cbi(PORTB,0);
  } else {
    sbi(PORTB,0);
  }
  
  if(icnt1++ == 32) {  // increment variable c4ms all 1 milliseconds
    icnt1 = 0;
    ms++;
    menu_select();
  }
}

