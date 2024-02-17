//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "clock.h"
#include "uart0.h"
#include "wait.h"
#include "tm4c123gh6pm.h"
#include "moto_pro.h"
#include "eeprom.h"


#define MAX_CHARS 80
#define MAX_FIELDS 5

// EEPROM
#define NVIC_APINT_VECT_RESET   0x00000001  // System Reset
#define NVIC_APINT_SYSRESETREQ  0x00000004  // System Reset Request

#define NUM_FIELDS 8
#define MAX_EVENT 20
#define MIN_EVENT 16
#define Fields 4
#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
  {
    uint8_t event;
    uint8_t sensor;
    uint16_t min_dist;
    uint16_t max_dist;

    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
  } USER_DATA;

  USER_DATA events[MIN_EVENT];

// LED bitbands
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

// PortF masks LED
#define GREEN_LED_MASK 8
#define BLUE_LED_MASK 4

#define trig_0      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4))) // Channel - 0
#define trig_1      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4))) // Channel - 1
#define trig_2      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4))) // Channel - 2

// Port C, D masks
#define echo_0 16            // PC4 - Channel 0
#define trigger_0 32        // PC5 - Channel 0

#define echo_1 64           // PC6 - Channel 1
#define trigger_1 128      // PC7 - Channel 1

#define echo_2 1            // PD0 - Channel 2
#define trigger_2 2        // PD1 - Channel 2

void getsUart0(USER_DATA* data)
{
  // initialize local variable count to 0
  uint8_t count = 0;

  while (true)
  {
      char c = getcUart0();
      if ((c == 8 || c == 127) && count > 0) // for Backspace
      {
          count--;
      }
      else if (c == 13)                     // Enter-Key found
      {
          data->buffer[count] = '\0';
          return;
      }
      else if (c >= 32 && count < 80)     // for SPACE
      {
          data->buffer[count] = c;
          count++;
      }
          if (count == 80)                // Reached MAX Chars
          {
              data->buffer[count] = '\0';
              return;
          }
      }
  }

//** -------- 02. Parse Field() -----------**//
//-----------------------------------------------

void parseFields(USER_DATA* data)
{
uint8_t i = 0;
bool flag = false;
data->fieldCount = 0;

while(data->buffer[i] != '\0')
{
  char c = data->buffer[i];
  if (c == 0 || c == 32 || c <= 44 && c <= 46)
        {
            data->buffer[i]='\0';
            flag = false;
        }
        if ((c >= 65 && c <= 90) || (c >= 97 && c <= 122))
            {
            if (flag == false)
            {
               data->fieldType[data->fieldCount] = 'a';
               data->fieldPosition[data->fieldCount]= i;
               data->fieldCount++;
               flag = true;
            }
            }
        else if ( c >= 48 && c <= 57)
            {
                if (flag == false)
                {
                data->fieldType[data->fieldCount] = 'n';
                data->fieldPosition[data->fieldCount]= i;
                data->fieldCount++;
                    flag = true;
                    }
                }
            i++;
    }

}

// Get field integer

int32_t getFieldInteger(USER_DATA* data,uint8_t fieldNumber)
{
    uint32_t PrevPos = 0;
    uint32_t result = 0;
    uint32_t i = data -> fieldPosition[fieldNumber];
    if(data -> fieldType[fieldNumber]=='n')
    {
        while(data->buffer[i]!='\0')
        {
            result = (data->buffer[i]-'0') + (PrevPos*10);
            PrevPos = result;
            i++;
        }
        return result;
    }
    else
    {
        return 0;
    }
}

// Get field string

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if (data -> fieldType[fieldNumber]=='a')
    return &data -> buffer[data -> fieldPosition[fieldNumber]];
    else
    return 0;
}

//-----------------------------------------------------------------------------
//Compare two strings. Buffer source and char string
// String Comparison //

bool compareTwoString(const char data_buffer[], const char strCommand[]);

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
   bool compare = compareTwoString(&data->buffer[data -> fieldPosition[0]], strCommand);
    uint8_t k = data -> fieldCount - 1;
    if (compare == true && k >= minArguments)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool compareTwoString(const char data_buffer[], const char strCommand[])
        {
        uint8_t i = 0;
        while (strCommand[i] != '\0' && data_buffer[i] == strCommand[i])
        {
            i++;
        }
        return (data_buffer[i] == '\0' && strCommand[i] == '\0');
}

void setEvent(uint8_t event, uint8_t sensor, uint16_t min_dist, uint16_t max_dist)
{
    events[event].sensor = sensor;
    events[event].min_dist = min_dist;
    events[event].max_dist = max_dist;

    writeEeprom((event * NUM_FIELDS) + 0, sensor);
    writeEeprom((event * NUM_FIELDS) + 1, min_dist);
    writeEeprom((event * NUM_FIELDS) + 2, max_dist);
}

void readEvents(USER_DATA* data)
{
        uint8_t i = 0;
        for (i = 0; i < 16; i++)
        {
            uint16_t add = 4 * i;
            events[i].sensor = readEeprom(add);
            events[i].min_dist = readEeprom(add + 1);
            events[i].max_dist = readEeprom(add + 2);

            char str[40];
            snprintf(str, sizeof(str), "event: %d, sensor: %" PRIu32 ", min_dist: %" PRIu32 ", max_dist: %" PRIu32 " (mm)\n", i, events[i].sensor, events[i].min_dist, events[i].max_dist);
            putsUart0(str);
        }
}

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
uint32_t state = 0;
uint32_t channel = 0;
uint32_t time_stamp1 = 0;
uint32_t time_stamp2 = 0;
uint32_t time = 0;
uint32_t distance[3] = {0, 0, 0};
uint16_t eeprom_address = 0;

bool detect_echo = true;

bool echorec_1 = true;
bool echorec_2 = true;
bool echorec_0 = true;


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0 | SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0 | SYSCTL_RCGCWTIMER_R1 | SYSCTL_RCGCWTIMER_R2;  //| SYSCTL_RCGCWTIMER_R5;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configuration
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK | BLUE_LED_MASK;

    GPIO_PORTC_DIR_R |= trigger_0 | trigger_1;
    GPIO_PORTD_DIR_R |= trigger_2;
    GPIO_PORTC_DEN_R |= trigger_0 | trigger_1;
    GPIO_PORTD_DEN_R |= trigger_2;

    GPIO_PORTC_AFSEL_R |= echo_0 | echo_1;
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC4_M | ~GPIO_PCTL_PC6_M;
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_WT0CCP0 | GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= echo_0 | echo_1;

    GPIO_PORTD_AFSEL_R |= echo_2;
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD0_M;
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD0_WT2CCP0;
    GPIO_PORTD_DEN_R |= echo_2;
}

void initEeprom(void)
{
    SYSCTL_RCGCEEPROM_R = 1;
    _delay_cycles(3);
    while (EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

void writeEeprom(uint16_t add, uint32_t data)
{
    EEPROM_EEBLOCK_R = add >> 4;
    EEPROM_EEOFFSET_R = add & 0xF;
    EEPROM_EERDWR_R = data;
    while (EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

uint32_t readEeprom(uint16_t add)
{
    EEPROM_EEBLOCK_R = add >> 4;
    EEPROM_EEOFFSET_R = add & 0xF;
    return EEPROM_EERDWR_R;
}

void enableTimerMode()
    {
    //Timer1
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 20000000 ;                       // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);              // turn-on interrupt 37 (TIMER1A) in NVIC

    // WTimer2
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER2_CFG_R = TIMER_CFG_16_BIT;
    WTIMER2_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
    WTIMER2_CTL_R = TIMER_CTL_TAEVENT_BOTH;
    WTIMER2_IMR_R = TIMER_IMR_CAEIM;
    WTIMER2_TAV_R = 0;
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;
    NVIC_EN3_R |= 1 << (INT_WTIMER2A-16-96);

    // WTimer1

    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER1_CFG_R = TIMER_CFG_16_BIT;
    WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_BOTH;
    WTIMER1_IMR_R = TIMER_IMR_CAEIM;
    WTIMER1_TAV_R = 0;
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;
    NVIC_EN3_R |= 1 << (INT_WTIMER1A-16-96);

    //WTimer0
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER0_CFG_R = TIMER_CFG_16_BIT;
    WTIMER0_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
    WTIMER0_CTL_R = TIMER_CTL_TAEVENT_BOTH;
    WTIMER0_IMR_R = TIMER_IMR_CAEIM;
    WTIMER0_TAV_R = 0;
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;
    NVIC_EN2_R |= 1 << (INT_WTIMER0A-16-64);

    }

// Distance Measurement
uint32_t measure_mm(uint8_t n)
{
    distance[channel] = (343000 * (time * 25e-9)) / 2.0;  // time = (time_stamp2 - time_stamp1) - pulse duration
    return distance[channel];
}

void echo_wideTimer0Isr()
{
    if (state == 1)
    {
        WTIMER0_TAV_R = 0;
        time_stamp1 = WTIMER0_TAV_R;
        state = 2;
     } else if (state == 2)
        {
        time_stamp2 = WTIMER0_TAV_R;
        time = time_stamp2-time_stamp1;
        distance[0] = measure_mm(0);
        detect_echo = true;
        state = 3;
        }
    WTIMER0_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void echo_wideTimer1Isr()
{
    if (state == 1)
    {
        WTIMER1_TAV_R = 0;
        time_stamp1 = WTIMER1_TAV_R;
        state = 2;
     } else if (state == 2)
        {
        time_stamp2 = WTIMER1_TAV_R;
        time = time_stamp2-time_stamp1;
        distance[1] = measure_mm(1);
        detect_echo = true;
        state = 3;
        }
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void echo_wideTimer2Isr()
{
    if (state == 1)
    {
        WTIMER2_TAV_R = 0;
        time_stamp1 = WTIMER2_TAV_R;
        state =2;
     }
    else if (state == 2)
        {
        time_stamp2 = WTIMER2_TAV_R;
        time = time_stamp2-time_stamp1;
        distance[2] = measure_mm(2);
        detect_echo = true;
        state = 3;
        }
    WTIMER2_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void timer1Isr()
{
    if(!detect_echo)
    {
     distance[channel] = 0;
    }
     channel = (channel + 1) % 3;
     detect_echo = true;
     state = 0;

TIMER1_ICR_R = TIMER_ICR_TATOCINT;           // Clear interrupt flag
}

void trigger()
{
    if(detect_echo == true)
    {
        detect_echo = false;

    if(channel == 0)
        {
           if(echorec_0 == true)
           {
               trig_0 = 1;
               waitMicrosecond(10);
               trig_0 = 0;
               state = 1;
               echorec_0 = false;
           }
        }
        else if (channel == 1)
        {
            if(echorec_1 == true)
            {
               trig_1 = 1;
               waitMicrosecond(10);
               trig_1 = 0;
               state = 1;
               echorec_1 = false;
            }
        }
        else if(channel == 2)
        {
            if(echorec_2 == true)
            {
               trig_2 = 1;
               waitMicrosecond(10);
               trig_2 = 0;
               state = 1;
               echorec_2 = false;
            }
        }
    }
}

void initiateSystemReset(void)
{
    NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
    while(true);
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{

      //Initialize hardware
      waitMicrosecond(2000000);
      USER_DATA data;
      initHw();
      initUart0();
      initMoto();
      enableTimerMode();
      setMoto(0);
      setUart0BaudRate(115200, 40e6);
      char str[40];

      while (true)
      {
       trigger();
       if (state == 3)
       {
       state = 0;
       snprintf(str, sizeof(str), "dist_mm: 0:  %7"PRIu32" (mm)  \n",distance[0]);
       putsUart0(str);
       snprintf(str, sizeof(str), "dist_mm: 1:  %7"PRIu32" (mm)  \n",distance[1]);
       putsUart0(str);
       snprintf(str, sizeof(str), "dist_mm: 2:  %7"PRIu32" (mm)  \n\n",distance[2]);
       putsUart0(str);

       if(distance[0] > 250 && distance[0] < 375 || distance[1] > 50 && distance[1] < 280 || distance[2] > 315  && distance[2] < 400)
       {
       BLUE_LED = 0;
       GREEN_LED = 1;
       WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
       WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
       WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;
       setMoto(512);
//     waitMicrosecond(370000);
//     setMoto(0);
       WTIMER0_CTL_R |= TIMER_CTL_TAEN;
       WTIMER1_CTL_R |= TIMER_CTL_TAEN;
       WTIMER2_CTL_R |= TIMER_CTL_TAEN;
       } else
       {
       GREEN_LED = 0;
       setMoto(0);
       BLUE_LED = 1;
       }
//     waitMicrosecond(50000);
       }

       if(kbhitUart0())
       {
       getsUart0(&data);
       putsUart0(data.buffer);
       putsUart0("\t \n");
       parseFields(&data);
       uint8_t i;
       for (i = 0; i < data.fieldCount; i++)
       {
        putcUart0(data.fieldType[i]);
        putsUart0("\t");
        putsUart0(&data.buffer[data.fieldPosition[i]]);
        putcUart0('\n');
       }
       if(isCommand(&data, "reboot", 0))
       {
        int32_t reboot = getFieldInteger(&data, 1);
        initiateSystemReset();
      }
       }
      }
}
