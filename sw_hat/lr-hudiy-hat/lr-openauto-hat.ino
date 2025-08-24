/*
 * 
 * lr-openauto-hat
 *
 *   Author: Ing. Robert Sedlacek
 *   Date: February 2022
 *
 */

/*
   * Power control:
   *  There is 12 V from battery and ignition signal. When ignition is turned on, it power up arduino. 
   *  Arduino then set power latch to HIGH for keeping device powered on even if the ignition is off.
   *  Arduino after this turn on power for RPi.
   *  When ignition is turned off, rpi should handle this signal and do "poweroff" after some delay (e.g. few minutes) and
   *  after much longer delay (more than rpi needs for power off), arduino shutdown whole device by seting power latch to LOW.
   *  Anytime ignition is turned on, RPi must be also turned on.
   * 
   * Ligh sensor:
   *  Just read analog value and send it to RPi python script.
   * 
   * Wheel buttons:
   *  Just read analog value, do some filtering and send to RPi.
   *   
   * Reverse camera:
   *  Read reverse input state and send the to RPi.
   *   
   * Webasto heating:
   *  Timing webasto run with time values set by RPi. Threeway valve also can be controlled.
   *  - Obtain values from RPi. RPi must do immediately 'poweroff' after receiving heating is on.
   *  - After some delay (same as in Power control) shutsdown RPi.
   *  - Arduino run heating cycles. After all cycles shutsdown whole device.
   * 
*/

#include <avr/wdt.h>

/* Input/output/analog pin definitions */
#define PIN_D4           4  /* */
#define PIN_RPI_PWR_EN   5  /* output-enable rpi power */
#define PIN_REVERSE      6  /* Reverse gear indicator. On PCB is marked as ILUM, but day/night is  not controlled by light sensor */
#define PIN_PWR_LATCH    7  /* output-latching power */
#define PIN_WEBASTO      8  /* */
#define PIN_THR_VALVE    9  /* */
#define PIN_SW_RX        10 /* */
#define PIN_SW_TX        11 /* */
#define PIN_IGNITION     12 /* */
#define PIN_D13          13 /* */
#define PIN_STEER_WHL    A0 /* */
#define PIN_OIL_PRESS    A1 /* */
#define PIN_AUX          A2 /* */
#define PIN_LIGHT_SENS   A3 /* */
#define PIN_DS1307_SDA   A4 /* */
#define PIN_DS1307_SCL   A5 /* */
//#define PIN_A6           A6 /* */
#define PIN_AAUX         A7 /* */

/*------------------------------------------------------------------------------------------------------*/
/*  VARIABLES - GLOBAL  */
/*------------------------------------------------------------------------------------------------------*/
/* State variable */
struct {
  struct {
    bool ignition;            /* State of ignition pin */
    bool reverse_gear;        /* Reverse gear engaged */
    unsigned int light_sens;  /* Snalog value - ambient light */
    unsigned int steer_whl;   /* Analog value from steering wheel buttons */
  } in;
  struct {
    int rpi_pwr_en;
    int webasto;
  } out;
  struct {
    unsigned long last_ignition_change;
  } timestamps;
  struct {
    bool webasto_on;
    unsigned int webasto_set_on; /* On time in minutes */
    unsigned int webasto_set_off; /* Off time in minutes */
    unsigned int webasto_set_cycles; /* Number of ON cycles */
  } remote;
} state;
/*------------------------------------------------------------------------------------------------------*/
/* Settings */
#define POWERCUT_DELAY_RPI        (5*60) /* sec */
#define POWERCUT_DELAY_ARDUINO    (POWERCUT_DELAY_RPI + 2*60) /* sec - must be higher than RPi delay ! */
#define FILTER_LIGHT_SENSOR       0.005f /* low pass filter (0.0 - 1.0) */
#define FILTER_STEERINGWHEEL_BTN  0.2f   /* low pass filter (0.0 - 1.0) */
/*------------------------------------------------------------------------------------------------------*/
/*  SETUP  */
/*------------------------------------------------------------------------------------------------------*/
void setup() {
  /* Arduino power latch first after ignition on */
  pinMode(PIN_PWR_LATCH, OUTPUT);
  digitalWrite(PIN_PWR_LATCH, HIGH);
  delay(3);

  /* Clear all state variables */
  memset(&state, 0, sizeof(state));
 
  /* When ignition is turned ON, RPi is immediately powerd ON by diode beween pins D12 and D5. Unless pin PIN_RPI_PWR_EN is keep LOW.
   * So, arduino can be programmed and restarted/reseted, while RPi is still ON.
   */
  digitalWrite(PIN_RPI_PWR_EN, HIGH);
  pinMode(PIN_RPI_PWR_EN, OUTPUT); 
  digitalWrite(PIN_RPI_PWR_EN, HIGH);
  state.out.rpi_pwr_en = HIGH; /* RPi on after system poweron */
  
  /* Threeway valve output */
  pinMode(PIN_THR_VALVE, OUTPUT);
  digitalWrite(PIN_THR_VALVE, LOW);

  /* Webasto */
  pinMode(PIN_WEBASTO, OUTPUT);
  digitalWrite(PIN_WEBASTO, LOW);
  
  pinMode(LED_BUILTIN, OUTPUT);

  /* Configure inputs */
  pinMode(PIN_REVERSE, INPUT);
  pinMode(PIN_IGNITION, INPUT);
  pinMode(PIN_OIL_PRESS, INPUT);

  /* USB to serial port for debuging */
  Serial.begin(115200);

  /* Say hello */
  Serial.println("lr-openauto-hat v1.0");

  wdt_enable(WDTO_4S);
}
/*------------------------------------------------------------------------------------------------------*/
/*  INPUT read  */
/* Input read with debouncing etc. */
/*------------------------------------------------------------------------------------------------------*/
void input_read(void) {
  /* If change of IGN pin occured, update timestamp */
  if(digitalRead(PIN_IGNITION) != state.in.ignition) {
    state.in.ignition = digitalRead(PIN_IGNITION);
    state.timestamps.last_ignition_change = millis();
  }

  /* Reverse gear engaged */
  state.in.reverse_gear = digitalRead(PIN_REVERSE);

  /* Light sensor */
  state.in.light_sens = FILTER_LIGHT_SENSOR * (float)analogRead(PIN_LIGHT_SENS) + (1.0f - FILTER_LIGHT_SENSOR) * state.in.light_sens;

  /* Steering wheel buttons */
  state.in.steer_whl = FILTER_STEERINGWHEEL_BTN * (float)analogRead(PIN_STEER_WHL) + (1.0f - FILTER_STEERINGWHEEL_BTN) * state.in.steer_whl;
}
/*------------------------------------------------------------------------------------------------------*/
/* Receive data from serial port */
/*
 * Example: N,30,60,4\n
 * N - Rpi off, 30min webasto on, 60min off, 4x
 */
void parse_received_line(char *buf) {
  unsigned int auxOn,auxOff,auxCycles;
  char * strtokIndx; /* This is used by strtok() as an index */

  Serial.println(buf);
  char cmd = buf[0];

  strtokIndx = strtok(buf,",");      /* Jump over first cmd part */
 
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  auxOn = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  auxOff = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  auxCycles = atoi(strtokIndx);

  /* Validity check */
  if (auxOn  < 5    ||
      auxOff < 5    ||
      auxOn  > 3600 ||
      auxOff > 3600) {
    Serial.println("Warning: Time must be between 5 and 3600 minutes");
    return;
  }

  if(auxCycles < 1 || auxCycles > 20) {
    Serial.println("Warning: Cycles must be between 1 and 20");
    return;    
  }

  switch(cmd) {
    case 'N': //Night heating
      state.remote.webasto_on = true;
      Serial.print("Webasto night mode ON");
      break;
    default:
      Serial.println("Warning: Unknown command");
      return;
  }

  state.remote.webasto_set_on = auxOn;
  state.remote.webasto_set_off = auxOff;
  state.remote.webasto_set_cycles = auxCycles;

  Serial.print(" - On:");
  Serial.print(state.remote.webasto_set_on, DEC);
  Serial.print("min, Off:");
  Serial.print(state.remote.webasto_set_off, DEC);
  Serial.print("min, On:");
  Serial.print(state.remote.webasto_set_cycles, DEC);
  Serial.println("x");
}
/*------------------------------------------------------------------------------------------------------*/
/* Receive data from serial port */
void remote_receive_serial(void) {
  const byte buffLen = 32; /* Size of receive serial buffer (for each line) */
  static char receivedChars[buffLen];   /* An array to store the received data */

  static byte ndx = 0;
  char endMarker = '\n'; /* 0x0D-CarriageReturn  0x0A-NewLine  */
  char rc;
    
  while (Serial.available() > 0) {
  rc = Serial.read();
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= buffLen) {
        ndx = buffLen - 1;
      }
    } else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;

      /* Parse received line */
      parse_received_line(receivedChars);
    }
  }
}
/*------------------------------------------------------------------------------------------------------*/
/*  TASKS  */
/* Simplest tasking is used - every task must return quickly as posible. */
/*------------------------------------------------------------------------------------------------------*/
/* Arduino power managment */
void task_arduino_pwr(void) {
  if(state.in.ignition != HIGH) { /* Ignition off */
    unsigned long timeS = (millis() - state.timestamps.last_ignition_change) / 1000;

    /* Keep arduino pwr on during heating */
    if(state.remote.webasto_on) {
      return;
    }
    
    if(timeS > POWERCUT_DELAY_ARDUINO) {
      Serial.println("Cutting arduino PWR...");
      while(1){ /* Cycle end forever */
        digitalWrite(PIN_PWR_LATCH, LOW);
      }
    }
  } else { /* Ignition on */
    if(millis() - state.timestamps.last_ignition_change > 50) {
      state.out.rpi_pwr_en = HIGH;
    }    
  }
}
/*------------------------------------------------------------------------------------------------------*/
/* RPi power managment */
void task_rpi_pwr(void) {
  /* Turn off RPi, if ignition off and RPi still on */
  if(state.in.ignition != HIGH && state.out.rpi_pwr_en != LOW) {
    int timeS = (millis() - state.timestamps.last_ignition_change) / 1000;

    /* Cut off power */
    if(timeS > POWERCUT_DELAY_RPI) {
      state.out.rpi_pwr_en = LOW;
    }
  }  
}
/*------------------------------------------------------------------------------------------------------*/
/*  */
void task_webasto_heating(void) {
  static bool isPause = false; /* True if off cycle */
  static int ccycle = 0; /* Current cycle */

  static unsigned long webOnLastTime=0;
  static unsigned long webOffLastTime=0;  

  /* Do nothing if on is not set */
  if(state.remote.webasto_on == false) {
    state.out.webasto = LOW;
    ccycle = 0;
    isPause = false;
    return;
  }

  /* If first cycle, init time variables, ... */
  if(ccycle == 0) {
    ccycle=1;
    webOnLastTime = millis();
    webOffLastTime = millis();
  }

  /* Main webasto timer control */
  if(isPause) {
    state.out.webasto = LOW;
    for(;millis() - webOffLastTime > 1000ul*60ul*state.remote.webasto_set_off; webOffLastTime=millis()) {
      //ccycle++;
      webOnLastTime = millis();
      isPause = false;
    }
  } else {
    state.out.webasto = HIGH;
    for(;millis() - webOnLastTime > 1000ul*60ul*state.remote.webasto_set_on; webOnLastTime=millis()) {
      ccycle++;
      webOffLastTime = millis();
      isPause = true;
    }
  }

  /* Last cycle, turn off all */
  if(ccycle > state.remote.webasto_set_cycles) {
    state.remote.webasto_on = false;
    state.out.webasto = LOW;
    isPause = false;
    ccycle = 0;
  }
}
/*------------------------------------------------------------------------------------------------------*/
/* Just Arduino onboard led blinking - shared with D13 ! */
void task_led_blink(void) {
  static int last = HIGH;
  static unsigned long LastTime=0; 
  
  for(;millis() - LastTime > 250 /* ms */; LastTime=millis()) {
    digitalWrite(LED_BUILTIN, last);
    if(last == HIGH) {
      last = LOW;
    } else {
      last = HIGH;
    }  
  }
}

/*------------------------------------------------------------------------------------------------------*/
/*  OUTPUT write  */
/* Writing outputs, serial outputs, etc. */
/*------------------------------------------------------------------------------------------------------*/
void output_write(void) {
  /* Rpi power */
  digitalWrite(PIN_RPI_PWR_EN, state.out.rpi_pwr_en);    

  /* Webasto control */
  digitalWrite(PIN_WEBASTO, state.out.webasto);

  static unsigned long printLastTime=0; 
  for(;millis() - printLastTime > 25 /* ms */; printLastTime=millis()) {
    /* */
    Serial.print("STATES IGN:");
    Serial.print(state.in.ignition);
  
    Serial.print(" LSENS:");
    Serial.print(state.in.light_sens);

    Serial.print(" WHBTN:");
    Serial.print(state.in.steer_whl);

    Serial.print(" HEATER:");
    Serial.print(state.remote.webasto_on);

    Serial.print(" REVERSE:");
    Serial.println(state.in.reverse_gear);
  }
}

/*------------------------------------------------------------------------------------------------------*/
/*  LOOP  */
/* Loop like in PLCs. Load inputs first, then tasks and write to outputs at the end. */
/*------------------------------------------------------------------------------------------------------*/
void loop() {
  /* Reset watchdog*/
  wdt_reset();
  /* Set up WDT interrupt */
  WDTCSR = (1 << WDCE) | (1 << WDE);
  /* Start watchdog timer with 4s prescaller */
  WDTCSR = (1 << WDIE) | (1 << WDE) | (1 << WDP3) | (1 << WDP0);
  
  input_read();
  remote_receive_serial();
  
  task_arduino_pwr();
  task_rpi_pwr();

  task_webasto_heating();
  task_led_blink();
  
  output_write();
}
