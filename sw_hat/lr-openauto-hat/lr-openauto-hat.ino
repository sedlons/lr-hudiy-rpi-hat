/*
 * 
 * lr-openauto-hat
 *
 *   Author: Ing. Robert Sedlacek
 *   Date: February 2022
 *
 */

#include <avr/wdt.h>

/* Input/output/analog pin definitions */
#define PIN_D4           4  /* */
#define PIN_RPI_PWR_EN   5  /* output-enable rpi power */
#define PIN_ILUM         6  /* */
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
#define PIN_A6           A6 /* */
#define PIN_AAUX         A7 /* */

/*------------------------------------------------------------------------------------------------------*/
/*  MACRO's  */
/*------------------------------------------------------------------------------------------------------*/
/* Simple period */
#define TASK_PERIOD(__NAME__, __PERIOD__) static unsigned long __NAME__##LastTime=0; for(;millis() - __NAME__##LastTime > __PERIOD__; __NAME__##LastTime=millis())
/* Defining wheel button resistance */
#define WHEEL_BTN_RES(__RES__) {__RES__, 0, 0}
/*------------------------------------------------------------------------------------------------------*/
/*  VARIABLES - GLOBAL  */
/*------------------------------------------------------------------------------------------------------*/
/* State variable */
struct {
  struct {
    int ignition;          /* State of ignition pin */
    int steer_whl_btn;     /* Steering wheel button reconstructed from analog value */
  } in;
  struct {
    int rpi_pwr_en;
    int webasto;
  } out;
  struct {
    unsigned long last_ignition_change;
  } timestamps;
  struct {
    unsigned int light_sens;
    unsigned int oil_press;
    unsigned int steer_whl;   /* Analog value from steering wheel buttons */
  } analog;
  struct {
    bool webasto_on_rpi_off;
    bool webasto_on_rpi_on;
    unsigned int webasto_set_on; /* On time in minutes */
    unsigned int webasto_set_off; /* Off time in minutes */
    unsigned int webasto_set_cycles; /* Number of ON cycles */
  } remote;
} state;
/*------------------------------------------------------------------------------------------------------*/
/* Settings */
struct {
  int shutdown_delay_cmdRpi;      /* sec */    /* Poweroff command for RPi delay after IGN off */
  int shutdown_delay_pwrRpi;      /* sec */    /* Shutdown of RPi delay after IGN off */
  int shutdown_delay_arduino;  /* sec */    /* Shutdown of Arduino delay after IGN off */
  float light_sensor_filter;
  float steer_whl_filter;
} settings;
/*------------------------------------------------------------------------------------------------------*/
/* Pre fill threshold values etc. based on button resistance array */
void setup_steering_wheel_button_calc(void);
/* Steering wheel buttons struct for distinction */
typedef struct wheel_btn{ 
  float ohm;  /* Button resistor value in ohm */
  int lthr;   /* AD value - low threshold */
  int ad;     /* AD value (0-1024) */
  int hthr;   /* AD value - upper threshold */
} wheel_btn;
/* Array of structs with steering wheel resistances. Must be in ascending order */
wheel_btn resistances[] = {
  /* 0 - volume down */ WHEEL_BTN_RES(0),         
  /* 1 - volume up */   WHEEL_BTN_RES(390),       
  /* 2 - search up */   WHEEL_BTN_RES(390 + 470),   
  /* 3 - search down */ WHEEL_BTN_RES(390 + 470 + 820),
  /* 4 - mode */        WHEEL_BTN_RES(390 + 470 + 820 + 1000),
  /* 5 - end */         WHEEL_BTN_RES(390 + 470 + 820 + 1000 + 1000),
};
/* Length of resistances */
const int resistance_len = sizeof(resistances) / sizeof(wheel_btn);
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

  /* Configure outputs */
  pinMode(PIN_RPI_PWR_EN, INPUT);
  //digitalWrite(PIN_RPI_PWR_EN, HIGH);
  
  pinMode(PIN_THR_VALVE, OUTPUT);
  digitalWrite(PIN_THR_VALVE, LOW);
  
  pinMode(PIN_WEBASTO, OUTPUT);
  digitalWrite(PIN_WEBASTO, LOW);
  
  pinMode(LED_BUILTIN, OUTPUT);

  /* Configure inputs */
  pinMode(PIN_ILUM, INPUT);
  pinMode(PIN_IGNITION, INPUT);
  //pinMode(PIN_STEER_WHL, INPUT);
  pinMode(PIN_OIL_PRESS, INPUT);
  //pinMode(PIN_LIGHT_SENS, INPUT);

  /* USB to serial port for debuging */
  Serial.begin(115200);

  /* DEFAULT Settings */
    /* TODO: Move to eeprom */
  settings.shutdown_delay_arduino    = 5 * 60; /* 5 minutes */
  settings.shutdown_delay_pwrRpi     = 4 * 60; /* 4 minutes MUST BE LOWER THAN shutdown_delay_arduino */
  settings.shutdown_delay_cmdRpi     = 2 * 60; /* 2 minutes MUST BE LOWER THAN shutdown_delay_pwrRpi */
  settings.light_sensor_filter       = 0.005f; /* Light sensor analog value filter (0.0 - 1.0) */
  settings.steer_whl_filter          = 0.05f;

  setup_steering_wheel_button_calc();

  /* Say hello */
  Serial.println("lr-openauto-hat v1.0");

  wdt_enable(WDTO_4S);
}
/*------------------------------------------------------------------------------------------------------*/
void setup_steering_wheel_button_calc(void) {
  /* Onboard pullup resistor value */
  const float res_R9 = 4300; /* ohm */
  
  /* Prefill */
  for (int i = 0; i < resistance_len; i++) {
    resistances[i].ad = 1024.0f * resistances[i].ohm / (resistances[i].ohm + res_R9);
    
    float part = 0.1*(float)resistances[i].ad;
    resistances[i].lthr = resistances[i].ad - part;
    resistances[i].hthr = resistances[i].ad + part;

    Serial.print("Steering wheel disctinction:");
    Serial.print((int)resistances[i].ohm, DEC); 
    Serial.print(": "); 
    Serial.print(resistances[i].lthr, DEC);
    Serial.print("<");
    Serial.print(resistances[i].ad, DEC);
    Serial.print(">"); 
    Serial.println(resistances[i].hthr, DEC);
  }
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
    Serial.print("Ignition: ");
    if(state.in.ignition) {
      Serial.println("on");  
    } else {
      Serial.println("off");  
    }
  }

  /* Light sensor */
  state.analog.light_sens = settings.light_sensor_filter * (float)analogRead(PIN_LIGHT_SENS) + (1.0f - settings.light_sensor_filter) * state.analog.light_sens;

  /* Light sensor */
  state.analog.steer_whl = settings.steer_whl_filter * (float)analogRead(PIN_STEER_WHL) + (1.0f - settings.steer_whl_filter) * state.analog.steer_whl;
}
/*------------------------------------------------------------------------------------------------------*/
/* Receive data from serial port */
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
      state.remote.webasto_on_rpi_off = true;
      Serial.print("Webasto night mode ON");
      break;
    case 'M': //Movie with heating
      state.remote.webasto_on_rpi_on = true;
      Serial.print("Webasto movie mode ON");
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

    /* Just leave arduino pwr on during movie watching */
    if(state.remote.webasto_on_rpi_on || state.remote.webasto_on_rpi_off) {
      return;
    }

    TASK_PERIOD(shArd,1000 /* ms */) {
      Serial.print("Shutdown in ");
      Serial.println(settings.shutdown_delay_arduino - timeS, DEC);
    }
    
    if(timeS > settings.shutdown_delay_arduino) {
      Serial.println("Cutting arduino PWR...");
      while(1){ /* Cycle end forever */
        digitalWrite(PIN_PWR_LATCH, LOW);
      }
    }
  } else { /* Ignition on */
    if(millis() - state.timestamps.last_ignition_change > 50) {
      /* Print text only when change */
      if(state.out.rpi_pwr_en != HIGH) {
        Serial.println("RPi power ON"); 
      }
      state.out.rpi_pwr_en = HIGH;
    }    
  }
}
/*------------------------------------------------------------------------------------------------------*/
/* RPi power managment */
void task_rpi_pwr(void) {
  /* Just leave rpi during movie watching */
  if(state.remote.webasto_on_rpi_on) {
    return;
  }
  
  /* Turn off rpi, if ignition off and Pi already on */
  if(state.in.ignition != HIGH && state.out.rpi_pwr_en != LOW) {
    int timeS = (millis() - state.timestamps.last_ignition_change) / 1000;

    /* Send command to RPI for software off */
    if(timeS > settings.shutdown_delay_cmdRpi) {
      TASK_PERIOD(shRpi,3000 /* ms */) {
        Serial.println("RPi power OFF");
      }
    }

    /* Cut off power */
    if(timeS > settings.shutdown_delay_pwrRpi) {
      pinMode(PIN_RPI_PWR_EN, OUTPUT);
      state.out.rpi_pwr_en = LOW;
      delay(150);
      pinMode(PIN_RPI_PWR_EN, INPUT);
    }
  }  
}
/*------------------------------------------------------------------------------------------------------*/
/*  */
void task_light_sensor(void) {
  TASK_PERIOD(,1000 /* ms */) {
    Serial.print("Light sens: ");
    Serial.println(state.analog.light_sens, DEC);
  }
}
/*------------------------------------------------------------------------------------------------------*/
/*  */
void task_steering_wheel(void) {
  /* Send button info every x ms */
  TASK_PERIOD(btnsense, 200) {
    for(int i = 0; i < resistance_len; i++) {
      if(resistances[i].lthr >= state.analog.steer_whl && state.analog.steer_whl <= resistances[i].hthr) {
        Serial.print("Steer wheel btn: ");
        Serial.println(i, DEC);        
        break;
      }
    }
  }
  
  TASK_PERIOD(text,1000 /* ms */) {
    Serial.print("Steer wheel sens: ");
    Serial.println(state.analog.steer_whl, DEC);
  }
  
}
/*------------------------------------------------------------------------------------------------------*/
/*  */
void task_oil_pressure(void) {
  
}
/*------------------------------------------------------------------------------------------------------*/
/*  */
void task_ilumination(void) {
  
}
/*------------------------------------------------------------------------------------------------------*/
/*  */
void task_communication(void) {
  
}
/*------------------------------------------------------------------------------------------------------*/
/*  */
void task_webasto_heating(void) {
  static bool isPause = false; /* True if off cycle */
  static int ccycle = 0; /* Current cycle */

  static unsigned long webOnLastTime=0;
  static unsigned long webOffLastTime=0;  

  /* Do nothing if on is not set */
  if(!(state.remote.webasto_on_rpi_on || state.remote.webasto_on_rpi_off)) {
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
    state.remote.webasto_on_rpi_on = false;
    state.remote.webasto_on_rpi_off = false;
    state.out.webasto = LOW;
    isPause = false;
    ccycle = 0;
  }
}
/*------------------------------------------------------------------------------------------------------*/
/* Just Arduino onboard led blinking - shared with D13 ! */
void task_led_blink(void) {
static int last = HIGH;
  TASK_PERIOD(,150 /* ms */) {
    digitalWrite(LED_BUILTIN, last);
    if(last == HIGH) {
      last = LOW;
    } else {
      last = HIGH;
    }  
  } /* TASK_PERIOD_END */
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

  TASK_PERIOD(outInfo,1000 /* ms */) {
    Serial.print("Webasto: ");
    Serial.println((state.out.webasto) ? "ON" : "Off");
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
  task_light_sensor();
  task_steering_wheel();
  task_oil_pressure();
  task_ilumination();
  task_communication();
  task_webasto_heating();
  task_led_blink();
  
  output_write();
}
