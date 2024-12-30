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
} state;
/*------------------------------------------------------------------------------------------------------*/
/* Settings */
struct {
  int shutdown_delay_rpi;      /* sec */    /* Shutdown of RPi delay after IGN off */
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
  settings.shutdown_delay_rpi        = 2 * 60; /* 2 minutes MUST BE LOWER THAN shutdown_delay_arduino */
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
/*  TASKS  */
/* Simplest tasking is used - every task must return quickly as posible. */
/*------------------------------------------------------------------------------------------------------*/
/* Arduino power managment */
void task_arduino_pwr(void) {
  if(state.in.ignition != HIGH) {
    int timeS = (millis() - state.timestamps.last_ignition_change) / 1000;
    TASK_PERIOD(,1000 /* ms */) {
      Serial.print("Shutdown in ");
      Serial.println(settings.shutdown_delay_arduino - timeS, DEC);
    }
    if(timeS > settings.shutdown_delay_arduino) {
      digitalWrite(PIN_PWR_LATCH, LOW);
      Serial.println("Cutting arduino PWR...");
      while(1);
    }
  } else {
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
  if(state.in.ignition != HIGH && state.out.rpi_pwr_en != LOW) {
    if(millis() - state.timestamps.last_ignition_change > (unsigned long)settings.shutdown_delay_rpi*1000ul) {
      Serial.println("RPi power OFF");
      pinMode(PIN_RPI_PWR_EN, OUTPUT);
      //state.out.rpi_pwr_en = LOW;
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
  digitalWrite(PIN_RPI_PWR_EN, state.out.rpi_pwr_en);
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
  
  task_arduino_pwr();
  task_rpi_pwr();
  task_light_sensor();
  task_steering_wheel();
  task_oil_pressure();
  task_ilumination();
  task_communication();
  task_led_blink();
  
  output_write();
}
