/* ------------------- Includes ------------------- */

#include "pid.h"
#include "metrics.h"
#include "can.h"
#include "mcp2515.h"
#include <string.h>
#include <algorithm>
#include <queue>
#include <hardware/flash.h>

/* ------------------ Variables ------------------- */

const int LED_PIN = 22;
const int interruptPin = 20;
const int DAC_RANGE = 4096;
const int avSize = 11;
const float P_MAX = 0.108;  // W

unsigned long restart = 0;
unsigned long elapsed = 0;
unsigned long prevTime = 0;

int controlled;  // 0 - controller off

float pwm = 0;
float lux = 0;

// for metrics
int numSamples;
float energyCons, visibilityError, flickering, prevDuty, prevPrevDuty;

// last minute buffer
std::queue<float> luxQ;
std::queue<float> pwmQ;

// states for 3 desks (passar para classes no futuro)
int desk = 0;
int occupancy[2];   // occupancy state
int antiWindup[2];  // anti - windup state
int feedback[2];    // feedback state
float gain[3] = { 0, 302.38, 0 };

// stream variables
char variable[2];  // variable to be streamed
int stream = 0;    // if stream of variable is on or off
int desk_stream = 0;

String inString = "";

// create pid controllers
// explicit pid_AntiWindup(float _h, float _K = 1, float b_ = 1, float Ti_ = 0.8, float Tt_ = 1, float Td_ = 0, float N_ = 10);
pid_AntiWindup pidComplex {       0.01231,      500,          0.00053712,    0.53,            0.05};
//  explicit pid_Simple( float _h, float _K = 1, float b_ = 1, float Ti_ = 1, float Td_ = 0, float N_ = 10);
pid_Simple pidSimple{ 0.01231, 1000, 0.00026865, 0.53};
float ref{ 0.0 };

// can stuff
MCP2515 can0{ spi0, 17, 19, 16, 18, 10000000 };
volatile byte data_available = false;
unsigned long time_to_write = 0, write_delay = 1000;
uint8_t this_pico_flash_id[8], node_address;
struct can_frame canMsgTx, canMsgRx;
unsigned long counterTx = 0, counterRx = 0;
MCP2515::ERROR err;

void read_interrupt(unsigned gpio, uint32_t events) {
  (void)gpio;
  (void)events;
  data_available = true;
}

void setup() {  // the setup function runs once
  Serial.begin(115200);
  analogReadResolution(12);     // default is 10
  analogWriteFreq(60000);       // 60KHz, about max
  analogWriteRange(DAC_RANGE);  // 100% duty cycle

  numSamples = 0;
  energyCons = 0;
  visibilityError = 0;
  flickering = 0;
  prevDuty = 0;
  prevPrevDuty = 0;

  controlled = 0;

  //canbus cenas
  flash_get_unique_id(this_pico_flash_id);
  node_address = this_pico_flash_id[7];  //NODE ADDRESS COMES FROM SERIAL NUMBER
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode();

  gpio_set_irq_enabled_with_callback(interruptPin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt);

  restart = millis();
  time_to_write = millis() + write_delay;
}
/* -------------------- Buffer -------------------- */
void printMinute(std::queue<float> q) {
  while (!q.empty()) {
    Serial.printf("%.2f, ", q.front());
    q.pop();
  }
}

/* --------------------- Main --------------------- */
void loop() {
  // unsigned long init = micros(), end;
  // if (millis() >= time_to_write) {
  //   canMsgTx.can_id =  node_address;
  //   canMsgTx.can_dlc = 8;
  //   unsigned long div = counterTx * 10;
  //   for ( int i = 0; i < 8; i++ )
  //     canMsgTx.data[7 - i] = '0' + ((div /= 10) % 10);
  //   err = can0.sendMessage(&canMsgTx);
  //   Serial.print("Sending message ");
  //   Serial.print( counterTx );
  //   Serial.print(" from node ");
  //   Serial.println( node_address, HEX );
  //   counterTx++;
  //   time_to_write = millis() + write_delay;
  // }
  // if ( data_available ) {
  //   can0.readMessage(&canMsgRx);
  //   Serial.print("Received message number ");
  //   Serial.print( counterRx++ );
  //   Serial.print(" from node ");
  //   Serial.print( canMsgRx.can_id , HEX);
  //   Serial.print(" : ");
  //   for (int i = 0 ; i < canMsgRx.can_dlc ; i++)
  //     Serial.print((char) canMsgRx.data[ i ]);
  //   Serial.println(" ");
  //   data_available = false;
  // }
  // end = micros();

  elapsed = millis();
  numSamples++;
  int read_adc;
  int adc_averaged = 0;
  float u;

  // temporaries to read from commands
  int state = 0;

  // averaging filter
  int values[avSize];
  for (int meas = 0; meas < avSize; meas++) {
    read_adc = analogRead(A0);  // read analog voltage
    values[meas] = read_adc;
  }
  std::sort(values, values + (sizeof(values) / sizeof(values[0])));
  adc_averaged = avSize % 2 == 0 ? (values[(avSize / 2) - 1] + values[(avSize / 2)]) / 2 : values[(avSize) / 2];
  lux = toLux(adc_averaged);

  while (Serial.available() > 0) {
    char input[13];
    char *command[4];
    String read = Serial.readStringUntil('\n');
    strcpy(input, read.c_str());
    char *token = strtok(input, " ");
    int numTokens = 0;
    while (token != NULL) {
      command[numTokens] = token;
      numTokens++;
      token = strtok(NULL, " ");
    }

    // see what command i am supposed to do
    if (strcmp(command[0], "d") == 0) {  // d <i> <val>
      controlled = 0;
      pwm = atof(command[2]);
      // check if it a valid pwm value
      if (0 <= pwm && pwm <= 1) {
        analogWrite(LED_PIN, (int)(pwm * DAC_RANGE));
        Serial.println("ack");
      } else {
        Serial.println("The duty cycle has to be between 0 and 1.");
        Serial.println("err");
      }
    } else if (strcmp(command[0], "reset") == 0) { // reset <i>
      desk = atoi(command[1]);
      occupancy[desk] = 0;
      antiWindup[desk] = 0;
      feedback[desk] = 0;
      ref = 0;
    } else if (strcmp(command[0], "g") == 0) {
      if (strcmp(command[1], "d") == 0) {  // g d <i>
        Serial.printf("d %s %.2f \n", command[2], pwm);
      } else if (strcmp(command[1], "r") == 0) {  // g r <i>
        Serial.printf("r %s %.2f \n", command[2], ref);
      } else if (strcmp(command[1], "l") == 0) {  // g l <i>
        Serial.print("Measure the illuminance of luminaire ");
        Serial.println(command[2]);
      } else if (strcmp(command[1], "o") == 0) {  // g o <i>
        Serial.printf("o %s %d \n", command[2], occupancy[atoi(command[2])]);
      } else if (strcmp(command[1], "a") == 0) {  // g a <i>
        Serial.printf("a %s %d \n", command[2], antiWindup[atoi(command[2])]);
      } else if (strcmp(command[1], "k") == 0) {  // g k <i>
        Serial.printf("k %s %d \n", command[2], feedback[atoi(command[2])]);
      } else if (strcmp(command[1], "x") == 0) {  // g x <i>
        Serial.print("Get current external illuminance of desk ");
        Serial.println(command[2]);
      } else if (strcmp(command[1], "p") == 0) {  // g p <i>
        Serial.print("Get instantaneous power consumption of desk ");
        Serial.println(command[2]);
      } else if (strcmp(command[1], "t") == 0) {  // g t <i>
        Serial.printf("t %s %lu \n", command[2], elapsed - restart);
      } else if (strcmp(command[1], "e") == 0) {  // g e <i>
        Serial.printf("e %s %.2f \n", command[2], energyCons);
      } else if (strcmp(command[1], "v") == 0) {  // g v <i>
        Serial.printf("v %s %.2f \n", command[2], visibilityError / numSamples);
      } else if (strcmp(command[1], "f") == 0) {  // g f <i>
        Serial.printf("f %s %.5f \n", command[2], flickering / numSamples);
      } else if (strcmp(command[1], "b") == 0 && numTokens == 4) {  // g b <x> <i>
        Serial.printf("b %s %d ", command[2], atoi(command[3]));
        if (strcmp(command[2], "l") == 0) {
          printMinute(luxQ);
        } else if (strcmp(command[2], "d") == 0) {
          printMinute(pwmQ);
        }
        Serial.printf("\nSize of queue: %d \n", luxQ.size());
      }
    } else if (strcmp(command[0], "r") == 0) {  // r <i> <val>
      ref = atoi(command[2]);
      controlled = 1;
      // check if it a valid lux value
      if (0 <= ref && ref < 30) {
        Serial.println("ack");
      } else {
        Serial.println("The refrence is in LUX.");
        Serial.println("err");
      }
    } else if (strcmp(command[0], "o") == 0) {  // o <i> <val>
      desk = atoi(command[1]);
      state = atoi(command[2]);
      if (state == 0 || state == 1) {
        occupancy[desk] = state;
        Serial.println("ack");
      } else {
        Serial.println("Occupancy state has to be 0 or 1.");
        Serial.println("err");
      }
    } else if (strcmp(command[0], "a") == 0) {  // a <i> <val>
      controlled = 1;
      desk = atoi(command[1]);
      state = atoi(command[2]);
      if (state == 0 || state == 1) {
        antiWindup[desk] = state;
        Serial.println("ack");
      } else {
        Serial.println("Anti-windup state has to be 0 or 1.");
        Serial.println("err");
      }
    } else if (strcmp(command[0], "k") == 0) {  // k <i> <val>
      controlled = 1;
      desk = atoi(command[1]);
      state = atoi(command[2]);
      if (state == 0 || state == 1) {
        feedback[desk] = state;
        Serial.println("ack");
      } else {
        Serial.println("Feedback state has to be 0 or 1.");
        Serial.println("err");
      }
    } else if (strcmp(command[0], "s") == 0) {  // s <x> <i>
      if (strcmp(command[1], "l") == 0 || strcmp(command[1], "d") == 0) {
        stream = 1;
        strcpy(variable, command[1]);
        desk_stream = atoi(command[2]);
      } else {
        Serial.println("Variable can be 'l' or 'd'.");
      }
    } else if (strcmp(command[0], "S") == 0) {  // S <x> <i>
      if (strcmp(command[1], variable) == 0 || desk_stream == atoi(command[1])) {
        stream = 0;
        strcpy(variable, "-");
        Serial.println("ack");
      } else {
        Serial.println("Stream referred not running.");
      }
    }
  }
  delay(10);

  if (controlled == 1) {
    // check which controller to use
    if (feedback[desk] == 1) {
      if (antiWindup[desk_stream] == 1) {

        u = pidComplex.computeControl(ref, lux);
        analogWrite(LED_PIN, (int)u);
        pwm = u / (float)DAC_RANGE;
        pidComplex.housekeep(ref, lux);

      } else if (antiWindup[desk_stream] == 0) {
        u = pidSimple.computeControl(ref, lux);
        analogWrite(LED_PIN, (int)u);
        pwm = u / (float)DAC_RANGE;
        pidSimple.housekeep(ref, lux);
      }
    } else if (feedback[desk_stream] == 0) {
      u = gain[1] * ref;
      analogWrite(LED_PIN, (int)u);
      pwm = u / (float)DAC_RANGE;
    }
  }
  // last minute buffers
  if (elapsed > 60000) {
    luxQ.pop();
    pwmQ.pop();
  }
  luxQ.push(lux);
  pwmQ.push(pwm);

  // metrics stuff
  energyCons += P_MAX * pwm * (float)(elapsed - prevTime) * 0.001f;
  visibilityError += std::max((float)0, ref - lux);
  if ((pwm - prevDuty) * (prevDuty - prevPrevDuty) < 0) {
    flickering += (abs(pwm - prevDuty) + abs(prevDuty - prevPrevDuty));
  }

  // keep previous values
  prevTime = elapsed;
  prevPrevDuty = prevDuty;
  prevDuty = pwm;

  // stream of real time variable
  if (stream == 1) {
    Serial.printf("s %s %d ", variable, desk_stream);
    if (strcmp(variable, "l") == 0) {
      Serial.print(lux);
      Serial.print(" ");
      Serial.print(elapsed);
      Serial.print(" ");
      Serial.println(ref);
    } else if (strcmp(variable, "d") == 0) {
      Serial.print(pwm);
      Serial.print(" ");
      Serial.print(elapsed);
      Serial.print(" ");
      Serial.println(ref);
    }
  }

}
