#ifndef _PID_HEAD_H_
#define _PID_HEAD_H_

#define ARDUINO_DEBUG 0

typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float error;
  int sampleTime;
  float previous_error;
} pid_t;

#define SAMPLE_TIME 10
#define KP_VALUE  30
#define KI_VALUE  0.03
#define KD_VALUE  13


#endif



