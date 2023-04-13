/**************************************************************************/ /**
   \file     encoder.h
   \brief    this is the header file of encoder.c.
   \author   Lao·Zhu
   \version  V1.0.1
   \date     10. October 2021
  ******************************************************************************/

#ifndef MINIFOC_ALGORITHM_ENCODER_H_
#define MINIFOC_ALGORITHM_ENCODER_H_

extern int total_machine_angle_e;
extern unsigned short angle_extern;
extern unsigned short angle_extern1;

inline unsigned short positive_mod(long long number, unsigned short mode) {
  short temp = number % mode;
  if (temp > 0) {
    return temp;
  } else {
    return temp + mode;
  }
}

inline int my_abs_int(int num) {
  if (num < 0) {
    return -num;
  } 
  return num;
}

extern volatile unsigned short machine_angle_offset;
void encoder_zeroing(void);
unsigned short encoder_get_mechanical_angle(void);
float encoder_get_electronic_angle(void);
void encoder_update_speed(void);
unsigned short encoder_read_data(unsigned short TxData);
#endif // MINIFOC_ALGORITHM_ENCODER_H_
