/**************************************************************************/ /**
   \file     encoder.c
   \brief    this file contains the code implementation of angle acquisition
             and initialization functions of sc60228 and sc60224.
   \author   Lao·Zhu
   \version  V1.0.2
   \date     29. October 2021
  ******************************************************************************/

#include "encoder.h"

#include "config.h"
#include "fast_math.h"
#include "filter.h"
#include "foc.h"
#include "gd32f1x0.h"
#include "spi.h"
#include "system.h"
#include "timer.h"

unsigned short angle_extern;
unsigned short angle_extern1;
int total_machine_angle_e;

extern char send_buf[100];

/*!
    \brief mechanical angle offset, which is used to align the mechanical
           angle with the zero point of the electrical angle.
*/
volatile unsigned short machine_angle_offset = 0;
/*!
    \brief the mechanical angle at the last moment is used to calculate
           the angle differential value and realize the angle measurement
           function.
*/
volatile static unsigned short last_mechanical_angle = 0;
volatile static float angle_prev = 0;
volatile static float vel_angle_prev = 0;
/*!
    \brief the total mechanical angle since power on is used to calculate
           the angle integral value and realize the angle measurement function.
*/
volatile static long long total_machine_angle = 0;
volatile static long long full_rotations = 0;
volatile static long long vel_full_rotations = 0;
/*!
    \brief the mechanical angle at the last moment is used to calculate
           the rotation speed of the motor rotor.
*/
volatile static long long systick_mechanical_angle_last = 0;

/*!
    \brief delay function for magnetic encoder
*/
void encoder_delay(void) {
  /* use loop functions to delay time */
  unsigned char delay_counter = 0xff;
  while (delay_counter) delay_counter--;
}

/*!
    \brief     read data from the register of the magnetic encoder
    \param[in] TxData: data to be sent to magnetic encoder
    \retval    data read from magnetic encoder
*/
unsigned short encoder_read_data(unsigned short TxData) {
  static unsigned short last_data = 0;
  unsigned short data;

  /* pull down the CS pin and prepare to send data */
  gpio_bit_write(GPIOA, GPIO_PIN_4, RESET);
  encoder_delay();

  /* call SPI related functions to send and receive data */
  data = spi_readwrite_halfworld(TxData);
  encoder_delay();

  /* pull up CS pin to end sending data */
  gpio_bit_write(GPIOA, GPIO_PIN_4, SET);

  if (data != 0) {
    last_data = data;
    return data;
  } else {
    return last_data;
  }
  // return data;
}

/*!
    \brief  read mechanical angle directly from encoder
    \retval register raw data reading back
*/
unsigned short encoder_get_mechanical_angle(void) {
  /* read back register raw data */
  unsigned short angle = encoder_read_data(0x0000) >> 4;
  angle_extern = angle;
  return positive_mod(angle - machine_angle_offset, ENCODER_RESO);
}

/*!
    \brief  according to the electrical angle calculated from the mechanical
   angle, this function will call encoder_get_mechanical_angle() function.
    \retval register raw data reading back
*/
float encoder_get_electronic_angle(void) {
  /* read back the mechanical angle directly from the magnetic encoder */
  unsigned short tmp_mechanical_angle = encoder_get_mechanical_angle();
  /* calculate and update the mechanical angle and electric angle */
  FOC_Struct.mechanical_angle =
      (float)tmp_mechanical_angle * MECHANGLE_COEFFICIENT;

  float d_angle = FOC_Struct.mechanical_angle - angle_prev;
  /* 这里判断是否转出一圈 */
  if (my_abs_int(d_angle) > (0.8f * M_2PI)) {
    full_rotations += (d_angle > 0) ? -1 : 1;
  }
  float electric_angle = FOC_Struct.mechanical_angle * POLAR_PAIRS;

  angle_prev = FOC_Struct.mechanical_angle;
  total_machine_angle_e = full_rotations;
  angle_extern1 = tmp_mechanical_angle;
  return electric_angle;
}

/*!
    \brief called every 2 milliseconds to calculate the speed.
*/
// void encoder_update_speed(void) {
//   /* calculate the difference between this angle and the last angle */
//   short tmp_mechanical_angle_velocity =
//       (short)(total_machine_angle - systick_mechanical_angle_last);

//   /* send it to low-pass filter for filtering to prevent PID high-frequency
//    * oscillation */
//   FOC_Struct.rotate_speed = filter_update_value(
//       (Filter_Structure_t *)&velocity_filter, tmp_mechanical_angle_velocity);
//   systick_mechanical_angle_last = total_machine_angle;
// }
void encoder_update_speed(void) {
  /* calculate the difference between this angle and the last angle */
  FOC_Struct.rotate_speed =
      ((float)(full_rotations - vel_full_rotations) * M_2PI +
       (angle_prev - vel_angle_prev)) *
      SPEED_UP_FREQ;
  vel_angle_prev = angle_prev;
  vel_full_rotations = full_rotations;
}

/*!
    \brief correct the mechanical angle zero deviation between the magnetic
   encoder and FOC.
*/

void encoder_zeroing(void) {
  static float u, v, w;
  
  /* delay to wait for the power supply voltage to be normal */
  delayms(2000);

  /* set that there is only a magnetic field on the straight axis. */
  foc_calculate_dutycycle(0, CALI_TORQUE, 0, &u, &v, &w);
  update_pwm_dutycycle(u, v, w);
  delayms(300);
  machine_angle_offset = 0;
  total_machine_angle = 0;

  /* read the angle at this time as the offset angle */
  machine_angle_offset = encoder_read_data(0x0000) >> 4;

  /* zero the torque in all directions to release the motor */
  foc_calculate_dutycycle(0, 0, 0, &u, &v, &w);
  update_pwm_dutycycle(u, v, w);
  delayms(300);
}

void t_encoder_zeroing(void) {
  float u, v, w;
  /* delay to wait for the power supply voltage to be normal */
  delayms(1000);

  /* set that there is only a magnetic field on the straight axis. */
  foc_calculate_dutycycle(0, 0.5, 0, &w, &v, &u);
  update_pwm_dutycycle(u, v, w);
  delayms(500);
  machine_angle_offset = 0;
  total_machine_angle = 0;

  /* read the angle at this time as the offset angle */
  machine_angle_offset = encoder_read_data(0x0000) >> 4;

  /* zero the torque in all directions to release the motor */
  foc_calculate_dutycycle(0, 0, 0, &w, &v, &u);
  update_pwm_dutycycle(u, v, w);
  delayms(300);
}
