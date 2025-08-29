/*! ----------------------------------------------------------------------------
 * @file    deca_sleep.c
 * @brief   platform dependent sleep implementation
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include <deca_device_api.h>
#include <delay.h>

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_sleep(unsigned int time_ms)
{
  Delay_ms(time_ms);
}


/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_usleep(unsigned long time_us)
{
  Delay_us(time_us);
}
