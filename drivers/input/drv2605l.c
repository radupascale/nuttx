/****************************************************************************
 * drivers/input/drv2605l.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/kmalloc.h>
#include <nuttx/input/ff.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/gpio.h>

#include <nuttx/config.h>
#include <nuttx/nuttx.h>

#include <stdio.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>

#include <nuttx/i2c/i2c_master.h>

#if defined(CONFIG_I2C) && defined(CONFIG_FF_DRV2605L)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define DRV2605L_ADDR (0x5A)  /* I2C Slave Address */
#define DRV2605L_FREQ CONFIG_DRV2605L_I2C_FREQUENCY
#define DRV2605L_DEVID (0x7)

/* Register addresses */

#define DRV2605L_STATUS_REG_ADDR (0x0)  /* Device ID, Diag Result, Over Temp, OC Detect */
#define DRV2605L_MODE_REG_ADDR (0x01)   /* Device Reset, Standby, Mode */
#define DRV2605L_RTP_INPUT_REG_ADDR (0x02)

/* Library and waveform-related registers */

#define DRV2605L_LIB_SEL_REG_ADDR (0x03)
#define DRV2605L_WAV_FRM_SEQ1_REG_ADDR (0x04)
#define DRV2605L_WAV_FRM_SEQ2_REG_ADDR (0x05)
#define DRV2605L_WAV_FRM_SEQ3_REG_ADDR (0x06)
#define DRV2605L_WAV_FRM_SEQ4_REG_ADDR (0x07)
#define DRV2605L_WAV_FRM_SEQ5_REG_ADDR (0x08)
#define DRV2605L_WAV_FRM_SEQ6_REG_ADDR (0x09)
#define DRV2605L_WAV_FRM_SEQ7_REG_ADDR (0x0A)
#define DRV2605L_WAV_FRM_SEQ8_REG_ADDR (0x0B)

#define DRV2605L_GO_REG_ADDR (0x0C)   /* Bit 1 is GO bit */

#define DRV2605L_ODT_REG_ADDR (0x0D)  /* Overdrive Time Offset */
#define DRV2605L_SPT_REG_ADDR (0x0E)  /* Sustain Time Offset, Positive Reg*/
#define DRV2605L_SND_REG_ADDR (0x0F)  /* Sustain Time Offset, Negative Reg*/
#define DRV2605L_BRT_REG_ADDR (0x10)  /* Brake Time Offset */

/* Audio-To-Vibe registers*/

#define DRV2605L_ATH_CTRL_REG_ADDR (0x11) /* Control register */
#define DRV2605L_ATH_MIN_INPUT_REG_ADDR (0x12)
#define DRV2605L_ATH_MAX_INPUT_REG_ADDR (0x13)
#define DRV2605L_ATH_MIN_DRIVE_REG_ADDR (0x14)
#define DRV2605L_ATH_MAX_DRIVE_REG_ADDR (0x15)

/* Voltage-related registers */

#define DRV2605L_RATED_VOLTAGE_REG_ADDR (0x16)
#define DRV2605L_OD_CLAMP_VOLTAGE_REG_ADDR (0x17)
#define DRV2605L_VBAT_REG_ADDR (0x21)

/* Auto-Calibration registers */

#define DRV2605L_A_CAL_COMP_REG_ADDR (0x18)
#define DRV2605L_A_CAL_BEMF_REG_ADDR (0x19)

#define DRV2605L_FEEDBACK_CTRL_REG_ADDR (0x1A)
#define DRV2605L_CTRL1_REG_ADDR (0x1B)
#define DRV2605L_CTRL2_REG_ADDR (0x1C)
#define DRV2605L_CTRL3_REG_ADDR (0x1D)
#define DRV2605L_CTRL4_REG_ADDR (0x1E)
#define DRV2605L_CTRL5_REG_ADDR (0x1F)

#define DRV2605L_OL_LRA_PERIOD_REG_ADDR (0x20) /* Open-loop period */
#define DRV2605L_LRA_PERIOD_REG_ADDR (0x22)    /* Resonance period */

/* Bit masks */

#define DRV2605L_DEVID_MASK_MSK (0xE0)
#define DRV2605L_DIAG_RESULT_MSK (1 << 3)
#define DRV2605L_OVERTEMP_MSK (1 << 1)
#define DRV2605L_OC_DETECT_MSK (1)

#define DRV2605L_DEV_RESET_MSK (1 << 7)
#define DRV2605L_STANDBY_MSK (1 << 6)
#define DRV2605L_MODE_MSK (0x7)

#define DRV2605L_HIZ_MSK (1 << 4)
#define DRV2605L_LIB_SEL_MSK (0x7)

#define DRV2605L_WAIT_BIT_MSK (1 << 7)

#define DRV2605L_N_ERM_LRA_MSK (1 << 7)
#define DRV2605L_FB_BRAKE_FACTOR_MSK (0x70)
#define DRV2605L_LOOP_GAIN_MSK (0xC)
#define DRV2605L_BEMF_GAIN_MSK (0x3)

#define DRV2605L_STARTUP_BOOST_MSK (1 << 7)
#define DRV2605L_AC_COUPLE_MSK (1 << 5)
#define DRV2605L_DRIVE_TIME_MSK (0x1F)

#define DRV2605L_BIDIR_INPUT_MSK (1 << 7)
#define DRV2605L_BRAKE_STABILIZER_MSK (1 << 6)
#define DRV2605L_SAMPLE_TIME_MSK ((1 << 4) | (1 <<< 5))
#define DRV2605L_BLANKING_TIME_MSK (0xC)
#define DRV2605L_IDISS_TIME_MSK (0x3)
#define DRV2605L_OL_LRA_PERIOD_MSK (0x7F)
#define DRV2605L_GO_BIT_MSK (1)

/* Result ids */

#define AUTO_CALIB_PASSED (0)
#define AUTO_CALIB_FAILED (1)

#define DIAG_PASSED AUTO_CALIB_PASSED
#define DIAG_FAILED AUTO_CALIB_FAILED /* Actuator missing/shorted/timed out */

#define OVER_TEMP_NORMAL (0)
#define OVER_TEMP_EXCEEDED (1)

#define NO_OVERCURRENT_DETECTED (0)
#define OVERCURRENT_DETECTED (1)

/* Standby bit */

#define DEVICE_READY (0)
#define DEVICE_STANDBY (1)

/* Modes */

#define MODE_INTERNAL_TRIGGER (0)
#define MODE_EXTERNAL_TRIGGER_EDGE (1)
#define MODE_EXTERNAL_TRIGGER_LEVEL (2)
#define MODE_PWM_ANALOG_INPUT (3)
#define MODE_AUDIO_TO_VIBE (4)
#define MODE_RTP (5)
#define MODE_DIAGNOSTICS (6)
#define MODE_AUTO_CALIB (7)

/* Control values */

#define FB_BRAKE_FACTOR_1X (0)
#define FB_BRAKE_FACTOR_2X (1)
#define FB_BRAKE_FACTOR_3X (2)
#define FB_BRAKE_FACTOR_4X (3)
#define FB_BRAKE_FACTOR_6X (4)
#define FB_BRAKE_FACTOR_8X (5)
#define FB_BRAKE_FACTOR_16X (6)
#define FB_BRAKE_FACTOR_DISABLED (7)

#define LOOP_GAIN_LOW (0)
#define LOOP_GAIN_MEDIUM (1)
#define LOOP_GAIN_HIGH (2)
#define LOOP_GAIN_VERY_HIGH (3)



/****************************************************************************
 * Private Types
 ****************************************************************************/

struct drv2605l_calib_s
{
  /* Input */

  uint8_t erm_lra;
  uint8_t fb_brake_factor;
  uint8_t loop_gain;
  uint8_t rated_voltage;
  uint8_t od_clamp;
  uint8_t auto_cal_time;
  uint8_t drive_time;

#ifdef CONFIG_DRV2605L_LRA_ACTUATOR
  uint8_t sample_time;
  uint8_t blanking_time;
  uint8_t idiss_time;
  uint8_t zc_det_time;
#endif /* CONFIG_DRV2605L_LRA_ACTUATOR */

  /* Output */

  uint8_t bemf_gain;
  uint8_t a_cal_comp;
  uint8_t a_cal_bemf;
  uint8_t diag_result;
};

struct drv2605l_dev_s
{
  bool enabled;                         /* Enable/Disable DRV2605L */
  struct ff_lowerhalf_s lower;
  FAR struct i2c_master_s *i2c;
  FAR struct ioexpander_dev_s *ioedev;
  FAR struct drv2605l_calib_s *calib;

  int en_pin;
  int int_pin;
#ifndef CONFIG_DRV2605L_NO_LIBRARY
  uint8_t lib_effect_duration; /* rise + brake time (ms) */
#endif /* !CONFIG_DRV2605L_NO_LIBRARY */

  uint16_t max_effects;
  mutex_t dev_lock;
  mutex_t rtp_lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/



/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv2605l_getregs
 *
 * Description:
 *   Read <length> bytes starting from a DRV2605L register addr
 *
 ****************************************************************************/

static int drv2605l_getregs(FAR struct drv2605l_dev_s *priv, uint8_t regaddr,
                            uint8_t *rxbuffer, uint8_t length)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = DRV2605L_FREQ;
  msg[0].addr = DRV2605L_ADDR;
  msg[0].flags = 0;
  msg[0].buffer = &regaddr;
  msg[0].length = 1;

  msg[1].frequency = DRV2605L_FREQ;
  msg[1].addr = DRV2605L_ADDR;
  msg[1].flags = I2C_M_READ;
  msg[1].buffer = rxbuffer;
  msg[1].length = length;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      ierr("I2C_TRANSFER failed: %d\n", ret);
      return -1;
    }

  return OK;
}

/****************************************************************************
 * Name: drv2605l_getreg8
 *
 * Description:
 *   Read from an 8-bit DRV2605L register
 *
 ****************************************************************************/

static uint8_t drv2605l_getreg8(FAR struct drv2605l_dev_s *priv, uint8_t regaddr)
{
  struct i2c_msg_s msg[2];
  uint8_t regval = 0;
  int ret;

  msg[0].frequency = DRV2605L_FREQ;
  msg[0].addr = DRV2605L_ADDR;
  msg[0].flags = 0;
  msg[0].buffer = &regaddr;
  msg[0].length = 1;

  msg[1].frequency = DRV2605L_FREQ;
  msg[1].addr = DRV2605L_ADDR;
  msg[1].flags = I2C_M_READ;
  msg[1].buffer = &regval;
  msg[1].length = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      ierr("I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

  return regval;
}

/****************************************************************************
 * Name: drv2605l_putreg8
 *
 * Description:
 *   Write to an 8-bit DRV2605L register
 *
 ****************************************************************************/

static int drv2605l_putreg8(FAR struct drv2605l_dev_s *priv, uint8_t regaddr,
                            uint8_t regval)
{
  struct i2c_msg_s msg[2];
  uint8_t txbuffer[2];
  int ret;

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = DRV2605L_FREQ;
  msg[0].addr = DRV2605L_ADDR;
  msg[0].flags = 0;
  msg[0].buffer = txbuffer;
  msg[0].length = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      ierr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: drv2605l_checkid
 *
 * Description:
 *   Read and verify the DRV2605L chip ID
 *
 ****************************************************************************/

static int drv2605l_checkid(FAR struct drv2605l_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read device ID */

  devid = drv2605l_getreg8(priv, DRV2605L_STATUS_REG_ADDR);
  up_mdelay(1);
  devid >>= 5;

  iinfo("devid: 0x%02x\n", devid);

  if (devid != (uint8_t)DRV2605L_DEVID)
    {
      /* ID is not Correct */

      ierr("Wrong Device ID! %02x\n", devid);
      return -ENODEV;
    }

  return OK;
}

static void drv2605l_save_rtp_effect(FAR struct drv2605l_dev_s *priv,
                                     FAR struct ff_effect *effect)
{
  struct drv2605l_effect_s *rtp_effect;
  int16_t level = effect->u.constant.level;
  uint8_t regval = 0;
  float percent = 0;

  rtp_effect = &priv->effects[effect->id];
  rtp_effect->type = DRV2605L_RTP_EFFECT;
  rtp_effect->delay = effect->replay.delay;
  rtp_effect->length = effect->replay.length;

#ifdef CONFIG_DRV2605L_OPEN_LOOP_MODE
  /* In Open Loop, negative drive values are allowed
   * (meaning counter-clockwise rotation for ERM,
   *   or 180-degree phase shift for LRA)
   * So, negative magnitudes map to negative drive [-100%, 0%]
   * and positive magnitudes map to positive drive [0%, 100%]
   */

  percent = level * 1.f / INT16_MAX;
  percent *= 0.5;
  percent += 0.5;
#endif /* CONFIG_DRV2605L_OPEN_LOOP_MODE */

#ifdef CONFIG_DRV2605L_CLOSED_LOOP_UNIDIR_MODE
  /* In Closed Loop Unidir Mode, negative drive values
   * are not allowed. 0% drive would mean Full Braking,
   * and is not recommended. 50% means 1/2 Rated Voltage,
   * 100% is Rated Voltage magnitude.
   * So, the absolute value of the magnitude is used.
   */

  percent = ABS(level) * 1.f / INT16_MAX;
  percent = MAX(percent, 0.1); /* Ensure at least 10% to avoid Full Braking */
#endif /* CONFIG_DRV2605L_CLOSED_LOOP_UNIDIR_MODE */

#ifdef CONFIG_DRV2605L_CLOSED_LOOP_BIDIR_MODE
  /* In Closed Loop Bidir Mode, negative drive values
   * are not allowed. [0%, 50%) drive would mean Full Braking,
   * 75% means 1/2 Rated Voltage,
   * 100% is Rated Voltage magnitude.
   * So, the absolute value of the magnitude is used and the
   * percent is scaled to fit in [50%, 100%]
   */

  percent = ABS(level) * 1.f / INT16_MAX;
  percent *= 0.5;
  percent += 0.5;
#endif /* CONFIG_DRV2605L_CLOSED_LOOP_BIDIR_MODE */

  iinfo("percent: %f\n", percent);

  /* Scale level to fit in 8-bits and consider RTP_DATA_FORMAT */

  regval = drv2605l_getreg8(priv, DRV2605L_CTRL3_REG_ADDR);

  if (regval & DRV2605L_DATA_FORMAT_RTP_MSK)
    {
      rtp_effect->v.rtp_data = (0xff * percent);
      iinfo("RTP Unsigned value: 0x%x\n", rtp_effect->v.rtp_data);
    }
  else
    {
      rtp_effect->v.rtp_data = (0xff * percent) - 0x80;
      iinfo("RTP Signed value: 0x%x\n", rtp_effect->v.rtp_data);
    }
}

static void drv2605l_save_rom_effect(FAR struct drv2605l_dev_s *priv,
                                     FAR struct ff_effect *effect)
{
  struct drv2605l_effect_s *rom_effect;
  uint8_t data_len = MIN(effect->u.periodic.custom_len, 16);
  uint16_t data = 0;
  uint8_t i = 0;

  rom_effect = &priv->effects[effect->id];
  rom_effect->type = DRV2605L_ROM_EFFECT;
  rom_effect->delay = effect->replay.delay;

  /**
   *  Expected custom_data field:
   *  [effect_id, wait_time,  ...., wait_time, effect_id, wait_time]
   *  with a maximum of 8 non-zero fields
   *  (wait_time = 0 => not written to any registers)
   *  Maximum custom_len is 16 (8 pairs of wait time and id)
   */

  for (i = 0; i < data_len; i++)
    {
      data = effect->u.periodic.custom_data[i];

      if (data)
        {
          if (i & 1)
            {
              /* Convert ms to register data:
               * WAV_FRM_SEQ[6:0] * 10ms = wait time
               */

              rom_effect->v.wav_frm_seq[i] = data / 10;
              rom_effect->v.wav_frm_seq[i] |= (1 << 7);
              rom_effect->length += data;
            }
          else
            {
              /* Library effect index: [1, 123] */

              rom_effect->v.wav_frm_seq[i] = MIN(data, 123);
              rom_effect->length += priv->lib_effect_duration;
            }
        }
    }

  /* To stop the waveform playback */

  if (data_len < 8)
    {
      rom_effect->v.wav_frm_seq[data_len] = 0;
    }
}

static int drv2605l_haptic_upload_effect(FAR struct ff_lowerhalf_s *lower,
                                         FAR struct ff_effect *effect,
                                         FAR struct ff_effect *old)
{
  FAR struct drv2605l_dev_s *priv;

  iinfo("effect_id = %d \n", effect->id);

  priv = container_of(lower, FAR struct drv2605l_dev_s, lower);

  if (effect->type == FF_CONSTANT)
    {
      drv2605l_save_rtp_effect(priv, effect);
      return OK;
    }

  if (effect->type == FF_PERIODIC)
    {
      if (effect->u.periodic.waveform == FF_CUSTOM)
        {
#ifndef CONFIG_DRV2605L_NO_LIBRARY
          drv2605l_save_rom_effect(priv, effect);
#else
          ierr("Cannot use FF_CUSTOM when no library is selected\n");
#endif /* !CONFIG_DRV2605L_NO_LIBRARY */
        }
    }
}

static int drv2605l_haptic_playback(struct ff_lowerhalf_s *lower,
                                     int effect_id, int val)
{
  iinfo("called: effect_id = %d val = %d\n", effect_id, val);
  return OK;
}

static int drv2605l_haptic_erase(FAR struct ff_lowerhalf_s *lower,
                                  int effect_id)
{
  iinfo("called: effect_id = %d\n", effect_id);
  return OK;
}

static float drv2605l_get_vbat(FAR struct drv2605l_dev_s *priv)
{
  iinfo("called: gain = %d\n", gain);
}

static void drv2605l_standby_work_routine(FAR void *arg)
{
  uint8_t regval = 0;
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)arg;

  /* In case of waveform playing, check the GO bit */

  regval = drv2605l_getreg8(priv, DRV2605L_GO_REG_ADDR);

  if (regval)
    {
      wd_start(&priv->wd_timer, MSEC2TICK(150),
            drv2605l_timer_func, (wdparm_t)priv);
      return;
    }

  /* Enter standby */

  drv2605l_putreg8(priv, DRV2605L_MODE_REG_ADDR, DEVICE_STANDBY);
  nxmutex_unlock(&priv->dev_lock);
}

static void drv2605l_timer_func(wdparm_t arg)
{
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)arg;

  work_queue(HPWORK, &priv->haptic_work,
             drv2605l_standby_work_routine, priv, 0);
}

static void drv2605l_rom_work_routine(FAR void *arg)
{
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)arg;
  FAR struct drv2605l_effect_s *effect = priv->current_effect;
  uint8_t i = 0;

  for (i = 0; i < 8; i++)
    {
      drv2605l_putreg8(priv, DRV2605L_WAV_FRM_SEQ_BASE_REG_ADDR + i,
                      effect->v.wav_frm_seq[i]);

      if (!effect->v.wav_frm_seq[i])
        {
          break;
        }
    }

  /* Play effects: MODE = Internal Trigger + Remove from standby */

  drv2605l_putreg8(priv, DRV2605L_MODE_REG_ADDR, DEVICE_READY);
  drv2605l_putreg8(priv, DRV2605L_GO_REG_ADDR, 0x01);

  /* Check periodically on the GO bit and then put the driver in standby */

  wd_start(&priv->wd_timer, MSEC2TICK(effect->length),
          drv2605l_timer_func, (wdparm_t)priv);
}

static void drv2605l_rtp_work_routine(FAR void *arg)
{
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)arg;
  FAR struct drv2605l_effect_s *effect = priv->current_effect;

  drv2605l_putreg8(priv, DRV2605L_RTP_INPUT_REG_ADDR,
                   effect->v.rtp_data);

  drv2605l_putreg8(priv, DRV2605L_MODE_REG_ADDR, MODE_RTP);

  wd_start(&priv->wd_timer, MSEC2TICK(effect->length),
          drv2605l_timer_func, (wdparm_t)priv);
}

static void drv2605l_play_current_effect(FAR struct drv2605l_dev_s *priv)
{
  FAR struct drv2605l_effect_s *effect;
  sclock_t time_us;

  time_us = wd_gettime(&priv->wd_timer);
  usleep(time_us);

  if (!priv->current_effect)
    {
      ierr("No effect selected!\n");
      return;
    }

  nxmutex_lock(&priv->dev_lock);
  effect = priv->current_effect;

  if (effect->type == DRV2605L_RTP_EFFECT)
    {
      work_queue(HPWORK, &priv->haptic_work,
                  drv2605l_rtp_work_routine, priv,
                  MSEC2TICK(effect->delay));
    }

  if (effect->type == DRV2605L_ROM_EFFECT)
    {
      work_queue(HPWORK, &priv->haptic_work,
                  drv2605l_rom_work_routine, priv,
                  MSEC2TICK(effect->delay));
    }
}

static int drv2605l_haptic_playback(struct ff_lowerhalf_s *lower,
                                    int effect_id, int val)
{
  FAR struct drv2605l_dev_s *priv;

  iinfo("effect_id = %d val = %d\n", effect_id, val);

  priv = container_of(lower, FAR struct drv2605l_dev_s, lower);
  priv->current_effect = &priv->effects[effect_id];

  if (val == FF_EVENT_START)
    {
      drv2605l_play_current_effect(priv);
      return OK;
    }

  if (val == FF_EVENT_STOP)
    {
      nxmutex_unlock(&priv->dev_lock);
      work_cancel(HPWORK, &priv->haptic_work);
      return OK;
    }

  ierr("Unsupported event value!\n");
  return OK;
}

static int drv2605l_set_loop_mode(FAR struct drv2605l_dev_s *priv)
{
  int ret = 0;

  ret = IOEXP_SETDIRECTION(priv->ioedev, priv->en_pin, IOEXPANDER_DIRECTION_OUT);
  if (ret < 0)
    {
      ierr("ioexpander set direction error: %d\n", ret);
      return -EIO;
    }

#ifdef CONFIG_DRV2605L_OPEN_LOOP_MODE
#ifdef CONFIG_DRV2605L_LRA_ACTUATOR
  regval |= 1;
#else
  regval |= (1 << 5);
#endif /* LRA_ACTUATOR */
#else
#ifdef CONFIG_DRV2605L_LRA_ACTUATOR
  regval &= ~1;
#else
  regval &= ~(1 << 5);
#endif /* LRA_ACTUATOR */
#endif /* OPEN_LOOP_MODE */

  ret = drv2605l_putreg8(priv, DRV2605L_CTRL3_REG_ADDR, regval);

  regval = drv2605l_getreg8(priv, DRV2605L_CTRL2_REG_ADDR);

#ifdef CONFIG_DRV2605L_CLOSED_LOOP_UNIDIR_MODE
  regval &= ~(1 << 7);
#else
  regval |= (1 << 7);
#endif /* CONFIG_DRV2605L_CLOSED_LOOP_UNIDIR_MODE */

  ret = drv2605l_putreg8(priv, DRV2605L_CTRL2_REG_ADDR, regval);

#ifdef CONFIG_DRV2605L_CLOSED_LOOP_UNIDIR_MODE
  /* Set DATA_FORMAT_RTP = 1 */

  regval = drv2605l_getreg8(priv, DRV2605L_CTRL3_REG_ADDR);
  regval |= DRV2605L_DATA_FORMAT_RTP_MSK;

  ret = drv2605l_putreg8(priv, DRV2605L_CTRL3_REG_ADDR, regval);
#endif /* CONFIG_DRV2605L_CLOSED_LOOP_UNIDIR_MODE */

  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

static int drv2605l_enable(FAR struct drv2605l_dev_s *priv, bool en)
{
  iinfo("enabling drv2605l: %d\n", en);
  int ret = 0;

  priv->enabled = en;

#ifdef CONFIG_DRV2605L_TS2200_LIBRARY_A
  regval |= 1;
  priv->lib_effect_duration = 100;
#endif /* CONFIG_DRV2605L_TS2200_LIBRARY_A */

#ifdef CONFIG_DRV2605L_TS2200_LIBRARY_B
  regval |= 2;
  priv->lib_effect_duration = 75;
#endif /* CONFIG_DRV2605L_TS2200_LIBRARY_B */

#ifdef CONFIG_DRV2605L_TS2200_LIBRARY_C
  regval |= 3;
  priv->lib_effect_duration = 100;
#endif /* CONFIG_DRV2605L_TS2200_LIBRARY_C */

#ifdef CONFIG_DRV2605L_TS2200_LIBRARY_D
  regval |= 4;
  priv->lib_effect_duration = 165;
#endif /* CONFIG_DRV2605L_TS2200_LIBRARY_D */

#ifdef CONFIG_DRV2605L_TS2200_LIBRARY_E
  regval |= 5;
  priv->lib_effect_duration = 180;
#endif /* CONFIG_DRV2605L_TS2200_LIBRARY_E */

#ifdef CONFIG_DRV2605L_TS2200_LIBRARY_F
  regval |= 7;
  priv->lib_effect_duration = 65;
#endif /* CONFIG_DRV2605L_TS2200_LIBRARY_F */

#ifdef CONFIG_DRV2605L_LRA_LIBRARY
  regval |= 6;
  priv->lib_effect_duration = 100;
#endif /* CONFIG_DRV2605L_LRA_LIBRARY */

  ret = drv2605l_putreg8(priv, DRV2605L_LIB_SEL_REG_ADDR, regval);

  nxmutex_unlock(&priv->dev_lock);

  return ret;
}

static int drv2605l_set_mode(FAR struct drv2605l_dev_s *priv, uint8_t mode)
{
  iinfo("setting drv2605l mode: %d\n", mode);
  int ret = 0;


  return ret;
}

static int drv2605l_auto_calib(FAR struct drv2605l_dev_s *priv)
{
  iinfo("performing auto calibration\n");

  int ret = 0;
  uint8_t regval = 0;
  FAR struct drv2605l_calib_s *calib_data;

  calib_data = kmm_malloc(sizeof(struct drv2605l_calib_s));
  if (calib_data == NULL)
    {
      /* Populate with default params */

      calib_data = kmm_zalloc(sizeof(struct drv2605l_calib_s));
      if (calib_data == NULL)
        {
          ierr("failed to allocate drv2605l_calib_s instance\n");
          return -ENOMEM;
        }

      calib_data->erm_lra = 0;

#ifdef CONFIG_DRV2605L_LRA_ACTUATOR
      calib_data->erm_lra = 1;
#endif /* CONFIG_DRV2605L_LRA_ACTUATOR */

      calib_data->fb_brake_factor = 3;
      calib_data->loop_gain = 1;
      calib_data->rated_voltage = 0x3e;
      calib_data->od_clamp = 0x8c;
      calib_data->auto_cal_time = 2;
      calib_data->drive_time = 0x13;

#ifdef CONFIG_DRV2605L_LRA_ACTUATOR
      calib_data->sample_time = 3;
      calib_data->blanking_time = 1;
      calib_data->idiss_time = 1;
      calib_data->zc_det_time = 0;
#endif /* CONFIG_DRV2605L_LRA_ACTUATOR */
    }

  priv->calib = calib_data;

  /* Place device in auto calibration mode */


  /* Populate input to auto calibration */

  regval = drv2605l_getreg8(priv, DRV2605L_FEEDBACK_CTRL_REG_ADDR);
  regval &= DRV2605L_BEMF_GAIN_MSK;
  regval |= (priv->calib->erm_lra << 7) |
            (priv->calib->fb_brake_factor << 4) |
            (priv->calib->loop_gain << 2);
  drv2605l_putreg8(priv, DRV2605L_FEEDBACK_CTRL_REG_ADDR, regval);
  drv2605l_putreg8(priv, DRV2605L_RATED_VOLTAGE_REG_ADDR,
                   priv->calib->rated_voltage);
  drv2605l_putreg8(priv, DRV2605L_OD_CLAMP_VOLTAGE_REG_ADDR,
                   priv->calib->od_clamp);

  regval = drv2605l_getreg8(priv, DRV2605L_CTRL1_REG_ADDR);
  regval |= (priv->calib->drive_time & DRV2605L_DRIVE_TIME_MSK);

  regval = drv2605l_getreg8(priv, DRV2605L_CTRL4_REG_ADDR);
  regval |= (priv->calib->auto_cal_time & 0x3) << 4;

#ifdef CONFIG_DRV2605L_LRA_ACTUATOR
  regval |= priv->calib->zc_det_time << 6;
#endif /* CONFIG_DRV2605L_LRA_ACTUATOR */

  drv2605l_putreg8(priv, DRV2605L_CTRL4_REG_ADDR, regval);

#ifdef CONFIG_DRV2605L_LRA_ACTUATOR
  regval = drv2605l_getreg8(priv, DRV2605L_CTRL2_REG_ADDR);

  regval |= (((priv->calib->sample_time & 0x3) << 4) |
            ((priv->calib->blanking_time & 0x3) << 2) |
            (priv->calib->idiss_time & 0x3)) & ~((1 << 6) | (1 << 7));

  drv2605l_putreg8(priv, DRV2605L_CTRL2_REG_ADDR, regval);

  regval = drv2605l_getreg8(priv, DRV2605L_CTRL5_REG_ADDR);
  regval &= ~(0xf);
  regval |= ((priv->calib->blanking_time & 0xc) |
            ((priv->calib->idiss_time >> 2) 0x3));

  drv2605l_putreg8(priv, DRV2605L_CTRL5_REG_ADDR, regval);
#endif /* CONFIG_DRV2605L_LRA_ACTUATOR */

  /* Trigger auto calibration */


  /* Store result if calibration was successful */


  return ret;
}

static int drv2605l_init(FAR struct drv2605l_dev_s *priv)
{
  int ret = 0;

  /* Init EN and IN/TRIG pins */

  drv2605l_pin_init(priv);

  usleep(250);

  ret = drv2605l_enable(priv, true);

  if (ret < 0)
    {
      return ret;
    }

  ret = drv2605l_checkid(priv);

  if (ret < 0)
    {
      return ret;
    }

  ret = drv2605l_auto_calib(priv);

  if (ret < 0)
    {
      ierr("auto calibration failed\n");
      return ret;
    }

  /* Select ROM library */

  /* Put device in desired mode (Open-Loop, Closed-Loop) */

  /* Standby / Deassert EN */

  return ret;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

int drv2605l_register(int devno, FAR struct i2c_master_s *i2c,
                      FAR struct ioexpander_dev_s *ioedev)
{
  FAR struct drv2605l_dev_s *priv;
  FAR struct ff_lowerhalf_s *lower;
  int ret = 0;

  DEBUGASSERT(i2c != NULL && ioedev != NULL);

  /* Initialize the drv2605l dev structure */

  priv = kmm_malloc(sizeof(struct drv2605l_dev_s));
  if (priv == NULL)
    {
      ierr("Failed to allocate drv2605l_dev_s instance\n");
      return -ENOMEM;
    }

  lower = kmm_malloc(sizeof(struct ff_lowerhalf_s));
  if (lower == NULL)
    {
      ierr("Failed to allocate ff_lowerhalf_s instance\n");
      kmm_free(priv);
      return -ENOMEM;
    }

  /* Init upper */

  priv->max_effects = 256; // TODO
  priv->i2c = i2c;
  priv->ioedev = ioedev;
  priv->en_pin = CONFIG_DRV2605L_EN_PIN;
  priv->int_pin = CONFIG_DRV2605L_IN_TRIG_PIN;

  // nxmutex_init(&priv->dev_lock);

  /* Init lowerhalf */

  lower                  = &priv->lower;
  lower->upload          = drv2605l_haptic_upload_effect;
  lower->erase           = drv2605l_haptic_erase;
  lower->playback        = drv2605l_haptic_playback;
  lower->set_gain        = drv2605l_haptic_set_gain;
  lower->set_autocenter  = NULL;
  lower->destroy         = NULL;

  /* Set up capabilities */

  set_bit(FF_CUSTOM, lower->ffbit);
  set_bit(FF_GAIN, lower->ffbit);
  set_bit(FF_CONSTANT, lower->ffbit);
  set_bit(FF_PERIODIC, lower->ffbit);

  /* Init device */

  ret = drv2605l_init(priv);

  if (ret < 0)
    {
      goto err;
    }

  /* Register driver */

  char *devpath = "/dev/input_ff0";

  ret = ff_register(lower, devpath, priv->max_effects);
  if (ret < 0)
    {
      ierr("Failed to register driver: %d\n", ret);
      goto err;
    }

  iinfo("drv2605l registered at %s\n", devpath);
  return OK;

err:
  /* Free memory, unregister stuff */

  kmm_free(priv);
  kmm_free(lower);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_FF_DRV2605L */
