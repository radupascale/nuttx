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

#ifdef LRA_ACTUATOR
  uint8_t sample_time;
  uint8_t blanking_time;
  uint8_t idiss_time;
  uint8_t zc_det_time;
#endif

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

static int drv2605l_haptic_upload_effect(FAR struct ff_lowerhalf_s *lower,
                                         FAR struct ff_effect *effect,
                                         FAR struct ff_effect *old)
{
  iinfo("called: effect_id = %d \n", effect->id);
  return OK;
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

static void drv2605l_haptic_set_gain(FAR struct ff_lowerhalf_s *lower,
                                      uint16_t gain)
{
  iinfo("called: gain = %d\n", gain);
}

static int drv2605l_pin_init(FAR struct drv2605l_dev_s *priv)
{
  int ret = 0;

  ret = IOEXP_SETDIRECTION(priv->ioedev, priv->en_pin, IOEXPANDER_DIRECTION_OUT);
  if (ret < 0)
    {
      ierr("ioexpander set direction error: %d\n", ret);
      return -EIO;
    }

  ret = IOEXP_SETDIRECTION(priv->ioedev, priv->int_pin, IOEXPANDER_DIRECTION_OUT);
  if (ret < 0)
    {
      ierr("ioexpander set direction error: %d\n", ret);
      return -EIO;
    }
  
  return ret;
}

static int drv2605l_enable(FAR struct drv2605l_dev_s *priv, bool en)
{
  iinfo("enabling drv2605l: %d\n", en);
  int ret = 0;

  priv->enabled = en;

  /* Set / Unset enable pin */

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
      ierr("failed to allocate drv2605l_calib_s instance\n");
      return -ENOMEM;
    }

  priv->calib = calib_data;

  /* Place device in auto calibration mode */


  /* Populate input to auto calibration */


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
