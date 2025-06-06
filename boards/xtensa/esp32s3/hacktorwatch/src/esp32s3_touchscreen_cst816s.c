#include <nuttx/config.h>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <syslog.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <syslog.h>
#include <debug.h>
#include <stdio.h>

#include <errno.h>
#include <nuttx/fs/fs.h>
#include <nuttx/himem/himem.h>
#include <arch/board/board.h>

#include "esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"

#ifdef CONFIG_ESP32S3_I2C
#  include "esp32s3_i2c.h"
#endif

#ifdef CONFIG_INPUT_CST816S
#define CST816S_DEVICE_ADDRESS 0x15
#define CST816S_INT_PIN 1
#  include <nuttx/input/cst816s.h>
#endif

#include "hacktorwatch.h"

int board_touchscreen_init(void) 
{
    /* Register the CST816S touch driver */

    struct i2c_master_s *cst816s_i2c_bus = esp32s3_i2cbus_initialize(0);
    if (!cst816s_i2c_bus)
    {
        _err("ERROR: Failed to get I2C%d interface\n", 0);
    }

    int ret = cst816s_register("/dev/input0", cst816s_i2c_bus, CST816S_DEVICE_ADDRESS);
    return ret;
}
