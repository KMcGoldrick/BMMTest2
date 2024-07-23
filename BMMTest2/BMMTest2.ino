#include <bmm150_defs.h>
#include <bmm150.h>
#include <Wire.h>

int loopCnt;
int8_t rslt;
float x, y, z;
bool hasMag;
struct bmm150_dev bmm1_dev;
uint8_t dev_addr;

/************************** Internal macros *******************************/
/* Sensor ODR, Repetition and axes enable/disable settings */
#define MODE_SETTING_SEL                UINT16_C(0x000F)

/* Interrupt pin settings like polarity,latch and int_pin enable */
#define INTERRUPT_PIN_SETTING_SEL       UINT16_C(0x01F0)

/* Settings to enable/disable interrupts */
#define INTERRUPT_CONFIG_SEL            UINT16_C(0x1E00)

/* Interrupt settings for configuring threshold values */
#define INTERRUPT_THRESHOLD_CONFIG_SEL  UINT16_C(0x6000)

void serialData() {
    Serial.print(0);
    Serial.print(',');
    Serial.print(loopCnt);
    Serial.print(',');
    Serial.print(x);
    Serial.print(',');
    Serial.print(y);
    Serial.print(',');
    Serial.println(z);
}

uint8_t my_burst_read(uint8_t reg_addr, uint8_t* reg_data, int length) {
    // Start a transmission to the I2C device
    Serial.print("     my_burst_read  ");Serial.print(dev_addr, HEX);Serial.print(reg_addr, HEX);
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);  // Send the register address
    Wire.endTransmission(false);  // End the transmission without releasing the bus

    // Request the data from the I2C device
    Wire.requestFrom(dev_addr, length);

    // Read the data bytes
    for (uint8_t i = 0; i < length; i++) {
        if (Wire.available()) {
            reg_data[i] = Wire.read();
        }
    }

    // Print the read data
    Serial.print(" Data read: ");
    for (uint8_t i = 0; i < length; i++) {
        Serial.print(reg_data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

}

uint8_t my_burst_write(uint8_t reg_addr, const uint8_t* reg_data, int length) {
    // Perform the burst write
    Serial.print("     my_burst_write ");Serial.print(dev_addr, HEX);Serial.print(reg_addr, HEX);
    // Print the read data
    Serial.print(" Data to write: ");
    for (uint8_t i = 0; i < length; i++) {
        Serial.print(reg_data[i], HEX);
        Serial.print(" ");
    }
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);  // Write the register address
    Wire.write(reg_data, length);  // Write the data array
    uint8_t error = Wire.endTransmission();

    // Check if the transmission was successful
    if (error == 0) {
        Serial.println(" Data written successfully!");
    }
    else {
        Serial.print(" Error: ");
        Serial.println(error);
    }

    return error;
}

int8_t my_suspend_to_sleep_mode(struct bmm150_dev* dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = BMM150_OK;//KJM null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMM150_OK)
    {
        if (dev->pwr_cntrl_bit == BMM150_POWER_CNTRL_DISABLE)
        {
            rslt = my_set_power_control_bit(BMM150_POWER_CNTRL_ENABLE, dev);

            /* Start-up time delay of 3ms */
            dev->delay_us(BMM150_START_UP_TIME, dev->intf_ptr);
        }
    }

    return rslt;
}

int8_t my_write_op_mode(uint8_t op_mode, struct bmm150_dev* dev)
{
    int8_t rslt;
    uint8_t reg_data;

    /* Read the 0x4C register */
    rslt = my_bmm150_get_regs(BMM150_REG_OP_MODE, &reg_data, 1, dev);

    if (rslt == BMM150_OK)
    {
        /* Set the op_mode value in Opmode bits of 0x4C */
        reg_data = BMM150_SET_BITS(reg_data, BMM150_OP_MODE, op_mode);
        rslt = my_bmm150_set_regs(BMM150_REG_OP_MODE, &reg_data, 1, dev);
    }

    return rslt;
}

uint8_t my_are_settings_changed(uint16_t sub_settings, uint16_t desired_settings)
{
    uint8_t settings_changed;

    if (sub_settings & desired_settings)
    {
        /* User wants to modify this particular settings */
        settings_changed = BMM150_TRUE;
    }
    else
    {
        /* User don't want to modify this particular settings */
        settings_changed = BMM150_FALSE;
    }

    return settings_changed;
}

int8_t my_set_control_measurement_xyz(const struct bmm150_settings* settings, struct bmm150_dev* dev)
{
    int8_t rslt;
    uint8_t reg_data;

    rslt = bmm150_get_regs(BMM150_REG_AXES_ENABLE, &reg_data, 1, dev);

    if (rslt == BMM150_OK)
    {
        /* Set the axes to be enabled/disabled */
        reg_data = BMM150_SET_BITS(reg_data, BMM150_CONTROL_MEASURE, settings->xyz_axes_control);
        rslt = bmm150_set_regs(BMM150_REG_AXES_ENABLE, &reg_data, 1, dev);
    }

    return rslt;
}

int8_t my_mode_settings(uint16_t desired_settings, const struct bmm150_settings* settings, struct bmm150_dev* dev)
{
    int8_t rslt = BMM150_E_INVALID_CONFIG;

    if (desired_settings & BMM150_SEL_DATA_RATE)
    {
        /* Sets the ODR */
        rslt = my_set_odr(settings, dev);
    }

    if (desired_settings & BMM150_SEL_CONTROL_MEASURE)
    {
        /* Enables/Disables the control measurement axes */
        rslt = my_set_control_measurement_xyz(settings, dev);
    }

    if (desired_settings & BMM150_SEL_XY_REP)
    {
        /* Sets the XY repetition */
        rslt = my_set_xy_rep(settings, dev);
    }

    if (desired_settings & BMM150_SEL_Z_REP)
    {
        /* Sets the Z repetition */
        rslt = my_set_z_rep(settings, dev);
    }

    return rslt;
}

int8_t my_interrupt_pin_settings(uint16_t desired_settings,
    const struct bmm150_settings* settings,
    struct bmm150_dev* dev)
{
    int8_t rslt;
    uint8_t reg_data;
    struct bmm150_int_ctrl_settings int_settings;

    rslt = bmm150_get_regs(BMM150_REG_AXES_ENABLE, &reg_data, 1, dev);

    if (rslt == BMM150_OK)
    {
        int_settings = settings->int_settings;
        if (desired_settings & BMM150_SEL_DRDY_PIN_EN)
        {
            /* Enables the Data ready interrupt and
             * maps it to the DRDY pin of the sensor
             */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_DRDY_EN, int_settings.drdy_pin_en);
        }

        if (desired_settings & BMM150_SEL_INT_PIN_EN)
        {
            /* Sets interrupt pin enable */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_INT_PIN_EN, int_settings.int_pin_en);
        }

        if (desired_settings & BMM150_SEL_DRDY_POLARITY)
        {
            /* Sets Data ready pin's polarity */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_DRDY_POLARITY, int_settings.drdy_polarity);
        }

        if (desired_settings & BMM150_SEL_INT_LATCH)
        {
            /* Sets Interrupt in latched or non-latched mode */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_INT_LATCH, int_settings.int_latch);
        }

        if (desired_settings & BMM150_SEL_INT_POLARITY)
        {
            /* Sets Interrupt pin's polarity */
            reg_data = BMM150_SET_BITS_POS_0(reg_data, BMM150_INT_POLARITY, int_settings.int_polarity);
        }

        /* Set the interrupt configurations in the 0x4E register */
        rslt = bmm150_set_regs(BMM150_REG_AXES_ENABLE, &reg_data, 1, dev);
    }

    return rslt;
}

int8_t my_interrupt_config(uint16_t desired_settings, const struct bmm150_settings* settings,
    struct bmm150_dev* dev)
{
    int8_t rslt;
    uint8_t reg_data;
    struct bmm150_int_ctrl_settings int_settings;

    rslt = bmm150_get_regs(BMM150_REG_INT_CONFIG, &reg_data, 1, dev);

    if (rslt == BMM150_OK)
    {
        int_settings = settings->int_settings;
        if (desired_settings & BMM150_SEL_DATA_OVERRUN_INT)
        {
            /* Sets Data overrun interrupt */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_DATA_OVERRUN_INT, int_settings.data_overrun_en);
        }

        if (desired_settings & BMM150_SEL_OVERFLOW_INT)
        {
            /* Sets Data overflow interrupt */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_OVERFLOW_INT, int_settings.overflow_int_en);
        }

        if (desired_settings & BMM150_SEL_HIGH_THRESHOLD_INT)
        {
            /* Sets high threshold interrupt */
            reg_data = BMM150_SET_BITS(reg_data, BMM150_HIGH_THRESHOLD_INT, int_settings.high_int_en);
        }

        if (desired_settings & BMM150_SEL_LOW_THRESHOLD_INT)
        {
            /* Sets low threshold interrupt */
            reg_data = BMM150_SET_BITS_POS_0(reg_data, BMM150_LOW_THRESHOLD_INT, int_settings.low_int_en);
        }

        /* Set the interrupt configurations in the 0x4D register */
        rslt = bmm150_set_regs(BMM150_REG_INT_CONFIG, &reg_data, 1, dev);
    }

    return rslt;
}

int8_t my_interrupt_threshold_settings(uint16_t desired_settings,
    const struct bmm150_settings* settings,
    struct bmm150_dev* dev)
{
    int8_t rslt = BMM150_E_INVALID_CONFIG;
    uint8_t reg_data;

    if (desired_settings & BMM150_SEL_LOW_THRESHOLD_SETTING)
    {
        /* Sets the Low threshold value to trigger interrupt */
        reg_data = settings->int_settings.low_threshold;
        rslt = bmm150_set_regs(BMM150_REG_LOW_THRESHOLD, &reg_data, 1, dev);
    }

    if (desired_settings & BMM150_SEL_HIGH_THRESHOLD_SETTING)
    {
        /* Sets the High threshold value to trigger interrupt */
        reg_data = settings->int_settings.high_threshold;
        rslt = bmm150_set_regs(BMM150_REG_HIGH_THRESHOLD, &reg_data, 1, dev);
    }

    return rslt;
}

int8_t my_set_xy_rep(const struct bmm150_settings* settings, struct bmm150_dev* dev)
{
    int8_t rslt;
    uint8_t rep_xy;

    /* Set the xy repetition */
    rep_xy = settings->xy_rep;
    rslt = my_bmm150_set_regs(BMM150_REG_REP_XY, &rep_xy, 1, dev);

    return rslt;
}

int8_t my_set_z_rep(const struct bmm150_settings* settings, struct bmm150_dev* dev)
{
    int8_t rslt;
    uint8_t rep_z;

    /* Set the z repetition */
    rep_z = settings->z_rep;
    rslt = my_bmm150_set_regs(BMM150_REG_REP_Z, &rep_z, 1, dev);

    return rslt;
}

int8_t my_set_odr(const struct bmm150_settings* settings, struct bmm150_dev* dev)
{
    int8_t rslt;
    uint8_t reg_data;

    /* Read the 0x4C register */
    rslt = my_bmm150_get_regs(BMM150_REG_OP_MODE, &reg_data, 1, dev);

    if (rslt == BMM150_OK)
    {
        /* Set the ODR value */
        reg_data = BMM150_SET_BITS(reg_data, BMM150_ODR, settings->data_rate);
        rslt = my_bmm150_set_regs(BMM150_REG_OP_MODE, &reg_data, 1, dev);
    }

    return rslt;
}

int8_t my_set_odr_xyz_rep(const struct bmm150_settings* settings, struct bmm150_dev* dev)
{
    int8_t rslt;

    /* Set the ODR */
    rslt = my_set_odr(settings, dev);

    if (rslt == BMM150_OK)
    {
        /* Set the XY-repetitions number */
        rslt = my_set_xy_rep(settings, dev);

        if (rslt == BMM150_OK)
        {
            /* Set the Z-repetitions number */
            rslt = my_set_z_rep(settings, dev);
        }
    }

    return rslt;
}

int8_t my_set_config(struct bmm150_dev* dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    struct bmm150_settings settings;

    /* Set powermode as normal mode */
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = my_bmm150_set_op_mode(&settings, dev);

    if (rslt == BMM150_OK)
    {
        settings.preset_mode = BMM150_PRESETMODE_LOWPOWER;
        rslt = my_bmm150_set_presetmode(&settings, dev);

        if (rslt == BMM150_OK)
        {
            /* Map the data interrupt pin */
            settings.int_settings.drdy_pin_en = 0x01;
            rslt = my_bmm150_set_sensor_settings(BMM150_SEL_DRDY_PIN_EN, &settings, dev);
        }
    }

    my_bmm150_error_codes_print_result("my_set_config: ", rslt);
    return rslt;
}

int8_t my_set_power_control_bit(uint8_t pwrcntrl_bit, struct bmm150_dev* dev)
{
    int8_t rslt;
    uint8_t reg_data = 0;

    /* Power control register 0x4B is read */
    rslt = my_bmm150_get_regs(BMM150_REG_POWER_CONTROL, &reg_data, 1, dev);

    /* Proceed if everything is fine until now */
    if (rslt == BMM150_OK)
    {
        /* Sets the value of power control bit */
        reg_data = BMM150_SET_BITS_POS_0(reg_data, BMM150_PWR_CNTRL, pwrcntrl_bit);
        rslt = my_bmm150_set_regs(BMM150_REG_POWER_CONTROL, &reg_data, 1, dev);

        if (rslt == BMM150_OK)
        {
            /* Store the power control bit
             * value in dev structure
             */
            dev->pwr_cntrl_bit = pwrcntrl_bit;
        }
    }

    my_bmm150_error_codes_print_result("my_set_power_control_bit: ", rslt);
    return rslt;
}

int8_t my_read_trim_registers(struct bmm150_dev* dev)
{
    int8_t rslt;
    uint8_t trim_x1y1[2] = { 0 };
    uint8_t trim_xyz_data[4] = { 0 };
    uint8_t trim_xy1xy2[10] = { 0 };
    uint16_t temp_msb = 0;

    /* Trim register value is read */
    rslt = bmm150_get_regs(BMM150_DIG_X1, trim_x1y1, 2, dev);

    if (rslt == BMM150_OK)
    {
        rslt = bmm150_get_regs(BMM150_DIG_Z4_LSB, trim_xyz_data, 4, dev);

        if (rslt == BMM150_OK)
        {
            rslt = bmm150_get_regs(BMM150_DIG_Z2_LSB, trim_xy1xy2, 10, dev);

            if (rslt == BMM150_OK)
            {
                /* Trim data which is read is updated
                 * in the device structure
                 */
                dev->trim_data.dig_x1 = (int8_t)trim_x1y1[0];
                dev->trim_data.dig_y1 = (int8_t)trim_x1y1[1];
                dev->trim_data.dig_x2 = (int8_t)trim_xyz_data[2];
                dev->trim_data.dig_y2 = (int8_t)trim_xyz_data[3];
                temp_msb = ((uint16_t)trim_xy1xy2[3]) << 8;
                dev->trim_data.dig_z1 = (uint16_t)(temp_msb | trim_xy1xy2[2]);
                temp_msb = ((uint16_t)trim_xy1xy2[1]) << 8;
                dev->trim_data.dig_z2 = (int16_t)(temp_msb | trim_xy1xy2[0]);
                temp_msb = ((uint16_t)trim_xy1xy2[7]) << 8;
                dev->trim_data.dig_z3 = (int16_t)(temp_msb | trim_xy1xy2[6]);
                temp_msb = ((uint16_t)trim_xyz_data[1]) << 8;
                dev->trim_data.dig_z4 = (int16_t)(temp_msb | trim_xyz_data[0]);
                dev->trim_data.dig_xy1 = trim_xy1xy2[9];
                dev->trim_data.dig_xy2 = (int8_t)trim_xy1xy2[8];
                temp_msb = ((uint16_t)(trim_xy1xy2[5] & 0x7F)) << 8;
                dev->trim_data.dig_xyz1 = (uint16_t)(temp_msb | trim_xy1xy2[4]);
            }
        }
    }

    return rslt;
}

void my_bmm150_error_codes_print_result(char funcName[], int8_t result) {
    Serial.print(funcName);
    Serial.println(result);
}

void my_bmm150_user_delay_us(uint32_t period_us, void* intf_ptr)
{
    delay(period_us);
}

int8_t my_bmm150_user_i2c_reg_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t length, void* intf_ptr)
{
    /*
    const struct device* i2c_dev = DEVICE_DT_GET(DT_ALIAS(i2csensor));
    if (!i2c_dev) {
        return -1;
    }

    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    */
    return my_burst_write(reg_addr, reg_data, length);
}

int8_t my_bmm150_user_i2c_reg_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t length, void* intf_ptr)
{
    /*
    const struct device* i2c_dev = DEVICE_DT_GET(DT_ALIAS(i2csensor));
    if (!i2c_dev) {
        return -1;
    }

    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    */
    my_burst_read(reg_addr, reg_data, length);
    return 0;
}

int8_t my_bmm150_get_regs(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, struct bmm150_dev* dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = BMM150_OK;//KJM null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMM150_OK) && (reg_data != NULL))
    {
        if (dev->intf != BMM150_I2C_INTF)
        {
            /* If interface selected is SPI */
            reg_addr = reg_addr | 0x80;
        }

        /* Read the data from the reg_addr */
        rslt = dev->read(reg_addr, reg_data, len, dev->intf_ptr);
    }
    else
    {
        rslt = BMM150_E_NULL_PTR;
    }

    //KJMmy_bmm150_error_codes_print_result("my_bmm150_get_regs: ", rslt);
    return rslt;
}

int8_t my_bmm150_set_regs(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, struct bmm150_dev* dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = BMM150_OK;//KJM null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMM150_OK) && (reg_data != NULL) && (len != 0))
    {
        /* Write the data to the reg_addr */

        /* SPI write requires to set The MSB of reg_addr as 0
         * but in default the MSB is always 0
         */
        dev->intf_rslt = dev->write(reg_addr, reg_data, len, dev->intf_ptr);
    }
    else
    {
        rslt = BMM150_E_NULL_PTR;
    }

    //KJMmy_bmm150_error_codes_print_result("my_bmm150_set_regs: ", rslt);
    return rslt;
}

int8_t my_bmm150_set_op_mode(const struct bmm150_settings* settings, struct bmm150_dev* dev)
{
    int8_t rslt;
    uint8_t pwr_mode;

    /* Check for null pointer in the device structure */
    rslt = BMM150_OK;//KJM null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMM150_OK)
    {
        pwr_mode = settings->pwr_mode;

        /* Select the power mode to set */
        switch (pwr_mode)
        {
        case BMM150_POWERMODE_NORMAL:

            /* If the sensor is in suspend mode
             * put the device to sleep mode
             */
            rslt = my_suspend_to_sleep_mode(dev);
            if (rslt == BMM150_OK)
            {
                /* write the op mode */
                rslt = my_write_op_mode(pwr_mode, dev);
            }

            break;
        case BMM150_POWERMODE_FORCED:

            /* If the sensor is in suspend mode
             * put the device to sleep mode
             */
            rslt = my_suspend_to_sleep_mode(dev);
            if (rslt == BMM150_OK)
            {
                /* write the op mode */
                rslt = my_write_op_mode(pwr_mode, dev);
            }

            break;
        case BMM150_POWERMODE_SLEEP:

            /* If the sensor is in suspend mode
             * put the device to sleep mode
             */
            rslt = my_suspend_to_sleep_mode(dev);
            if (rslt == BMM150_OK)
            {
                /* write the op mode */
                rslt = my_write_op_mode(pwr_mode, dev);
            }

            break;
        case BMM150_POWERMODE_SUSPEND:

            /* Set the power control bit to zero */
            rslt = my_set_power_control_bit(BMM150_POWER_CNTRL_DISABLE, dev);
            break;
        default:
            rslt = BMM150_E_INVALID_CONFIG;
            break;
        }
    }

    my_bmm150_error_codes_print_result("my_bmm150_set_op_mode: ", rslt);
    return rslt;
}

int8_t my_bmm150_set_sensor_settings(uint16_t desired_settings,
    const struct bmm150_settings* settings,
    struct bmm150_dev* dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = BMM150_OK;//KJM null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMM150_OK)
    {
        if (my_are_settings_changed(MODE_SETTING_SEL, desired_settings))
        {
            /* ODR, Control measurement, XY,Z repetition values */
            rslt = my_mode_settings(desired_settings, settings, dev);
        }

        if ((!rslt) && my_are_settings_changed(INTERRUPT_PIN_SETTING_SEL, desired_settings))
        {
            /* Interrupt pin settings */
            rslt = my_interrupt_pin_settings(desired_settings, settings, dev);
        }

        if ((!rslt) && my_are_settings_changed(INTERRUPT_CONFIG_SEL, desired_settings))
        {
            /* Interrupt configuration settings */
            rslt = my_interrupt_config(desired_settings, settings, dev);
        }

        if ((!rslt) && my_are_settings_changed(INTERRUPT_THRESHOLD_CONFIG_SEL, desired_settings))
        {
            /* Interrupt threshold settings */
            rslt = my_interrupt_threshold_settings(desired_settings, settings, dev);
        }
    }

    return rslt;
}

int8_t my_bmm150_init(struct bmm150_dev* dev)
{
    int8_t rslt;
    uint8_t chip_id = 0;

    /* Power up the sensor from suspend to sleep mode */
    rslt = my_set_power_control_bit(BMM150_POWER_CNTRL_ENABLE, dev);

    if (rslt == BMM150_OK)
    {
        /* Start-up time delay of 3ms */
        dev->delay_us(BMM150_START_UP_TIME, dev->intf_ptr);

        /* Chip ID of the sensor is read */
        rslt = my_bmm150_get_regs(BMM150_REG_CHIP_ID, &chip_id, 1, dev);

        /* Proceed if everything is fine until now */
        if (rslt == BMM150_OK)
        {
            /* Check for chip id validity */
            if (chip_id == BMM150_CHIP_ID)
            {
                dev->chip_id = chip_id;

                /* Function to update trim values */
                rslt = my_read_trim_registers(dev);
            }
        }
    }

    my_bmm150_error_codes_print_result("my_bmm150_init: ", rslt);
    return rslt;
}

int8_t my_bmm150_set_presetmode(struct bmm150_settings* settings, struct bmm150_dev* dev)
{
    int8_t rslt;
    uint8_t preset_mode;

    /* Check for null pointer in the device structure */
    rslt = BMM150_OK;//KJM null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMM150_OK)
    {
        preset_mode = settings->preset_mode;

        switch (preset_mode)
        {
        case BMM150_PRESETMODE_LOWPOWER:

            /* Set the data rate x,y,z repetition
             * for Low Power mode
             */
            settings->data_rate = BMM150_DATA_RATE_10HZ;
            settings->xy_rep = BMM150_REPXY_LOWPOWER;
            settings->z_rep = BMM150_REPZ_LOWPOWER;
            rslt = my_set_odr_xyz_rep(settings, dev);
            break;
        case BMM150_PRESETMODE_REGULAR:

            /* Set the data rate x,y,z repetition
             * for Regular mode
             */
            settings->data_rate = BMM150_DATA_RATE_10HZ;
            settings->xy_rep = BMM150_REPXY_REGULAR;
            settings->z_rep = BMM150_REPZ_REGULAR;
            rslt = my_set_odr_xyz_rep(settings, dev);
            break;
        case BMM150_PRESETMODE_HIGHACCURACY:

            /* Set the data rate x,y,z repetition
             * for High Accuracy mode *
             */
            settings->data_rate = BMM150_DATA_RATE_20HZ;
            settings->xy_rep = BMM150_REPXY_HIGHACCURACY;
            settings->z_rep = BMM150_REPZ_HIGHACCURACY;
            rslt = my_set_odr_xyz_rep(settings, dev);
            break;
        case BMM150_PRESETMODE_ENHANCED:

            /* Set the data rate x,y,z repetition
             * for Enhanced Accuracy mode
             */
            settings->data_rate = BMM150_DATA_RATE_10HZ;
            settings->xy_rep = BMM150_REPXY_ENHANCED;
            settings->z_rep = BMM150_REPZ_ENHANCED;
            rslt = my_set_odr_xyz_rep(settings, dev);
            break;
        default:
            rslt = BMM150_E_INVALID_CONFIG;
            break;
        }
    }

    return rslt;
}

int8_t my_bmm150_user_i2c_init(void)
{
    /* Implement I2C bus initialization according to the target machine. */
    return 0;
}

int8_t my_bmm150_interface_selection(struct bmm150_dev* dev)
{
    int8_t rslt = BMM150_OK;

    if (dev != NULL) {
        dev->intf = BMM150_I2C_INTF;

        /* Bus configuration : I2C */
        if (true)//dev->intf == BMM150_I2C_INTF) 
        {
            //LOG_DBG("I2C Interface \n");

            /* To initialize the user I2C function */
            my_bmm150_user_i2c_init();

            dev_addr = BMM150_DEFAULT_I2C_ADDRESS;
            dev->read = my_bmm150_user_i2c_reg_read;
            dev->write = my_bmm150_user_i2c_reg_write;
        }

        /* Assign device address to interface pointer */
        dev->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        dev->delay_us = my_bmm150_user_delay_us;
    }
    else {
        rslt = BMM150_E_NULL_PTR;
    }

    my_bmm150_error_codes_print_result("my_bmm150_interface_selection: ", rslt);
    return rslt;
}

void initMyBMM() {
    /* Status of api are returned to this variable */
    int8_t rslt;

    rslt = my_bmm150_interface_selection(&bmm1_dev);
    if (rslt == BMM150_OK) {
        rslt = my_bmm150_init(&bmm1_dev);

        if (rslt == BMM150_OK) {
            rslt = my_set_config(&bmm1_dev);
        }
        hasMag = true;
        Serial.println("initMyBMM: Complete ");
    }
    else {
        Serial.println("Unable to init BMM150");
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial);
    // Initialize I2C communication
    Wire.begin();
    Serial.println("*******************BMMTest2*******************");

    initMyBMM();

    Serial.println("**********************************************");
    while (true) {}
}

void loop() {
        ++loopCnt;
        serialData();
}
