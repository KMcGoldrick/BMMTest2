#include <bmm150_defs.h>
#include <bmm150.h>
#include <Wire.h>


int loopCnt;
int8_t rslt;
float x, y, z;
bool hasMag;
struct bmm150_dev bmm1_dev;
uint8_t dev_addr;

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
    Serial.print("Data read: ");
    for (uint8_t i = 0; i < length; i++) {
        Serial.print(reg_data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

}

uint8_t my_burst_write(uint8_t reg_addr, const uint8_t* reg_data, int length) {
    // Perform the burst write
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);  // Write the register address
    Wire.write(reg_data, length);  // Write the data array
    uint8_t error = Wire.endTransmission();

    // Check if the transmission was successful
    if (error == 0) {
        Serial.println("Data written successfully!");
    }
    else {
        Serial.print("Error: ");
        Serial.println(error);
    }

    return error;
}

void bmm150_error_codes_print_result(char funcName[], int8_t result) {
    Serial.print(funcName);
    Serial.println(result);
}

int8_t bmm150_user_i2c_init(void)
{
    /* Implement I2C bus initialization according to the target machine. */
    return 0;
}

void bmm150_user_delay_us(uint32_t period_us, void* intf_ptr)
{
    delay(1000*period_us);
}

int8_t bmm150_user_i2c_reg_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t length,
    void* intf_ptr)
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

int8_t bmm150_user_i2c_reg_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t length,
    void* intf_ptr)
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

int8_t bmm150_interface_selection(struct bmm150_dev* dev)
{
    int8_t rslt = BMM150_OK;

    if (dev != NULL) {
        dev->intf = BMM150_I2C_INTF;

        /* Bus configuration : I2C */
        if (true)//dev->intf == BMM150_I2C_INTF) 
        {
            //LOG_DBG("I2C Interface \n");

            /* To initialize the user I2C function */
            bmm150_user_i2c_init();

            dev_addr = BMM150_DEFAULT_I2C_ADDRESS;
            dev->read = bmm150_user_i2c_reg_read;
            dev->write = bmm150_user_i2c_reg_write;
        }

        /* Assign device address to interface pointer */
        dev->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        dev->delay_us = bmm150_user_delay_us;
    }
    else {
        rslt = BMM150_E_NULL_PTR;
    }

    return rslt;
}

void initMyBMM() {
    /* Status of api are returned to this variable */
    int8_t rbslt;

    rbslt = bmm150_interface_selection(&bmm1_dev);
    bmm150_error_codes_print_result("bmm150_interface_selection", rbslt);
    if (rbslt == BMM150_OK) {
        rbslt = bmm150_init(&bmm1_dev);
        bmm150_error_codes_print_result("bmm150_init", rbslt);

        if (rbslt == BMM150_OK) {
            rbslt = 247;//KJM set_config(&bmm1_dev);
            bmm150_error_codes_print_result("set_config", rbslt);
        }
        hasMag = true;
    }
    else {
        Serial.println("Unable to init BMM150");
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("*******************BMMTest2*******************");

    initMyBMM();

    Serial.println("**********************************************");
    while (true) {}
}

void loop() {
        ++loopCnt;
        serialData();
}
