package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BNO055;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "BNO055 Absolute Orientation Sensor", xmlTag = "BNO055")
public class BNO055 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {
        return "Adafruit BNO055 Absolute Orientation Sensor";
    }

    private I2cAddr ADDRESS_I2C_DEFAULT = new I2cAddr(0x28); // Default I2C address

    public enum Register {
        FIRST(0),
        CHIP_ID(0x00),
        ACCEL_DATA_X_LSB(0x08),
        ACCEL_DATA_X_MSB(0x09),
        ACCEL_DATA_Y_LSB(0x0A),
        ACCEL_DATA_Y_MSB(0x0B),
        ACCEL_DATA_Z_LSB(0x0C),
        ACCEL_DATA_Z_MSB(0x0D),
        MAG_DATA_X_LSB(0x0E),
        MAG_DATA_X_MSB(0x0F),
        MAG_DATA_Y_LSB(0x10),
        MAG_DATA_Y_MSB(0x11),
        MAG_DATA_Z_LSB(0x12),
        MAG_DATA_Z_MSB(0x13),
        GYRO_DATA_X_LSB(0x14),
        GYRO_DATA_X_MSB(0x15),
        GYRO_DATA_Y_LSB(0x16),
        GYRO_DATA_Y_MSB(0x17),
        GYRO_DATA_Z_LSB(0x18),
        GYRO_DATA_Z_MSB(0x19),
        EULER_ANGLES_H(0x1A),
        EULER_ANGLES_L(0x1B),
        QUATERNION_1_LSB(0x20),
        QUATERNION_1_MSB(0x21),
        QUATERNION_2_LSB(0x22),
        QUATERNION_2_MSB(0x23),
        QUATERNION_3_LSB(0x24),
        QUATERNION_3_MSB(0x25),
        QUATERNION_4_LSB(0x26),
        QUATERNION_4_MSB(0x27),
        LIN_ACC_DATA_X_LSB(0x28),
        LIN_ACC_DATA_X_MSB(0x29),
        LIN_ACC_DATA_Y_LSB(0x2A),
        LIN_ACC_DATA_Y_MSB(0x2B),
        LIN_ACC_DATA_Z_LSB(0x2C),
        LIN_ACC_DATA_Z_MSB(0x2D),
        GRAVITY_DATA_X_LSB(0x2E),
        GRAVITY_DATA_X_MSB(0x2F),
        GRAVITY_DATA_Y_LSB(0x30),
        GRAVITY_DATA_Y_MSB(0x31),
        GRAVITY_DATA_Z_LSB(0x32),
        GRAVITY_DATA_Z_MSB(0x33),
        TEMP_DATA_LSB(0x34),
        TEMP_DATA_MSB(0x35),
        STATUS(0x39),
        POWER_MODE(0x3E),
        OPR_MODE(0x3F),
        SYS_TRIGGER(0x40),
        TEMP_SOURCE(0x41),
        RESET(0x42),
        INT_EN(0x50),
        INT_MAP(0x51),
        INT_STATUS(0x52),
        ACCEL_CONFIG(0x53),
        MAG_CONFIG(0x54),
        GYRO_CONFIG(0x55),
        CONFIG_MODE(0x56),
        SENS_CONFIG(0x57),
        ID(0xD0),
        LAST(ID.bVal);

        public int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    // Set the read window for optimal performance
    protected void setOptimalReadWindow() {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    // Constructor
    public BNO055(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        this.deviceClient.engage();
    }

    // Helper method to write a short value to a register
    protected void writeShort(Register reg, short value) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    // Helper method to read a short value from a register
    protected short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    // Method to initialize the sensor
    public void init() {
        // Set the sensor to CONFIG_MODE for configuration
        setSensorMode(Register.CONFIG_MODE);

        // Perform any necessary reset (software reset)
        resetSensor();

        // Set the sensor to NDOF mode (9-DOF sensor fusion mode)
        setSensorMode(Register.OPR_MODE);

        // Wait for initialization to complete
        waitForInitialization();
    }

    // Reset the sensor (software reset)
    private void resetSensor() {
        writeShort(Register.RESET, (short) 0x20);  // 0x20 is the reset command
    }

    // Set the operation mode of the sensor
    private void setSensorMode(Register mode) {
        writeShort(Register.OPR_MODE, (short) mode.bVal);
    }

    // Wait for the sensor to initialize and be ready
    private void waitForInitialization() {
        int status = readShort(Register.STATUS);
        while ((status & 0x01) == 0) {  // Check the "System Error" bit in STATUS register
            status = readShort(Register.STATUS);
            try {
                Thread.sleep(100);  // Wait a bit before checking again
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    // Perform the calibration process
    public void calibrate() {
        // Start calibration by setting the sensor to CONFIG_MODE
        setSensorMode(Register.CONFIG_MODE);

        // Read the calibration status
        int calibrationStatus = readShort(Register.STATUS);

        // Wait for the sensor to be calibrated (check calibration status bits)
        while (calibrationStatus != 0x3F) {  // Calibration complete status = 0x3F
            calibrationStatus = readShort(Register.STATUS);
            try {
                Thread.sleep(100);  // Wait a bit before checking again
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        // Once calibration is done, set the sensor to NDOF mode
        setSensorMode(Register.OPR_MODE);
    }

    // Check if the sensor is calibrated
    public boolean isCalibrated() {
        int calibrationStatus = readShort(Register.STATUS);
        return (calibrationStatus == 0x3F);  // 0x3F means all sensors are calibrated
    }
}