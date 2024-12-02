//package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BNO055;
//
//import com.qualcomm.robotcore.hardware.I2cAddr;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
//import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
//import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
//import com.qualcomm.robotcore.util.TypeConversion;
//
//@I2cDeviceType
//@DeviceProperties(name = "BNO055 Absolute Orientation Sensor", xmlTag = "BNO055")
//public class BNO055 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
//
//    @Override
//    public Manufacturer getManufacturer() {
//        return Manufacturer.Adafruit;
//    }
//
//    @Override
//    protected synchronized boolean doInitialize() {
//        return true;
//    }
//
//    @Override
//    public String getDeviceName() {
//        return "Adafruit BNO055 Absolute Orientation Sensor";
//    }
//
//    private I2cAddr ADDRESS_I2C_DEFAULT = new I2cAddr(0x28); // Default I2C address
//
//    public enum Register {
//        FIRST(0),
//        MAG_RADIUS_MSB(0x6A),
//        MAG_RADIUS_LSB(0x69),
//        ACCEL_RADIUS_MSB(0x68),
//        ACCEL_RADIUS_LSB(0x67),
//        GYR_OFFSET_Z_MSB(0x66),
//        GYR_OFFSET_Z_LSB(0x65),
//        GYR_OFFSET_Y_MSB(0x64),
//        GYR_OFFSET_Y_LSB(0x63),
//        GYR_OFFSET_X_MSB(0x62),
//        GYR_OFFSET_X_LSB(0x61),
//        MAG_OFFSET_Z_MSB(0x60),
//        MAG_OFFSET_Z_LSB(0x5F),
//        MAG_OFFSET_Y_MSB(0x5E),
//        MAG_OFFSET_Y_LSB(0x5D),
//        MAG_OFFSET_X_MSB(0x5C),
//        MAG_OFFSET_X_LSB(0x5B),
//        ACC_OFFSET_Z_MSB(0x5A),
//        ACC_OFFSET_Z_LSB(0x59),
//        ACC_OFFSET_Y_MSB(0x58),
//        ACC_OFFSET_Y_LSB(0x57),
//        ACC_OFFSET_X_MSB(0x56),
//        ACC_OFFSET_X_LSB(0x55),
//
//        LAST(FIRST.bVal);
//
//        public int bVal;
//
//        Register(int bVal) {
//            this.bVal = bVal;
//        }
//    }
//
//    // Set the read window for optimal performance
//    protected void setOptimalReadWindow() {
//        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
//                Register.FIRST.bVal,
//                Register.LAST.bVal - Register.FIRST.bVal + 1,
//                I2cDeviceSynch.ReadMode.REPEAT);
//        this.deviceClient.setReadWindow(readWindow);
//    }
//
//    // Constructor
//    public BNO055(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
//        super(deviceClient, deviceClientIsOwned);
//        this.setOptimalReadWindow();
//        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
//        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
//        this.deviceClient.engage();
//    }
//
//    // Helper method to write a short value to a register
//    protected void writeShort(Register reg, short value) {
//        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
//    }
//
//    // Helper method to read a short value from a register
//    protected short readShort(Register reg) {
//        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
//    }
//
//    // Method to initialize the sensor
//    public void init() {
//        // Set the sensor to CONFIG_MODE for configuration
//        setSensorMode(Register.CONFIG_MODE);
//
//        // Perform any necessary reset (software reset)
//        resetSensor();
//
//        // Set the sensor to NDOF mode (9-DOF sensor fusion mode)
//        setSensorMode(Register.OPR_MODE);
//
//        // Wait for initialization to complete
//        waitForInitialization();
//    }
//
//    // Reset the sensor (software reset)
//    private void resetSensor() {
//        writeShort(Register.RESET, (short) 0x20);  // 0x20 is the reset command
//    }
//
//    // Set the operation mode of the sensor
//    private void setSensorMode(Register mode) {
//        writeShort(Register.OPR_MODE, (short) mode.bVal);
//    }
//
//    // Wait for the sensor to initialize and be ready
//    private void waitForInitialization() {
//        int status = readShort(Register.STATUS);
//        while ((status & 0x01) == 0) {  // Check the "System Error" bit in STATUS register
//            status = readShort(Register.STATUS);
//            try {
//                Thread.sleep(100);  // Wait a bit before checking again
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
//    }
//
//    // Perform the calibration process
//    public void calibrate() {
//        // Start calibration by setting the sensor to CONFIG_MODE
//        setSensorMode(Register.CONFIG_MODE);
//
//        // Read the calibration status
//        int calibrationStatus = readShort(Register.STATUS);
//
//        // Wait for the sensor to be calibrated (check calibration status bits)
//        while (calibrationStatus != 0x3F) {  // Calibration complete status = 0x3F
//            calibrationStatus = readShort(Register.STATUS);
//            try {
//                Thread.sleep(100);  // Wait a bit before checking again
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
//
//        // Once calibration is done, set the sensor to NDOF mode
//        setSensorMode(Register.OPR_MODE);
//    }
//
//    // Check if the sensor is calibrated
//    public boolean isCalibrated() {
//        int calibrationStatus = readShort(Register.STATUS);
//        return (calibrationStatus == 0x3F);  // 0x3F means all sensors are calibrated
//    }
//}