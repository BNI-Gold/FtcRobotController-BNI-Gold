package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Outgrabber;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outgrabber {
    public HardwareMap hwBot = null;
    public Servo outgrabber = null;
    public Servo tilt = null;
    public Servo rotate = null;

    double grabberOpen = .3072;
    double grabberClosed = .13;

    //TODO: calibrate
    double grabberUp = .3433;
    double grabberMid = .2472;
    double grabberDown = .2044;

    public double straight = .3756;
    public double right = .7206;
    public double left = .0444;

    public double grabberAdjust = .001;
    public double rotationAdjust = .001;
    public double tiltAdjust = .001;

    public Outgrabber() {}

    public void initOutgrabber(HardwareMap hwMap) {
        hwBot = hwMap;

        outgrabber = hwBot.servo.get("outgrabber");
        tilt = hwBot.servo.get("outgrabber_tilt");
        rotate = hwBot.servo.get("outgrabber_rotate");

        outgrabber.setDirection(Servo.Direction.FORWARD);
        tilt.setDirection(Servo.Direction.FORWARD);
        rotate.setDirection(Servo.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }

    public void release() {
        outgrabber.setPosition(grabberOpen);
    }

    public void grab() {
        outgrabber.setPosition(grabberClosed);
    }

    public void goOpen() {
        double position = outgrabber.getPosition();
        outgrabber.setPosition(position + grabberAdjust);
    }

    public void goClose() {
        double position = outgrabber.getPosition();
        outgrabber.setPosition(position - grabberAdjust);
    }

    public void headStraight() {
        rotate.setPosition(straight);
    }

    public void headRight() {
        rotate.setPosition(right);
    }

    public void headLeft() {
        rotate.setPosition(left);
    }

    public void rotateRight() {
        double position = rotate.getPosition();
        rotate.setPosition(position + rotationAdjust);
    }

    public void rotateLeft() {
        double position = rotate.getPosition();
        rotate.setPosition(position - rotationAdjust);
    }

    public void rotate(double x, double y) {
        // Calculate the angle in radians, then convert to degrees
        double angle = Math.atan2(-x, -y); // Negative y to match joystick orientation
        angle = Math.toDegrees(angle);

        // Normalize the angle to a range of [0, 360]
        if (angle < 0) {
            angle += 360;
        }

        // Map the angle to the servo range
        double servoPosition;
        if (angle >= 90 && angle <= 270) {
            // Map from 90° to 270° to servo range (left to right)
            servoPosition = map(angle, 90, 270, left, right);
        } else {
            // Wrap around for angles outside 90° to 270°
            if (angle < 90) {
                angle += 360; // e.g., -45 becomes 315
            }
            servoPosition = map(angle, 270, 450, right, left); // Right to left
        }

        // Set the servo position
        rotate.setPosition(servoPosition);
    }

    private double map(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
        return toLow + (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow);
    }

    public void tiltUp() {
        double position = tilt.getPosition();
        tilt.setPosition(Math.min(position + tiltAdjust, 1.0)); // Ensure position does not exceed 1.0
    }

    public void tiltDown() {
        double position = tilt.getPosition();
        tilt.setPosition(Math.max(position - tiltAdjust, 0.0)); // Ensure position does not go below 0.0
    }

    public void tiltUp(double mult) {
        double position = tilt.getPosition();
        tilt.setPosition(Math.min(position + (tiltAdjust * 7 * mult), 1.0)); // Ensure position does not exceed 1.0
    }

    public void tiltDown(double mult) {
        double position = tilt.getPosition();
        tilt.setPosition(Math.max(position - (tiltAdjust * 7 * mult), 0.0)); // Ensure position does not go below 0.0
    }

    public void upPosition() {
        tilt.setPosition(grabberUp);
    }

    public void midPosition() {
        tilt.setPosition(grabberMid);
    }

    public void downPosition() {
        tilt.setPosition(grabberDown);
    }
}
