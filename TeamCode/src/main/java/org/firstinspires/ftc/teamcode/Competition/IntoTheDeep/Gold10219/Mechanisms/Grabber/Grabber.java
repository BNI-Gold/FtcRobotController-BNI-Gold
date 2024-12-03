package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Grabber;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Grabber {
    public HardwareMap hwBot = null;
    public Servo grabber = null;
    public Servo tilt = null;
    public Servo rotate = null;
    public BNO055IMU imu = null;

    //TODO: Update these values
    double grabberOpen = .625;
    double grabberClosed = .925;

    private Orientation angles;
    public float heading = 0;

    public double tuckPosition = .6428;

    public double straight = .5261;
    public double right = .8511;
    public double qRight = .6866;
    public double left = .1906;
    public double qLeft = .3584;

    public double grabberAdjust = .001;
    public double rotationAdjust = .001;
    public double tiltAdjust = .001;

    public double outAngle = 45;
    public double downAngle = -45;

    public double angleDeadband = 5;
    public double servoDeadband = .005;

    public Grabber() {}

    public void initGrabber(HardwareMap hwMap) {
        hwBot = hwMap;

        grabber = hwBot.servo.get("grabber");
        tilt = hwBot.servo.get("tilt");
        rotate = hwBot.servo.get("rotate");

        grabber.setDirection(Servo.Direction.FORWARD);
        tilt.setDirection(Servo.Direction.FORWARD);
        rotate.setDirection(Servo.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwBot.get(BNO055IMU.class, "bnoimu");
        imu.initialize(parameters);
    }

    public void open() {
        grabber.setPosition(grabberOpen);
    }

    public void close() {
        grabber.setPosition(grabberClosed);
    }

    public void goOpen() {
        double position = grabber.getPosition();
        grabber.setPosition(position + grabberAdjust);
    }

    public void goClose() {
        double position = grabber.getPosition();
        grabber.setPosition(position - grabberAdjust);
    }

    private void tuck() {
        tilt.setPosition(tuckPosition);
    }

    public void headStraight() {
        rotate.setPosition(straight);
    }

    public void headQRight() {
        rotate.setPosition(qRight);
    }

    public void headRight() {
        rotate.setPosition(right);
    }

    public void headQLeft() {
        rotate.setPosition(qLeft);
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
        double angle = Math.atan2(-y, x); // Negative y to match joystick orientation
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

    public double ang = 0;
    public double diff = 0;
    public double diff1 = 0;
    public double pch = 0;
    public double csp = 0;
    public double nsp = 0;
    public double nsp2 = 0;

    public enum grabberStates {
        OUT, DOWN, MANUAL
    }

    public enum tiltStates {
        SETTLED, CALL_TILT, TILTING
    }

    public grabberStates grabberState = grabberStates.MANUAL;

    public tiltStates tiltState = tiltStates.SETTLED;

    public void setGrabberState(grabberStates state) {
        grabberState = state;
    }

    private double desiredPos = 0;

    public void tiltStateCheck() {
        switch (tiltState) {
            case SETTLED:
                if (grabberState != grabberStates.MANUAL) {
                    tiltState = tiltStates.CALL_TILT;
                }
                break;

            case CALL_TILT:
                double desiredAngle = 0;

                switch (grabberState) {
                    case OUT:
                        desiredAngle = outAngle;
                        break;
                    case DOWN:
                        desiredAngle = downAngle;
                        break;
                }

                double currentAngle = getTilt();
                ang = currentAngle;

                double angleDifference = desiredAngle - currentAngle;
                diff = angleDifference;

                // Normalize the difference to avoid wrapping issues
                if (angleDifference > 150) {
                    angleDifference -= 300;
                } else if (angleDifference < -150) {
                    angleDifference += 300;
                }
                diff1 = angleDifference;

                // Deadband to prevent unnecessary adjustments
                if (Math.abs(angleDifference) < angleDeadband) {
                    tiltState = tiltStates.SETTLED;
                    return;
                }

                // Apply a damping factor to reduce overshooting
                double dampingFactor = 0.5; // Adjust this value (e.g., 0.1 to 0.5) to fine-tune correction speed
                double positionChange = (angleDifference / 300.0) * dampingFactor;
                pch = positionChange;

                double currentServoPosition = tilt.getPosition();
                csp = currentServoPosition;

                double newServoPosition = currentServoPosition + positionChange;
                nsp = newServoPosition;

                // Clamp the servo position to the valid range [0.4, 1]
                newServoPosition = Range.clip(newServoPosition, 0.4, 1);
                desiredPos = newServoPosition;
                nsp2 = newServoPosition;

                tiltState = tiltStates.TILTING;

                tilt.setPosition(newServoPosition);
                break;

            case TILTING:
                double servoPosition = tilt.getPosition();
                double posDiff = servoPosition - desiredPos;

                // If the servo position is within the deadband, consider the movement complete
                if (Math.abs(posDiff) < servoDeadband) {
                    tiltState = tiltStates.SETTLED;
                }
                break;
        }
    }

//    public void tiltStateCheck() {
//        if (doForThis) return;
//        double desiredAngle = 0;
//        switch (currentState) {
//            case OUT:
//                desiredAngle = 45;
//                doForThis = true;
//                break;
//            case DOWN:
//                desiredAngle = -45;
//                doForThis = true;
//                break;
//            case CONTROL:
//                desiredAngle = -1000;
//                break;
//        }
//
//        if (desiredAngle == -1000) return;
//        // Get the current tilt angle of the grabber from the IMU
//        double currentAngle = getTilt(); // Measured in degrees
//
//        ang = currentAngle;
//
//        // Calculate the difference between the current and desired angles
//        double angleDifference = desiredAngle - currentAngle;
//
//        diff = angleDifference;
//
//        // Normalize the difference to avoid wrapping issues
//        if (angleDifference > 150) {
//            angleDifference -= 300;
//        } else if (angleDifference < -150) {
//            angleDifference += 300;
//        }
//
//        diff1 = angleDifference;
//
//        // Deadband to prevent unnecessary adjustments
//        if (Math.abs(angleDifference) < 2) {
//            return; // No adjustment needed
//        }
//
//        // Map the angle difference to a servo position adjustment
//        double positionChange = angleDifference / 300.0; // Scale to the servo range
//
//        pch = positionChange;
//
//        // Get the current servo position
//        double currentServoPosition = tilt.getPosition();
//
//        csp = currentServoPosition;
//
//        // Calculate the new servo position
//        double newServoPosition = currentServoPosition + positionChange;
//
//        nsp = newServoPosition;
//
//        // Clamp the servo position to the valid range [0.4, 1.0]
//        newServoPosition = Range.clip(newServoPosition, .4, 1);
//
//        nsp2 = newServoPosition;
//
//        // Set the servo to the new position
//        tilt.setPosition(newServoPosition);
//    }

    public void doTuck() {
        close();
        headStraight();
        tuck();
    }

    public float getTilt() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = formatAngle(angles.angleUnit, angles.secondAngle);
        return heading;
    }

    float formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees((float) AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    float formatDegrees(float degrees) {
        return AngleUnit.DEGREES.normalize(degrees);
    }
}
