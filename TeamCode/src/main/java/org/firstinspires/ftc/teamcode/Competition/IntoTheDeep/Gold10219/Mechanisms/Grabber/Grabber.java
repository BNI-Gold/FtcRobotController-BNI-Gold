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

    double grabberOpen = .6644;
    double grabberClosed = .9972;

    private Orientation angles;
    public float heading = 0;

    public double straight = .4683;
    public double right = .7528;
    public double left = .2144;

    public double down = 0;
    public double up = 0.55;

    public double grabberAdjust = .001;
    public double rotationAdjust = .001;
    public double tiltAdjust = .001;

    public double outAngle = 45;
    public double downAngle = -45;
    public double finishHookAngle = 0;
    public double tuckAngle = -70;

    public double angleDeadband = 2;
    public double servoDeadband = .005;

    public Grabber() {}

    public void initGrabber(HardwareMap hwMap) {
        hwBot = hwMap;

        grabber = hwBot.servo.get("grabber");
        tilt = hwBot.servo.get("grabber_tilt");
        rotate = hwBot.servo.get("grabber_rotate");

        grabber.setDirection(Servo.Direction.FORWARD);
        tilt.setDirection(Servo.Direction.FORWARD);
        rotate.setDirection(Servo.Direction.REVERSE);

        imu = hwBot.get(BNO055IMU.class, "bnoimu");
        initializeIMU();
    }

    private enum imuCheckStates {
        RUNNING,
        INITIALIZE,
        INITIALIZING
    }

    private imuCheckStates imuCheckState = imuCheckStates.RUNNING;

    public boolean isImuRunning() {
        return imu.getSystemStatus() == BNO055IMU.SystemStatus.RUNNING_FUSION;
    }

    public void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    public void imuGyroCheck() {
        switch (imuCheckState) {
            case RUNNING:
                if (imu.getSystemStatus() != BNO055IMU.SystemStatus.RUNNING_FUSION) {
                    imuCheckState = imuCheckStates.INITIALIZE;
                }
                break;
            case INITIALIZE:
                initializeIMU();
                imuCheckState = imuCheckStates.INITIALIZING;
                break;
            case INITIALIZING:
                if (imu.getSystemStatus() == BNO055IMU.SystemStatus.RUNNING_FUSION || imu.getSystemStatus() == BNO055IMU.SystemStatus.IDLE) {
                    imuCheckState = imuCheckStates.RUNNING;
                }
                break;
        }
    }

    public void release() {
        grabber.setPosition(grabberOpen);
    }

    public void grab() {
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
        tilt.setPosition(Math.min(position + tiltAdjust, up)); // Ensure position does not exceed 1.0
    }

    public void tiltDown() {
        double position = tilt.getPosition();
        tilt.setPosition(Math.max(position - tiltAdjust, down)); // Ensure position does not go below 0.0
    }

    public void tiltUp(double mult) {
        double position = tilt.getPosition();
        tilt.setPosition(Math.min(position + (tiltAdjust * 7 * mult), up)); // Ensure position does not exceed 1.0
    }

    public void tiltDown(double mult) {
        double position = tilt.getPosition();
        tilt.setPosition(Math.max(position - (tiltAdjust * 7 * mult), down)); // Ensure position does not go below 0.0
    }

    public double ang = 0;
    public double diff = 0;
    public double diff1 = 0;
    public double pch = 0;
    public double csp = 0;
    public double nsp = 0;
    public double nsp2 = 0;

    public enum grabberStates {
        OUT, DOWN, HOOK, TUCK, MANUAL
    }

    public enum tiltStates {
        SETTLED, CALL_TILT, TILTING
    }

    public grabberStates grabberState = grabberStates.MANUAL;

    private tiltStates tiltState = tiltStates.SETTLED;

    public void setGrabberState(grabberStates state) {
        grabberState = state;
    }

    public tiltStates getTiltState() {
        return tiltState;
    }

    public boolean isSettled() {
        return tiltState == tiltStates.SETTLED;
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
                double dampingFactor = .2;

                switch (grabberState) {
                    case OUT:
                        desiredAngle = outAngle;
                        break;
                    case DOWN:
                        desiredAngle = downAngle;
                        break;
                    case HOOK:
                        desiredAngle = finishHookAngle;
                        break;
                    case TUCK:
                        desiredAngle = tuckAngle;
                        break;
                }

                if (!isImuRunning()) break;

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
                double positionChange = (angleDifference / 300.0) * dampingFactor;
                pch = positionChange;

                double currentServoPosition = tilt.getPosition();
                csp = currentServoPosition;

                double newServoPosition = currentServoPosition + positionChange;
                nsp = newServoPosition;

                // Clamp the servo position to the valid range [0.4, 1]
                newServoPosition = Range.clip(newServoPosition, down, up);
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

    public void doTuck() {
        grab();
        headStraight();
        setGrabberState(grabberStates.TUCK);
    }

    public float getTilt() {
        try {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = formatAngle(angles.angleUnit, angles.secondAngle);
            return heading;
        } catch (Exception e) {
            imuGyroCheck();
            return 0;
        }
    }

    float formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees((float) AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    float formatDegrees(float degrees) {
        return AngleUnit.DEGREES.normalize(degrees);
    }
}
