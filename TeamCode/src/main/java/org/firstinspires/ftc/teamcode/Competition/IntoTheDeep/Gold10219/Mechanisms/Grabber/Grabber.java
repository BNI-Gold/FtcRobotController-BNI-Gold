package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Grabber;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    public void tiltToAngle(double desiredAngle) {
        double currentAngle = getTilt(); // Get current tilt angle
        double difference = desiredAngle - currentAngle;

        // Deadband to prevent unnecessary oscillation
        if (Math.abs(difference) < 5) {
            return; // Do nothing if within the deadband
        }

        // Normalize the difference to fit within [-150, 150] range
        if (difference > 150) {
            difference -= 300;
        } else if (difference < -150) {
            difference += 300;
        }

        // Calculate the new servo position
        double currentServoPosition = tilt.getPosition();
        double positionChange = difference / 300.0; // Map the angle difference to servo range
        double newServoPosition = currentServoPosition + positionChange;

        // Clamp the new position to [0.0, 1.0]
        newServoPosition = Math.max(0.0, Math.min(1.0, newServoPosition));

        // Set the servo to the new position
        tilt.setPosition(newServoPosition);
    }

    public double tiltTo = 0;

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
