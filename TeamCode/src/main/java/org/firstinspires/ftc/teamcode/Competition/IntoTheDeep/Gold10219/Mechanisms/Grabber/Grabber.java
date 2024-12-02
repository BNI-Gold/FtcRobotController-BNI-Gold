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

    public void tiltUp() {
        double position = tilt.getPosition();
        tilt.setPosition(position + tiltAdjust);
    }

    public void tiltDown() {
        double position = tilt.getPosition();
        tilt.setPosition(position - tiltAdjust);
    }

    public enum tiltStates {
        RUNNING, STOP
    }

    private void toAngle(double angle) {
        double heading = getHeading();

        if (heading - angle > 2) {
            tiltDown();
        } else if (heading - angle < -2) {
            tiltUp();
        }
    }

    public tiltStates tiltState = tiltStates.STOP;
    public double tiltTo = 0;

    public void tiltStateCheck() {
        if (tiltState == tiltStates.RUNNING) {
            toAngle(tiltTo);
        } else {
            tiltState = tiltStates.STOP;
        }
    }

    public void tiltToAngle(double angle) {
        tiltTo = angle;
        tiltState = tiltStates.RUNNING;
    }

    public void doTuck() {
        close();
        headStraight();
        tuck();
    }

    public float getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = formatAngle(angles.angleUnit, angles.firstAngle);
        return heading;
    }

    float formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees((float) AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    float formatDegrees(float degrees) {
        return AngleUnit.DEGREES.normalize(degrees);
    }
}
