package org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.Robots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.DriveTrains.MecanumDrive;

public class CompetionBot extends MecanumDrive {

    public HardwareMap hwBot = null;


    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    public final double SPEED = 0.3;
    public final double TOLERANCE = 0.4;


    public DcMotorEx grabberLiftOne = null;
    public DcMotorEx grabberLiftTwo = null;
    public DcMotorEx turretPlatform = null;
    public Servo grabberArmServo = null;
    public CRServo bigConeSucker = null;
    TouchSensor magSwitch;

    public double LiftEncoderAvg = 0;
    public double stallPower = 0.25;


    public void initRobot(HardwareMap hardwareMap) {
        hwBot = hardwareMap;

        grabberArmServo = hwBot.get(Servo.class, "grabber_arm");
        grabberArmServo.setDirection(Servo.Direction.FORWARD);

        bigConeSucker = hwBot.get(CRServo.class, "cone_sucker");
        bigConeSucker.setDirection(CRServo.Direction.FORWARD);

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        rearLeftMotor = hardwareMap.get(DcMotorEx.class, "rear_left_motor");
        rearRightMotor = hardwareMap.get(DcMotorEx.class, "rear_right_motor");

//            frontLeftMotor=hwBot.dcMotor.get("front_left_motor");
//            frontRightMotor=hwBot.dcMotor.get("front_right_motor");
//            rearLeftMotor=hwBot.dcMotor.get("rear_left_motor");
//            rearRightMotor=hwBot.dcMotor.get("rear_right_motor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//            frontLeftMotor.setPositionPIDFCoefficients(1);
//            frontRightMotor.setPositionPIDFCoefficients(1);
//            rearRightMotor.setPositionPIDFCoefficients(1);
//            rearLeftMotor.setPositionPIDFCoefficients(1);
//
//            frontLeftMotor.setVelocityPIDFCoefficients(0.8,0.4,1,1);
//            frontRightMotor.setVelocityPIDFCoefficients(0.8,0.4,1,1);
//            rearRightMotor.setVelocityPIDFCoefficients(0.8,0.4,1,1);
//            rearLeftMotor.setVelocityPIDFCoefficients(0.8,0.4,1,1);


        BNO055IMU.Parameters parametersimu = new BNO055IMU.Parameters();
        parametersimu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersimu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersimu.calibrationDataFile = "BN0055IMUCalibration.json";

        parametersimu.loggingEnabled = true;
        parametersimu.loggingTag = "IMU";
        parametersimu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwBot.get(BNO055IMU.class, "imu");
        imu.initialize(parametersimu);

//            grabberLift = hwBot.dcMotor.get("grabber_lift");
//            grabberLift.setDirection(DcMotor.Direction.FORWARD);
//            grabberLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            grabberLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            grabberLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        grabberLiftOne = hardwareMap.get(DcMotorEx.class, "grabber_lift_one");
        grabberLiftOne.setDirection(DcMotorSimple.Direction.REVERSE);
        grabberLiftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabberLiftOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabberLiftOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        grabberLiftOne.setPositionPIDFCoefficients(1);

        grabberLiftTwo = hardwareMap.get(DcMotorEx.class, "grabber_lift_two");
        grabberLiftTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        grabberLiftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabberLiftTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabberLiftTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        grabberLiftTwo.setPositionPIDFCoefficients(1);


        turretPlatform = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretPlatform.setDirection(DcMotorSimple.Direction.REVERSE);
        turretPlatform.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretPlatform.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretPlatform.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Grabber Lift Sensor Hardware Mapping & Configuration
        magSwitch = hardwareMap.get(TouchSensor.class, "magnetic_switch");

        // Grabber Arms Servo Hardware Mapping/Configuration

    }

    public void coneOuttake() {

        bigConeSucker.setDirection(CRServo.Direction.REVERSE);
        bigConeSucker.setPower(1);

    }

    public void coneIntake() {

        bigConeSucker.setDirection(CRServo.Direction.FORWARD);
        bigConeSucker.setPower(1);

    }

    public void intakeStop() {

        bigConeSucker.setPower(0);

    }

    public void openGrabberArms() {

        //was 0.875 for fully open but camera was seeing servo pole.
        grabberArmServo.setPosition(0.84);

    }

    public void closeGrabberArms() {

        grabberArmServo.setPosition(.68);

    }

    public void intakeGrabberArms() {

        grabberArmServo.setPosition(.72);

    }

    public void extendGrabberLift(double power) {

        grabberLiftOne.setPower(Math.abs(power));
        grabberLiftTwo.setPower(Math.abs(power));
    }

    public void retractGrabberLift(double power) {

        grabberLiftOne.setPower(-Math.abs(power));
        grabberLiftTwo.setPower(-Math.abs(power));
    }

    public void stopGrabberLift() {
        grabberLiftOne.setPower(0);
        grabberLiftTwo.setPower(0);
    }

    public void turretClockwise(double power) {

        turretPlatform.setPower(Math.abs(power));

    }

    public void autoTurretClockwise(double power, double rotations) {
        turretPlatform.setPower(Math.abs(power));
    }

    public void turretCounterClockwise(double power) {
        turretPlatform.setPower(-Math.abs(power));

    }

    public void autoTurretCounterClockwise(double power, double rotations) {
        turretPlatform.setPower(-Math.abs(power));

    }

    public void turretStop() {
        turretPlatform.setPower(0);
    }

    public void gyroCorrection(double speed, double angle) {

        angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (angles.firstAngle >= angle + TOLERANCE && linearOp.opModeIsActive()) {
            while (angles.firstAngle >= angle + TOLERANCE && linearOp.opModeIsActive()) {
                rotateRight(speed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                linearOp.telemetry.addData("Current Angle: ", angles.firstAngle);
            }
        } else if (angles.firstAngle <= angle - TOLERANCE && linearOp.opModeIsActive()) ;
        {
            while (angles.firstAngle <= angle - TOLERANCE && linearOp.opModeIsActive()) {
                rotateLeft(speed);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                linearOp.telemetry.addData("Current Angle ", angles.firstAngle);
            }
        }

        stopMotors();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    // TESTING GYRODRIVE

    public void driveBackwardGyro(double power, double rotations) throws InterruptedException {
        double ticks = rotations * (1) * TICKS_PER_ROTATION;

        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentPos = 0;
        double leftSideSpeed;
        double rightSideSpeed;


        double target = angles.firstAngle;
        double startPosition = frontLeftMotor.getCurrentPosition();
        linearOp.telemetry.addData("Angle to start: ", angles.firstAngle);
        linearOp.telemetry.update();
        linearOp.sleep(100);
//        while (currentPos < ticks + startPosition && linearOp.opModeIsActive()) {
        while (currentPos < ticks + startPosition && linearOp.opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            currentPos = Math.abs(frontLeftMotor.getCurrentPosition());


            leftSideSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
            rightSideSpeed = power + (angles.firstAngle - target) / 100;

            leftSideSpeed = Range.clip(leftSideSpeed, -1, 1);        // helps prevent out of bounds error
            rightSideSpeed = Range.clip(rightSideSpeed, -1, 1);

            frontLeftMotor.setPower(-leftSideSpeed);
            rearLeftMotor.setPower(-leftSideSpeed);

            frontRightMotor.setPower(-rightSideSpeed);
            rearRightMotor.setPower(-rightSideSpeed);

            linearOp.idle();

        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        linearOp.idle();

    }

    public void driveForwardGyro(double power, double rotations) throws InterruptedException {

        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double ticks = rotations * (1) * TICKS_PER_ROTATION;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentPos = 0;
        double leftSideSpeed;
        double rightSideSpeed;


        double target = angles.firstAngle;
        double startPosition = frontLeftMotor.getCurrentPosition();
        //  linearOp.telemetry.addData("Angle to start: ", angles.firstAngle);
        //  linearOp.telemetry.update();
        linearOp.sleep(100);
        while (currentPos < ticks + startPosition && linearOp.opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            currentPos = Math.abs(frontLeftMotor.getCurrentPosition());

            currentPos = frontLeftMotor.getCurrentPosition();
            leftSideSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
            rightSideSpeed = power - (angles.firstAngle - target) / 100;

            leftSideSpeed = Range.clip(leftSideSpeed, -1, 1);        // helps prevent out of bounds error
            rightSideSpeed = Range.clip(rightSideSpeed, -1, 1);

            frontLeftMotor.setPower(leftSideSpeed);
            rearLeftMotor.setPower(leftSideSpeed);

            frontRightMotor.setPower(rightSideSpeed);
            rearRightMotor.setPower(rightSideSpeed);

            linearOp.idle();

        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        linearOp.idle();

    }

    public void driveGyroStrafe(double power, double rotations, String direction) throws InterruptedException {
        double ticks = 0;
        ticks = rotations * TICKS_PER_ROTATION;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentPos = 0;
        double frontLeftSpeed;
        double frontRightSpeed;
        double rearLeftSpeed;
        double rearRightSpeed;


        double target = angles.firstAngle;
        double startPosition = frontLeftMotor.getCurrentPosition();
        linearOp.telemetry.addData("Angle to start: ", angles.firstAngle);
        linearOp.telemetry.update();
        linearOp.sleep(2000);
        while (currentPos < ticks + startPosition && linearOp.opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            currentPos = Math.abs(frontLeftMotor.getCurrentPosition());

            switch (direction) {
                case "left":
                    frontLeftSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
                    frontRightSpeed = power - (angles.firstAngle - target) / 100;
                    rearLeftSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
                    rearRightSpeed = power + (angles.firstAngle - target) / 100;

                    frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
                    rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

                    frontLeftMotor.setPower(-frontLeftSpeed);
                    frontRightMotor.setPower(frontRightSpeed);

                    rearLeftMotor.setPower(rearLeftSpeed);
                    rearRightMotor.setPower(-rearRightSpeed);
                    break;
                case "right":
                    frontLeftSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
                    frontRightSpeed = power + (angles.firstAngle - target) / 100;
                    rearLeftSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
                    rearRightSpeed = power - (angles.firstAngle - target) / 100;

                    frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
                    rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

                    frontLeftMotor.setPower(frontLeftSpeed);
                    frontRightMotor.setPower(-frontRightSpeed);

                    rearLeftMotor.setPower(-rearLeftSpeed);
                    rearRightMotor.setPower(rearRightSpeed);
                    break;
            }

            linearOp.idle();

        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        linearOp.idle();

    }

    public void driveGyroStrafeAngle(double power, double rotations, String direction, double angle) throws InterruptedException {
        double ticks = 0;
        ticks = rotations * TICKS_PER_ROTATION;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentPos = 0;
        double frontLeftSpeed;
        double frontRightSpeed;
        double rearLeftSpeed;
        double rearRightSpeed;


        double target = angle;
        double startPosition = frontLeftMotor.getCurrentPosition();
        linearOp.telemetry.addData("Angle to start: ", angles.firstAngle);
        linearOp.telemetry.update();
        linearOp.sleep(2000);
        while (currentPos < ticks + startPosition && linearOp.opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            currentPos = Math.abs(frontLeftMotor.getCurrentPosition());

            switch (direction) {
                case "left":
                    frontLeftSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
                    frontRightSpeed = power - (angles.firstAngle - target) / 100;
                    rearLeftSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
                    rearRightSpeed = power + (angles.firstAngle - target) / 100;

                    frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
                    rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

                    frontLeftMotor.setPower(-frontLeftSpeed);
                    frontRightMotor.setPower(frontRightSpeed);

                    rearLeftMotor.setPower(rearLeftSpeed);
                    rearRightMotor.setPower(-rearRightSpeed);
                    break;
                case "right":
                    frontLeftSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
                    frontRightSpeed = power + (angles.firstAngle - target) / 100;
                    rearLeftSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
                    rearRightSpeed = power - (angles.firstAngle - target) / 100;

                    frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
                    rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);        // helps prevent out of bounds error
                    rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

                    frontLeftMotor.setPower(frontLeftSpeed);
                    frontRightMotor.setPower(-frontRightSpeed);

                    rearLeftMotor.setPower(-rearLeftSpeed);
                    rearRightMotor.setPower(rearRightSpeed);
                    break;
            }


        }
    }

    int turretClockwise = 415;
    int turretCounterclocwise = -415;

    boolean turretEncoderCW = false;
    boolean turretEncoderCCW = false;
    boolean turretEncoderCollect = false;
    boolean turrentEncoder180 = false;

    double turretPowerEncoder = 0.2;

    public void turretAuto90CW () {

        turretEncoderCW = true;

    }

    public void turretAuto90CCW () {

        turretEncoderCCW = true;

    }

    public void turretAutoCollect () {

        turretEncoderCollect = true;

    }

    public void turretControlEncoder() {
        int currentTurretEncoder = turretPlatform.getCurrentPosition();

        //
        // ARE WE USING ENCODER TURN?
        //

        //
        //  MANUAL CONTROL OF TURRET
        //

//        if (gamepad2.left_bumper) {
//            Bot.turretPlatform.setPower(-turretPowerManual);
//        }
//        else if (gamepad2.right_bumper) {
//            Bot.turretPlatform.setPower(+turretPowerManual);
//        }


        //
        //  ENCODER CONTROL OF TURRET
        //

        if (turretEncoderCW) {
            if (turretPlatform.getCurrentPosition() >= turretClockwise) {
//

                if (turretPlatform.getCurrentPosition() >= turretClockwise) {
                    turretPlatform.setPower(-turretPowerEncoder);
                }

                if (turretPlatform.getCurrentPosition() < turretClockwise) {
                    turretEncoderCW = false;
                    turretPlatform.setPower(0);
                }


            } else if (turretPlatform.getCurrentPosition() < turretClockwise) {
//
                if (turretPlatform.getCurrentPosition() < turretClockwise) {
                    turretPlatform.setPower(+turretPowerEncoder);
                }

                if (turretPlatform.getCurrentPosition() > turretClockwise) {
                    turretEncoderCW = false;
                    turretPlatform.setPower(0);
                }


            }

        } else if (turretEncoderCCW) {
            if (turretPlatform.getCurrentPosition() >= turretCounterclocwise) {
//

                if (turretPlatform.getCurrentPosition() >= turretCounterclocwise) {
                    turretPlatform.setPower(-turretPowerEncoder);
                }

                if (turretPlatform.getCurrentPosition() < turretCounterclocwise) {
                    turretEncoderCCW = false;
                    turretPlatform.setPower(0);
                }


            } else if (turretPlatform.getCurrentPosition() < turretCounterclocwise) {
//
                if (turretPlatform.getCurrentPosition() < turretCounterclocwise) {
                    turretPlatform.setPower(+turretPowerEncoder);
                }

                if (turretPlatform.getCurrentPosition() > turretCounterclocwise) {
                    turretEncoderCCW = false;
                    turretPlatform.setPower(0);
                }

            }

        } else if (turretEncoderCollect) {
            if (turretPlatform.getCurrentPosition() >= 0) {
//

                if (turretPlatform.getCurrentPosition() >= 0) {
                    turretPlatform.setPower(-turretPowerEncoder);
                }

                if (turretPlatform.getCurrentPosition() < 0) {
                    turretEncoderCollect = false;
                    turretPlatform.setPower(0);
                }


            } else if (turretPlatform.getCurrentPosition() < 0) {
//
                if (turretPlatform.getCurrentPosition() < 0) {
                    turretPlatform.setPower(+turretPowerEncoder);
                }

                if (turretPlatform.getCurrentPosition() > 0) {
                    turretEncoderCollect = false;
                    turretPlatform.setPower(0);
                }


            }

        } else if (turrentEncoder180 == true) {
            if (turretPlatform.getCurrentPosition() >= 0) {

                if (turretPlatform.getCurrentPosition() < turretClockwise * 2) {
                    turretPlatform.setPower(+turretPowerEncoder);
                }
            } else if (turretPlatform.getCurrentPosition() < 0) {
                if (turretPlatform.getCurrentPosition() > turretCounterclocwise * 2) {
                    turretPlatform.setPower(-turretPowerEncoder);
                } else {
                    turrentEncoder180 = false;
                }
            }
        } else {
            turretPlatform.setPower(0);
            turrentEncoder180 = false;
        }
    }

}