package org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Robots;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Drivetrains.MecanumDrive;

public class CompBot extends MecanumDrive {



        public HardwareMap hwBot = null;

        public DcMotor viperSlideRight = null;
        public DcMotor viperSlideLeft = null;
        public DcMotor wormgearRight = null;
//        public DcMotor wormgearLeft = null;
        public DcMotor endgameArm = null;
        public Servo endgameArmRotator = null;
        public Servo pixelRotatorRight = null;

        public Servo pixelRotatorLeft = null;

//        public DcMotor pixelRotatorButThisTimeItsAMotor = null;

        public Servo pixelClawLeft = null;
        public Servo pixelClawRight = null;

        public static final double TICKS_PER_ROTATION_WORMGEAR = 384.5;

        public DcMotor planeLauncher = null;

        public Servo planeLauncherServo = null;

        public DistanceSensor pixelDistanceSensor1;
        public DistanceSensor pixelDistanceSensor2;
        public ElapsedTime currentTime = new ElapsedTime();

        public ElapsedTime upTimer = new ElapsedTime();
        public ElapsedTime downTimer = new ElapsedTime();

        RevBlinkinLedDriver blinkinLedDriver;
        RevBlinkinLedDriver.BlinkinPattern pattern;

        RevBlinkinLedDriver blinkinLedDriver2;
        RevBlinkinLedDriver.BlinkinPattern pattern2;


    public IMU imu  = null;
    public double headingError  = 0;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


//        public BNO055IMU imu_old;
//        public Orientation angles;
//        public Acceleration gravity;
//        public final double SPEED = .3;
//        public final double TOLERANCE = .4;

        public CompBot (){}

        public void initRobot(HardwareMap hwMap) {
            hwBot = hwMap;

            frontLeftMotor = hwBot.dcMotor.get("front_left_motor");
            frontRightMotor = hwBot.dcMotor.get("front_right_motor");
            rearLeftMotor = hwBot.dcMotor.get("rear_left_motor");
            rearRightMotor = hwBot.dcMotor.get("rear_right_motor");

            frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
            rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

            //Initialize Motor Run Mode for Robot
            setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            viperSlideRight = hwBot.dcMotor.get("viper_slide_right");
            viperSlideRight.setDirection(DcMotor.Direction.FORWARD);
            viperSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            viperSlideLeft = hwBot.dcMotor.get("viper_slide_left");
            viperSlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            viperSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            wormgearRight = hwBot.dcMotor.get("wormgear_right");
            wormgearRight.setDirection(DcMotor.Direction.FORWARD); //check direction b/f testing
            wormgearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//            pixelRotatorButThisTimeItsAMotor = hwBot.dcMotor.get("pixel_rotator_motor");
//            pixelRotatorButThisTimeItsAMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//            pixelRotatorButThisTimeItsAMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//            wormgearLeft = hwBot.dcMotor.get("wormgear_left");
//            wormgearLeft.setDirection(DcMotor.Direction.FORWARD);  //check direction b/f testing
//            wormgearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



            //Expantion Hub Port 0

            endgameArm = hwBot.dcMotor.get("endgame_arm");
            endgameArm.setDirection(DcMotorSimple.Direction.FORWARD);
            endgameArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            endgameArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            endgameArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            endgameArmRotator = hwBot.servo.get("end_game_arm_rotator");
//            endgameArmRotator.setDirection(Servo.Direction.FORWARD);


            pixelRotatorRight = hwBot.servo.get("pixel_rotator");
            //pixelRotator.setDirection(Servo.Direction.REVERSE);
            pixelRotatorLeft = hwBot.servo.get("pixel_rotator_left");
            pixelRotatorLeft.setDirection(Servo.Direction.REVERSE);

            pixelClawLeft = hwBot.servo.get("pixel_claw_left");
            pixelClawLeft.setDirection(Servo.Direction.FORWARD);

            pixelClawRight = hwBot.servo.get("pixel_claw_right");
            pixelClawRight.setDirection(Servo.Direction.FORWARD);

            planeLauncherServo = hwBot.servo.get("plane_launcher_servo");
            planeLauncherServo.setDirection(Servo.Direction.FORWARD);
//
//            planeLauncher = hwBot.dcMotor.get("plane_launcher");
//            planeLauncher.setDirection(DcMotor.Direction.FORWARD);
//            planeLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            currentTime.reset();


//            BNO055IMU.Parameters parametersimu = new BNO055IMU.Parameters();
//            parametersimu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//            parametersimu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//            parametersimu.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//
//            parametersimu.loggingEnabled = true;
//            parametersimu.loggingTag = "IMU";
//            parametersimu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//            imu = hwBot.get(BNO055IMU.class, "imu");
//            imu.initialize(parametersimu);

            imu = hwBot.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(orientationOnRobot));

            pixelDistanceSensor1 = hwBot.get(DistanceSensor.class, "pixel_distance_1");
            pixelDistanceSensor2 = hwBot.get(DistanceSensor.class, "pixel_distance_2");


            blinkinLedDriver = hwBot.get(RevBlinkinLedDriver.class, "left_light");
            blinkinLedDriver2 = hwBot.get(RevBlinkinLedDriver.class, "right_light");

//            BNO055IMU.Parameters parametersimu = new BNO055IMU.Parameters();
//        parametersimu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parametersimu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parametersimu.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parametersimu.loggingEnabled = true;
//        parametersimu.loggingTag = "IMU";
//        parametersimu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

//        imu_old = hwBot.get(BNO055IMU.class, "imu_old");
//        imu_old.initialize(parametersimu);

        }

//    public void oldGyroCorrection (double speed, double angle) {
//
//        angles = imu_old.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        if (angles.firstAngle >= angle + TOLERANCE) {
//            while (angles.firstAngle >=  angle + TOLERANCE && LinearOp.opModeIsActive()) {
//                rotateRight(speed);
//                angles = imu_old.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            }
//        }
//        else if (angles.firstAngle <= angle - TOLERANCE) {
//            while (angles.firstAngle <= angle - TOLERANCE && LinearOp.opModeIsActive()) {
//                rotateLeft(speed);
//                angles = imu_old.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            }
//        }
//        stopMotors();
//
//        angles = imu_old.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//    }


//    public void oldGyroReset () {
//        BNO055IMU.Parameters parametersimu = new BNO055IMU.Parameters();
//        imu_old.initialize(parametersimu);
//    }





    // *********** Gyro Drive and Gyro Stafing Methods

    /* encoder ticks per rotatons  = 537.7 */

//    public void driveGyro (int encoders, double power) throws InterruptedException {
//        angles = imu_old.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double leftSideSpeed;
//        double rightSideSpeed;
//
//
//        double  target = angles.firstAngle;
//        double startPosition =  frontLeftMotor.getCurrentPosition();
//
//        while (frontLeftMotor.getCurrentPosition() < encoders + startPosition) {
//            double targetAngle = angles.firstAngle;
//
//            leftSideSpeed = power + (angles.firstAngle  - target) / 100;
//            rightSideSpeed = power + (angles.firstAngle - target) / 100;
//
//
//            leftSideSpeed = Range.clip(leftSideSpeed, -1, 1);        // helps prevent out of bounds error
//            rightSideSpeed = Range.clip(rightSideSpeed, -1, 1);
//
//            frontLeftMotor.setPower(leftSideSpeed);
//            rearLeftMotor.setPower(rightSideSpeed);
//
//            frontRightMotor.setPower(rightSideSpeed);
//            frontRightMotor.setPower(rightSideSpeed);
//
//            LinearOp.telemetry.addData("Left Speed",frontLeftMotor.getPower());
//            LinearOp.telemetry.addData("Right Speed", frontRightMotor.getPower());
//            LinearOp.telemetry.addData("Distance till destination ", encoders + startPosition - frontLeftMotor.getCurrentPosition());
//
//            // missing waiting
//            LinearOp.idle();
//        }
//
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        rearLeftMotor.setPower(0);
//        rearRightMotor.setPower(0);
//
//        LinearOp.idle();
//
//    }
//
//    public void driveGyroStraight (int encoders, double power, String direction) throws InterruptedException {
//        angles = imu_old.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double currentPos = 0;
//        double leftSideSpeed;
//        double rightSideSpeed;
//
//
//        double target = angles.firstAngle;
//        double startPosition = frontLeftMotor.getCurrentPosition();
//      //  linearOp.telemetry.addData("Angle to start: ", angles.firstAngle);
//      //  linearOp.telemetry.update();
//        LinearOp.sleep(100);
//        while (currentPos < encoders + startPosition && LinearOp.opModeIsActive()) {
//
//            angles = imu_old.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//
//            currentPos = Math.abs(frontLeftMotor.getCurrentPosition());
//
//            switch (direction) {
//                case "forward":
////                        currentPos = frontLeftMotor.getCurrentPosition();
//                    leftSideSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
//                    rightSideSpeed = power - (angles.firstAngle - target) / 100;
//
//                    leftSideSpeed = Range.clip(leftSideSpeed, -1, 1);        // helps prevent out of bounds error
//                    rightSideSpeed = Range.clip(rightSideSpeed, -1, 1);
//
//                    frontLeftMotor.setPower(leftSideSpeed);
//                    rearLeftMotor.setPower(leftSideSpeed);
//
//                    frontRightMotor.setPower(rightSideSpeed);
//                    rearRightMotor.setPower(rightSideSpeed);
//                    break;
//                case "backward":
////                        currentPos = -frontLeftMotor.getCurrentPosition();
//                    leftSideSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
//                    rightSideSpeed = power + (angles.firstAngle - target) / 100;
//
//                    leftSideSpeed = Range.clip(leftSideSpeed, -1, 1);        // helps prevent out of bounds error
//                    rightSideSpeed = Range.clip(rightSideSpeed, -1, 1);
//
//                    frontLeftMotor.setPower(-leftSideSpeed);
//                    rearLeftMotor.setPower(-leftSideSpeed);
//
//                    frontRightMotor.setPower(-rightSideSpeed);
//                    rearRightMotor.setPower(-rightSideSpeed);
//                    break;
//            }
//
//
///*
//            linearOp.telemetry.addData("Left Speed", frontLeftMotor.getPower());
//            linearOp.telemetry.addData("Right Speed", frontRightMotor.getPower());
//            linearOp.telemetry.addData("Distance till destination ", encoders + startPosition - frontLeftMotor.getCurrentPosition());
//            linearOp.telemetry.addData("Current Position", currentPos);
//            linearOp.telemetry.addData("Target Position", target);
//            linearOp.telemetry.addData("Angle: ", angles.firstAngle);
//            linearOp.telemetry.update();
//            // missing waiting
//*/
//            LinearOp.idle();
//        }
//
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        rearLeftMotor.setPower(0);
//        rearRightMotor.setPower(0);
//
//        LinearOp.idle();
////
//                }
//
//
//    public void driveGyroStrafe (int encoders, double power, String direction) throws InterruptedException {
//        angles = imu_old.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double currentPos = 0;
//        double frontLeftSpeed;
//        double frontRightSpeed;
//        double rearLeftSpeed;
//        double rearRightSpeed;
//
//
//        double target = angles.firstAngle;
//        double startPosition = frontLeftMotor.getCurrentPosition();
//    //    linearOp.telemetry.addData("Angle to start: ", angles.firstAngle);
//    //    linearOp.telemetry.update();
//        LinearOp.sleep(2000);
//        while (currentPos < encoders + startPosition && LinearOp.opModeIsActive()) {
//
//            angles = imu_old.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//
//            currentPos = Math.abs(frontLeftMotor.getCurrentPosition());
//
//            switch (direction) {
//                case "left":
//                    frontLeftSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
//                    frontRightSpeed = power - (angles.firstAngle - target) / 100;
//                    rearLeftSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
//                    rearRightSpeed = power + (angles.firstAngle - target) / 100;
//
//                    frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);        // helps prevent out of bounds error
//                    frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
//                    rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);        // helps prevent out of bounds error
//                    rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);
//
//                    frontLeftMotor.setPower(-frontLeftSpeed);
//                    frontRightMotor.setPower(frontRightSpeed);
//
//                    rearLeftMotor.setPower(rearLeftSpeed);
//                    rearRightMotor.setPower(-rearRightSpeed);
//                    break;
//                case "right":
//                    frontLeftSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
//                    frontRightSpeed = power + (angles.firstAngle - target) / 100;
//                    rearLeftSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
//                    rearRightSpeed = power - (angles.firstAngle - target) / 100;
//
//                    frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);        // helps prevent out of bounds error
//                    frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
//                    rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);        // helps prevent out of bounds error
//                    rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);
//
//                    frontLeftMotor.setPower(frontLeftSpeed);
//                    frontRightMotor.setPower(-frontRightSpeed);
//
//                    rearLeftMotor.setPower(-rearLeftSpeed);
//                    rearRightMotor.setPower(rearRightSpeed);
//                    break;
//            }
//
//
///*
//            linearOp.telemetry.addData("Left Speed", frontLeftMotor.getPower());
//            linearOp.telemetry.addData("Right Speed", frontRightMotor.getPower());
//            linearOp.telemetry.addData("Distance till destination ", encoders + startPosition - frontLeftMotor.getCurrentPosition());
//            linearOp.telemetry.addData("Current Position", currentPos);
//            linearOp.telemetry.addData("Target Position", target);
//            linearOp.telemetry.addData("Angle: ", angles.firstAngle);
//
//            linearOp.telemetry.update();
//*/
//            // missing waiting
//            LinearOp.idle();
//        }
//
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        rearLeftMotor.setPower(0);
//        rearRightMotor.setPower(0);
//
//        LinearOp.idle();
//
//    }
//
//
//
//    public void driveGyroStrafeAngle (int encoders, double power, String direction, double angle) throws InterruptedException {
//        angles = imu_old.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double currentPos = 0;
//        double frontLeftSpeed;
//        double frontRightSpeed;
//        double rearLeftSpeed;
//        double rearRightSpeed;
//
//
//        double target = angle;
//        double startPosition = frontLeftMotor.getCurrentPosition();
//    //    linearOp.telemetry.addData("Angle to start: ", angles.firstAngle);
//    //    linearOp.telemetry.update();
//        LinearOp.sleep(2000);
//        while (currentPos < encoders + startPosition && LinearOp.opModeIsActive()) {
//
//            angles = imu_old.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//
//            currentPos = Math.abs(frontLeftMotor.getCurrentPosition());
//
//            switch (direction) {
//                case "left":
//                    frontLeftSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
//                    frontRightSpeed = power - (angles.firstAngle - target) / 100;
//                    rearLeftSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
//                    rearRightSpeed = power + (angles.firstAngle - target) / 100;
//
//                    frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);        // helps prevent out of bounds error
//                    frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
//                    rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);        // helps prevent out of bounds error
//                    rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);
//
//                    frontLeftMotor.setPower(-frontLeftSpeed);
//                    frontRightMotor.setPower(frontRightSpeed);
//
//                    rearLeftMotor.setPower(rearLeftSpeed);
//                    rearRightMotor.setPower(-rearRightSpeed);
//                    break;
//                case "right":
//                    frontLeftSpeed = power + (angles.firstAngle - target) / 100;            // they need to be different
//                    frontRightSpeed = power + (angles.firstAngle - target) / 100;
//                    rearLeftSpeed = power - (angles.firstAngle - target) / 100;            // they need to be different
//                    rearRightSpeed = power - (angles.firstAngle - target) / 100;
//
//                    frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);        // helps prevent out of bounds error
//                    frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
//                    rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);        // helps prevent out of bounds error
//                    rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);
//
//                    frontLeftMotor.setPower(frontLeftSpeed);
//                    frontRightMotor.setPower(-frontRightSpeed);
//
//                    rearLeftMotor.setPower(-rearLeftSpeed);
//                    rearRightMotor.setPower(rearRightSpeed);
//                    break;
//            }
//
//
///*
//           linearOp.telemetry.addData("Left Speed", frontLeftMotor.getPower());
//           linearOp.telemetry.addData("Right Speed", frontRightMotor.getPower());
//           linearOp.telemetry.addData("Distance till destination ", encoders + startPosition - frontLeftMotor.getCurrentPosition());
//           linearOp.telemetry.addData("Current Position", currentPos);
//           linearOp.telemetry.addData("Target Position", target);
//           linearOp.telemetry.addData("Angle: ", angles.firstAngle);
//
//           linearOp.telemetry.update();
//*/
//            // missing waiting
//            LinearOp.idle();
//        }
//
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        rearLeftMotor.setPower(0);
//        rearRightMotor.setPower(0);
//
//        LinearOp.idle();
//
//    }

//    public void gyroCorrection (double speed, double angle) {
//
//        angles = imu.getAngularOrientation(
//                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        if (angles.firstAngle >= angle + TOLERANCE && LinearOp.opModeIsActive()) {
//            while (angles.firstAngle >=  angle + TOLERANCE && LinearOp.opModeIsActive()) {
//                rotateRight(speed);
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//                LinearOp.telemetry.addData("Current Angle Est: ", angles.firstAngle);
//            }
//        }
//        else if (angles.firstAngle <= angle - TOLERANCE && LinearOp.opModeIsActive()) {
//            while (angles.firstAngle <= angle - TOLERANCE && LinearOp.opModeIsActive()) {
//                rotateLeft(speed);
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//                LinearOp.telemetry.addData("Current Angle Est:" , angles.firstAngle);
//            }
//        }
//        stopMotors();
//
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//    }
//
//
//    public void gyroReset () {
//        BNO055IMU.Parameters parametersimu = new BNO055IMU.Parameters();
//        imu.initialize(parametersimu);
//    }


    public void gyroCorrection(double speed, double targetAngle) {
        imu.resetYaw();
        currentHeading = getHeading();
        if (currentHeading >= targetAngle + headingTolerance && LinearOp.opModeIsActive()) {
            while (currentHeading >= targetAngle + headingTolerance && LinearOp.opModeIsActive()) {
                rotateRight(speed);

                currentHeading = getHeading();
                LinearOp.telemetry.addData("Current Angle: ", currentHeading);
                LinearOp.telemetry.addData("Target Angle: ", targetAngle);
                LinearOp.telemetry.update();
            }
        } else if (currentHeading <= targetAngle - headingTolerance && LinearOp.opModeIsActive()) ;
        {
            while (currentHeading <= targetAngle - headingTolerance && LinearOp.opModeIsActive()) {
                rotateLeft(speed);

                currentHeading = getHeading();
                LinearOp.telemetry.addData("Current Angle: ", currentHeading);
                LinearOp.telemetry.addData("Target Angle: ", targetAngle);
                LinearOp.telemetry.update();
            }
        }

        stopMotors();
        currentHeading = getHeading();
    }


    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }




    public void linearSlideExtend(double power) {
        viperSlideRight.setPower(-Math.abs(power));
        viperSlideLeft.setPower(-Math.abs(power));
    }

    public void linearSlideRetract(double power) {
            viperSlideRight.setPower(Math.abs(power));
            viperSlideLeft.setPower(Math.abs(power));
    }

    public void stopLinearSlide () {
            viperSlideLeft.setPower(0);
            viperSlideRight.setPower(0);

    }

    public void linearSlideExtend(double power, double rotations)  {
        double ticks = rotations * (1) * TICKS_PER_ROTATION;
        viperSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(viperSlideRight.getCurrentPosition()) < ticks && LinearOp.opModeIsActive()) {
            linearSlideExtend(power);
        }
        linearSlideStop();
    }

    public void linearSlideRetract(double power, double rotations) {
        double ticks = rotations * TICKS_PER_ROTATION;
        viperSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(viperSlideRight.getCurrentPosition())< ticks && LinearOp.opModeIsActive()) {
            linearSlideRetract(power);
        }
        linearSlideStop();
    }

    public void rightWormgearUp (double power, double ticks) {
//        double ticks = rotations * TICKS_PER_ROTATION_WORMGEAR;
        wormgearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormgearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(wormgearRight.getCurrentPosition()) < ticks && LinearOp.opModeIsActive()) {
           rightWormgearDown(power);
        }
        rightWormgearStop();
    }

    public void rightWormgearDown(double power, double ticks){
//        double ticks = rotations * TICKS_PER_ROTATION_WORMGEAR;
        wormgearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormgearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(wormgearRight.getCurrentPosition()) < ticks && LinearOp.opModeIsActive()) {
            rightWormgearUp(power);
        }
        rightWormgearStop();
    }

    public void linearSlideStop() {
        viperSlideRight.setPower(0);
    }

    public void rightWormgearUp(double power) {
        wormgearRight.setPower(Math.abs(power));
    }

    public void rightWormgearDown(double power) {
        wormgearRight.setPower(-Math.abs(power));
    }

    public void rightWormgearStop() {wormgearRight.setPower(0);}

    public void endgameArmExtend(){
        endgameArm.setPower(-1);
    }

    public void endgameArmRetract(){
       endgameArm.setPower(1);
    }
    public void endgameArmStop(){
        endgameArm.setPower(0);
    }

    public void endgameArmRotatorMovement (double position) {
            endgameArmRotator.setPosition(position);
    }

    public void rightPixelClawOpen () { pixelClawRight.setPosition(0.948);//378
    }
    public void rightPixelClawClose(){
        pixelClawRight.setPosition(0.478);
    }//848

    public void leftPixelClawOpen (){
        pixelClawLeft.setPosition(0.948);
    }

    public void leftPixelClawClose (){
        pixelClawLeft.setPosition(0.478);
    }

    public void leftPixelLEDNone() {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void leftPixelLEDIn() {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void leftPixelLEDCaptured() {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void rightPixelLEDNone(){
            blinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
    public void rightPixelLEDIn(){
            blinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }
    public void rightPixelLEDCaptured(){
            blinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

//    public void planeLauncherOn(){
//            planeLauncherServo.setPower(1);
//    }
//
//    public void planeLauncherOff(){
//            planeLauncherServo.setPower(0);
//    }

    public void collectorPosition(){
            pixelRotatorRight.setPosition(.437);
            pixelRotatorLeft.setPosition(.437);
    }

    public void drivePosition(){
            pixelRotatorRight.setPosition(.5);
            pixelRotatorLeft.setPosition(.5);
    }

    public void autoPlacePosition() {

            pixelRotatorRight.setPosition(0.7);

    }

    public void automousPosition(){
            pixelRotatorRight.setPosition(.9);
    }

    //Need to determine new hang position based on how hang arm is mounted
    public void hangPosition(){
            pixelRotatorRight.setPosition(.5);
    }

    public void tuckPosition(){
            pixelRotatorRight.setPosition(0.283);
            pixelRotatorLeft.setPosition(0.283);
    }


    //collector position .22
    //drive position .35
    //auto pos .9

}
