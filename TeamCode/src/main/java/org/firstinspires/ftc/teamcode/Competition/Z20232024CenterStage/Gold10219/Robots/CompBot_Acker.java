package org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Robots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Drivetrains.MecanumDrive;


public class CompBot_Acker extends MecanumDrive {

        // Hardware Mapping Variables
        public HardwareMap hwBot = null;

        // Pixel Arm Variables
        public DcMotor viperSlideRight = null;
        public DcMotor wormgearRight = null;
        public Servo pixelClaw = null;

        // End Game Arm Variables
        public DcMotor wormgearLeft = null;
        public DcMotor endgameArm = null;
        public Servo endGameRotatorServo = null;

        //Gyro Variables
        public BNO055IMU imu;
        public Orientation angles;
        public Acceleration gravity;
        public final double SPEED = .3;
        public final double TOLERANCE = .4;

        // Timer Objects
        public ElapsedTime currentTime = new ElapsedTime();
        public ElapsedTime timer = new ElapsedTime();

        // Constructors
        public CompBot_Acker(){}

        // Initialization Method
        public void initRobot(HardwareMap hwMap) {
            hwBot = hwMap;

            // Drivetrain Motors HW Mapping
            frontLeftMotor = hwBot.dcMotor.get("front_left_motor");
            frontRightMotor = hwBot.dcMotor.get("front_right_motor");
            rearLeftMotor = hwBot.dcMotor.get("rear_left_motor");
            rearRightMotor = hwBot.dcMotor.get("rear_right_motor");

            frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
            rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

            setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Pixel Arm Extension Mechanism HW Mapping
            viperSlideRight = hwBot.dcMotor.get("viper_slide_right");
            viperSlideRight.setDirection(DcMotor.Direction.FORWARD);
            viperSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Pixel Arm Rotational Mechanism HW Mapping
            wormgearRight = hwBot.dcMotor.get("wormgear_right");
            wormgearRight.setDirection(DcMotor.Direction.FORWARD); //check direction b/f testing
            wormgearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //Pixel Claw Mechanism HW Mapping
            pixelClaw = hwBot.servo.get("pixel_claw");
            endGameRotatorServo.setDirection(Servo.Direction.FORWARD);

            // End Game Rotational Mechanism (Servo and Motor) HW Mapping
            endGameRotatorServo = hwBot.servo.get("end_game_rotate_servo");
            endGameRotatorServo.setDirection(Servo.Direction.FORWARD);

            wormgearLeft = hwBot.dcMotor.get("end_game_rotator");
            wormgearLeft.setDirection(DcMotor.Direction.FORWARD);  //check direction b/f testing
            wormgearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // End Game Arm Lifting HW Mapping
            endgameArm = hwBot.dcMotor.get("end_game_arm");
            endgameArm.setDirection(DcMotorSimple.Direction.FORWARD);
            endgameArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            endgameArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            endgameArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Reset Time During Init
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


        }

        // Why is this here?  This should be in drivetrain class?  Remove
        public void stopMotors(){
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rearLeftMotor.setPower(0);
            rearRightMotor.setPower(0);
        }

        // ********** Pixel Arm Extension, Retraction, and Stoppage Methods **********
        public void linearSlideUp (double power) {

            viperSlideRight.setPower(-Math.abs(power));
        }

        public void linearSlideDown (double power) {
            viperSlideRight.setPower(Math.abs(power));
        }

        public void linearSlideUp (double power, double rotations)  {
            double ticks = rotations * (1) * TICKS_PER_ROTATION;
            viperSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            viperSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (Math.abs(viperSlideRight.getCurrentPosition()) < ticks && LinearOp.opModeIsActive()) {
                  linearSlideUp(power);
            }
            linearSlideStop();
        }
        public void linearSlideDown (double power, double rotations) {
            double ticks = rotations * TICKS_PER_ROTATION;
            viperSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            viperSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (Math.abs(viperSlideRight.getCurrentPosition())< ticks && LinearOp.opModeIsActive()) {
                linearSlideDown(power);
            }
            linearSlideStop();
        }

        public void linearSlideStop() {
            viperSlideRight.setPower(0);
        }

        // **********  Pixel Arm Rotating Methods  ************

        public void rotatePixelArmUp(double power) {
            wormgearRight.setPower(Math.abs(power));
        }

        public void rotatePixelArmDown(double power) {
            wormgearRight.setPower(-Math.abs(power));
        }

        public void stopPixelArmRotation() {
            wormgearRight.setPower(0);
        }

        // ********** End Game Arm Rotating Methods **********
        public void rotateEndGameArmUp(double position) {
            endGameRotatorServo.setPosition(position);
        }

        public void rotateEndGameArmDown (double position) {
          endGameRotatorServo.setPosition(position);
        }

        // ********** End Game Arm Extension and Lifting Methods **********
        public void endgameArmExtend (){
            endgameArm.setPower(100);
        }

        public void endgameArmRetract(){
            endgameArm.setPower(-100);
        }

        public void endgameArmStop(){
            endgameArm.setPower(0);
        }

}
