//This is the primary bot, hence it only being named 'Bot'. All other bots should use a name
//referencing what is special about them, i.e. AckerBot or ProgrammingBot or StrafeBot

package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Drivetrains.MecanumDrive;

public class CompBot extends MecanumDrive {

    public HardwareMap hwBot = null;

    //Mechanism motors and servos
    public DcMotor primaryArmRotator = null;
    public DcMotor primaryArm = null;
    public CRServo intake = null;
    public Servo intakeRotator = null;
    public ColorRangeSensor sampleSensor1 = null;
    public boolean isCollecting = false;
    public boolean isDropping = false;

    //Set hub directions
    //TODO: confirm directions
    //Why do we have this? `orientationOnRobot` is never referenced...
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    public CompBot() {}

    public void initRobot(HardwareMap hwMap) {
        hwBot = hwMap;

        //********** DRIVETRAIN CONFIG **********
        //Get motor ports & info from control hub config
        frontLeftMotor = hwBot.dcMotor.get("front_left_motor");
        frontRightMotor = hwBot.dcMotor.get("front_right_motor");
        rearLeftMotor = hwBot.dcMotor.get("rear_left_motor");
        rearRightMotor = hwBot.dcMotor.get("rear_right_motor");

        //Assign direction to motors
        //TODO: confirm directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        //Initialize motor run mode
        //TODO: Ensure this is how we want it (or don't use at all?)
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set motor zero power behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //****************************************


        //********** MECHANISM MOTOR & SERVO CONFIG **********
        //Get servo ports & info from control hub config
        intake = hwBot.crservo.get("intake");
        intakeRotator = hwBot.servo.get("intake_rotator");

        //Get motor ports & info from control hub config
        primaryArmRotator = hwBot.dcMotor.get("primary_arm_rotator");
        primaryArm = hwBot.dcMotor.get("viper_slide");

        //Assign direction to servos
        //TODO: confirm directions
        intake.setDirection(CRServo.Direction.REVERSE);
        intakeRotator.setDirection(Servo.Direction.FORWARD);
        
        //Assign direction to motors
        //TODO: confirm directions
        primaryArmRotator.setDirection(DcMotor.Direction.FORWARD);
        primaryArm.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set motor zero power behavior
        primaryArmRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        primaryArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //****************************************

        //********** SENSOR CONFIG **********
        sampleSensor1 = hwBot.get(ColorRangeSensor.class, "sample_sensor");
        //****************************************
    }

    //********** GYRO METHODS **********
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
    //****************************************


}
