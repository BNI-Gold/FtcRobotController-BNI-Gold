//This is the primary bot, hence it only being named 'Bot'. All other bots should use a name
//referencing what is special about them, i.e. AckerBot or ProgrammingBot or StrafeBot

package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Drivetrains.MecanumDrive;

public class C1CompBot extends MecanumDrive {

    public HardwareMap hwBot = null;

    //Mechanism motors and servos
    public CRServo primaryExtender = null;
    public CRServo secondaryExtender = null;
    public Servo claw = null;

    //Set hub directions
    //TODO: confirm directions
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    public C1CompBot() {
    }

    public void initRobot(HardwareMap hwMap) {
        hwBot = hwMap;

        //********** MOTOR CONFIG **********
        //Get motor ports & info from control hub config
        frontLeftMotor = hwBot.dcMotor.get("front_left_motor");
        frontRightMotor = hwBot.dcMotor.get("front_right_motor");
        rearLeftMotor = hwBot.dcMotor.get("rear_left_motor");
        rearRightMotor = hwBot.dcMotor.get("rear_right_motor");

        //Assign direction to motors
        //TODO: confirm directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        //Initialize motor run mode
        //TODO: Ensure this is how we want it (or don't use at all?)
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //****************************************


        //********** SERVO CONFIG **********
        //Get servo ports & info from control hub config
        primaryExtender = hwBot.crservo.get("primary_extender");
        //primaryExtender = hwBot.servo.get("primary_extender");
        secondaryExtender = hwBot.crservo.get("secondary_extender");
        //secondaryExtender = hwBot.servo.get("secondary_extender");
        claw = hwBot.servo.get("claw");

        //Assign direction to servos
        //TODO: confirm directions
        primaryExtender.setDirection(CRServo.Direction.FORWARD);
        //primaryExtender.setDirection(Servo.Direction.FORWARD);
        secondaryExtender.setDirection(CRServo.Direction.FORWARD);
        //secondaryExtender.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
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


    //********** PERIPHERAL METHODS **********
    //Still need the claw pos vars bc claw servo is not continuous and is servo-servo
    double clawOpen = 0;
    double clawClose = 0;

    //Sets claw to correct position
    public void useClaw(clawOptions option) {
        switch(option) {
            case OPEN:
                //This should be a button press to actuate to correct position
                //each time.

                //claw.setPosition(clawOpen);
                break;
            case CLOSE:
                //This should be a button press to actuate to correct position
                //each time.

                //claw.setPosition(clawClose);
                break;
        }
    }
}
