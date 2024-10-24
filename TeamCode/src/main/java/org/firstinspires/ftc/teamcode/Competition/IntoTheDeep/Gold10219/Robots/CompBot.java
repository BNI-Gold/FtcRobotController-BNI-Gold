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

public class CompBot extends MecanumDrive {

    public HardwareMap hwBot = null;

    //Mechanism motors and servos
    public Servo primaryExtender = null;
    public Servo secondaryExtender = null;
    public Servo claw = null;

    //Set hub directions
    //TODO: confirm directions
    //Why do we have this? `orientationOnRobot` is never referenced...
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    public CompBot() {}

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
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
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
        primaryExtender = hwBot.servo.get("primary_extender");
        secondaryExtender = hwBot.servo.get("secondary_extender");
        claw = hwBot.servo.get("claw");

        //Assign direction to servos
        //TODO: confirm directions
        primaryExtender.setDirection(Servo.Direction.REVERSE);
        secondaryExtender.setDirection(Servo.Direction.FORWARD);
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
    //Values for peripheral positions
    //TODO: determine these positions (use three below methods to figure out what they're for lol)
    double primaryExtenderExtend = 1;
    double primaryExtenderDrive = .7;
    double primaryExtenderRetract = .3;
    double primaryExtenderPastChassis = .6644;

    double secondaryExtenderExtend = 1;
    double secondaryExtenderHover = .9267;
    double secondaryExtenderRetract = .81;
    double secondaryExtenderJump = .001;

    double clawOpen = .1989;
    double clawClose = .4717;


    //These first three methods *technically* can be used directly from
    //controller button presses, however I'd prefer to use the methods
    //defined after these first three.
    public void usePrimaryExtender(extenderDirections direction) {
        switch (direction) {
            case EXTEND:
                //I'd prefer to use these methods as a button press (not hold)
                //to actuate to correct position each time, but another option
                //is to allow for continuous operation based on button hold

                primaryExtender.setPosition(primaryExtenderExtend);
                break;
            case DRIVE:
                //to robot as possible

                //This should be a button press to actuate to correct position
                //each time.

                primaryExtender.setPosition(primaryExtenderDrive);
                break;
            case RETRACT:
                //I'd prefer to use these methods as a button press (not hold)
                //to actuate to correct position each time, but another option
                //is to allow for continuous operation based on button hold

                primaryExtender.setPosition(primaryExtenderRetract);
                break;
        }
    }
    public void useSecondaryExtender(extenderDirections direction) {
        switch (direction) {
            case EXTEND:
                //I'd prefer to use these methods as a button press (not hold)
                //to actuate to correct position each time, but another option
                //is to allow for continuous operation based on button hold

                //Should only extend if primary extender is past robot chassis
                if (primaryExtender.getPosition() >= primaryExtenderPastChassis) {
                    secondaryExtender.setPosition(secondaryExtenderExtend);
                }
                break;
            case HOVER:
                //I'd prefer to use these methods as a button press (not hold)
                //to actuate to correct position each time, but another option
                //is to allow for continuous operation based on button hold

                if (primaryExtender.getPosition() >= primaryExtenderPastChassis) {
                    secondaryExtender.setPosition(secondaryExtenderHover);
                }
                break;
            case RETRACT:
                //I'd prefer to use these methods as a button press (not hold)
                //to actuate to correct position each time, but another option
                //is to allow for continuous operation based on button hold

                secondaryExtender.setPosition(secondaryExtenderRetract);
                break;
        }
    }
    public void useSecondaryExtender(boolean use, extenderDirections direction) {
        double position = secondaryExtender.getPosition();
        if (use) {
            switch (direction) {
                case EXTEND:
                    secondaryExtender.setPosition(position + secondaryExtenderJump);
                    break;
                case RETRACT:
                    secondaryExtender.setPosition(position - secondaryExtenderJump);
                    break;
            }
        } else useSecondaryExtender(direction);
    }
    public void useClaw(clawOptions option) {
        switch(option) {
            case OPEN:
                //This should be a button press to actuate to correct position
                //each time.

                claw.setPosition(clawOpen);
                break;
            case CLOSE:
                //This should be a button press to actuate to correct position
                //each time.

                claw.setPosition(clawClose);
                break;
        }
    }

    //Sets extenders and claw to optimal driving position for when
    //a sample is being held
    public void driveWithSample() {
        //Claw is closed, then primary
        //extender is retracted as close to robot as possible while
        //sample is being held (see comments in usePrimaryExtender
        //method), then secondary extender is retracted completely.
        useClaw(clawOptions.CLOSE);
        useSecondaryExtender(extenderDirections.RETRACT);
        usePrimaryExtender(extenderDirections.DRIVE);
    }

    //Retracts all peripherals
    public void retractAll() {
        useClaw(clawOptions.CLOSE);
        useSecondaryExtender(extenderDirections.RETRACT);
        usePrimaryExtender(extenderDirections.RETRACT);
    }

    //Extends primary arm, retracts secondary arm, opens claw
    public void prepCollection() {
        useClaw(clawOptions.OPEN);
        useSecondaryExtender(extenderDirections.RETRACT);
        usePrimaryExtender(extenderDirections.EXTEND);
    }

    //Extends secondary arm to sample then closes claw.
    //Calls prepCollection in case primary isn't already extended
    public void hover() {
        prepCollection();
        useSecondaryExtender(extenderDirections.HOVER);
    }
}
