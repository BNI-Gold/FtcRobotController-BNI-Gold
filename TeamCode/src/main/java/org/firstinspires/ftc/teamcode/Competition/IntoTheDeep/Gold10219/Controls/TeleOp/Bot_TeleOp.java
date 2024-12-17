package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.PoseHelper;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Vision;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;

@TeleOp(name = "A - Into the Deep", group = "competition")
public class Bot_TeleOp extends OpMode {

    double leftStickXVal;
    double leftStickYVal;
    double rightStickXVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double powerThreshold = 0;
    double speedMultiply = 1;
    boolean armSpeedMultiplier = false;

    private final Vision vision = new Vision();
    private final Pinpoint pinpoint = new Pinpoint();
    private final PoseHelper pose = new PoseHelper();

    public CompBot Bot = new CompBot();

    IMU imu = null;

    Grabber grabber = new Grabber();

    PrimaryArm arm = new PrimaryArm();

    private Pose startPose;

    private Follower follower;

    ElapsedTime timer = new ElapsedTime();

    public void init() {
        Bot.initRobot(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        telemetry.addLine("IMU Initialized");

        timer.reset();

//        pinpoint.setOp(this);
//        pinpoint.initPinpoint(hardwareMap);
//
//        telemetry.addLine("Pinpoint Initialized");
//
//        vision.setOp(this);
//        vision.initVision(hardwareMap, pinpoint);
//
//        telemetry.addLine("Vision Initialized");
//
//        pose.setOp(this);
//        pose.setDevices(vision, pinpoint);
//
//        telemetry.addLine("Pose Initiated");
//
//        vision.start();
//
//        telemetry.addLine("Vision Started");
//
//        pose.updateLLUsage(false);
//
//        Pose2D currentPose = pose.getSmartPose(PoseHelper.Alliances.BLUE);
//        telemetry.addData("Pose X: ", currentPose.getX(DistanceUnit.INCH));
//        telemetry.addData("Pose Y: ", currentPose.getY(DistanceUnit.INCH));
//        telemetry.addData("Pose H: ", currentPose.getHeading(AngleUnit.DEGREES));
//
//        startPose = new Pose(
//                currentPose.getX(DistanceUnit.INCH),
//                currentPose.getY(DistanceUnit.INCH),
//                currentPose.getHeading(AngleUnit.RADIANS)
//        );
//
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        telemetry.addLine("Follower Initialized");

        arm.initPrimaryArm(hardwareMap, Bot.LinearOp);
        grabber.initGrabber(hardwareMap);

        telemetry.update();
    }

    public void start() {
        grabber.grab();
        grabber.headStraight();
        arm.setRetract();
    }

    public Pose getCurrentPose() {
        pose.updateHeading();
        pose.syncPose();
        pose.updatePose();
        Pose2D currentPose = pose.getPose();

        return new Pose(
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.RADIANS)
        );
    }

    public void loop() {
//        pose.updatePose();
//        follower.update();
        speedControl();
        driverProfileSwitcher();
        drive();
        shortcutChecker();
        shortcuts();
        grabberControl();
        primaryArmControl();
        grabber.tiltStateCheck();
        telemetryOutput();
    }

    public void speedControl() {
        if (gamepad1.left_trigger > 0.35) {
            speedMultiply = 0.3;
        } else {
            speedMultiply = 1;
        }

        if (gamepad2.left_stick_button) {
            armSpeedMultiplier = true;
        } else {
            armSpeedMultiplier = false;
        }
    }

    private enum driverProfiles {
        CLASSIC, FIELD_CENTRIC
    }

    private driverProfiles driverProfile = driverProfiles.CLASSIC;

    public void driverProfileSwitcher() {
        if (gamepad1.a) driverProfile = driverProfiles.CLASSIC;
        else if (gamepad1.b) driverProfile = driverProfiles.FIELD_CENTRIC;
    }

    public void drive() {
        switch (driverProfile) {
            case CLASSIC:
                leftStickXVal = gamepad1.left_stick_x;
                leftStickXVal = Range.clip(leftStickXVal, -1, 1);

                leftStickYVal = -gamepad1.left_stick_y;
                leftStickYVal = Range.clip(leftStickYVal, -1, 1);

                rightStickXVal = gamepad1.right_stick_x;
                rightStickXVal = Range.clip(rightStickXVal, -1, 1);

                frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
                frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

                frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
                frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

                rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
                rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

                rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
                rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

                if (frontLeftSpeed <= powerThreshold && frontLeftSpeed >= -powerThreshold) {
                    frontLeftSpeed = 0;
                    Bot.frontLeftMotor.setPower(frontLeftSpeed);
                } else {
                    Bot.frontLeftMotor.setPower(frontLeftSpeed * speedMultiply);
                }

                if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold) {
                    frontRightSpeed = 0;
                    Bot.frontRightMotor.setPower(frontRightSpeed);
                } else {
                    Bot.frontRightMotor.setPower(frontRightSpeed * speedMultiply);
                }

                if (rearLeftSpeed <= powerThreshold && rearLeftSpeed >= -powerThreshold) {
                    rearLeftSpeed = 0;
                    Bot.rearLeftMotor.setPower(rearLeftSpeed);
                } else {
                    Bot.rearLeftMotor.setPower(rearLeftSpeed * speedMultiply);
                }

                if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold) {
                    rearRightSpeed = 0;
                    Bot.rearRightMotor.setPower(rearRightSpeed);
                } else {
                    Bot.rearRightMotor.setPower(rearRightSpeed * speedMultiply);
                }
                break;
            case FIELD_CENTRIC:
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                Bot.frontLeftMotor.setPower(frontLeftPower);
                Bot.rearLeftMotor.setPower(backLeftPower);
                Bot.frontRightMotor.setPower(frontRightPower);
                Bot.rearRightMotor.setPower(backRightPower);
                break;
        }
    }

    private shortcutCases shortcutCase = shortcutCases.NONE;

    private enum shortcutCases {
        NONE, GRAB_SPECIMEN, HOOK_SPECIMEN, RETRACT_FROM_CHAMBER
    }

    private grabSpecimenCases grabSpecimenCase = grabSpecimenCases.CLOSE;

    private enum grabSpecimenCases {
        CLOSE,
        TIMEOUT1,
        UP,
        TIMEOUT2,
        TUCK,
        RETRACT
    }

    private hookSpecimenCases hookSpecimenCase = hookSpecimenCases.HOOK;

    private enum hookSpecimenCases {
        HOOK,
        TIMEOUT1,
        DOWN
    }

    private retractFromChamberCases retractFromChamberCase = retractFromChamberCases.OPEN;

    private enum retractFromChamberCases {
        OPEN,
        TIMEOUT1,
        UP,
        TIMEOUT2,
        RETRACT,
        TUCK
    }

    public void shortcutChecker() {
        switch (shortcutCase) {
            case GRAB_SPECIMEN:
                grabSpecimen();
                break;
            case HOOK_SPECIMEN:
                hookSpecimen();
                break;
            case RETRACT_FROM_CHAMBER:
                retractFromChamber();
                break;
        }
    }

    public void grabSpecimen() {
        switch (grabSpecimenCase) {
            case CLOSE:
                grabber.grab();
                timer.reset();
                grabSpecimenCase = grabSpecimenCases.TIMEOUT1;
                break;
            case TIMEOUT1:
                if (timer.time() > .5) {
                    grabSpecimenCase = grabSpecimenCases.UP;
                }
                break;
            case UP:
                arm.up(2, false);
                timer.reset();
                grabSpecimenCase = grabSpecimenCases.TIMEOUT2;
                break;
            case TIMEOUT2:
                if (timer.time() > .5) {
                    grabSpecimenCase = grabSpecimenCases.TUCK;
                }
                break;
            case TUCK:
                grabber.doTuck();
                grabSpecimenCase = grabSpecimenCases.RETRACT;
                break;
            case RETRACT:
                arm.setRetract();
                shortcutCase = shortcutCases.NONE;
                grabSpecimenCase = grabSpecimenCases.CLOSE;
                break;
        }
    }

    public void hookSpecimen() {
        switch (hookSpecimenCase) {
            case HOOK:
                grabber.setGrabberState(Grabber.grabberStates.HOOK);
                timer.reset();
                hookSpecimenCase = hookSpecimenCases.TIMEOUT1;
                break;
            case TIMEOUT1:
                if (timer.time() > .5) {
                    hookSpecimenCase = hookSpecimenCases.DOWN;
                }
                break;
            case DOWN:
                arm.down(.3, false);
                shortcutCase = shortcutCases.NONE;
                hookSpecimenCase = hookSpecimenCases.HOOK;
                break;
        }
    }

    public void retractFromChamber() {
        switch (retractFromChamberCase) {
            case OPEN:
                grabber.release();
                timer.reset();
                retractFromChamberCase = retractFromChamberCases.TIMEOUT1;
                break;
            case TIMEOUT1:
                if (timer.time() > .5) {
                    retractFromChamberCase = retractFromChamberCases.UP;
                }
                break;
            case UP:
                arm.up(.5, false);
                timer.reset();
                retractFromChamberCase = retractFromChamberCases.TIMEOUT2;
                break;
            case TIMEOUT2:
                if (timer.time() > .5) {
                    retractFromChamberCase = retractFromChamberCases.RETRACT;
                }
                break;
            case RETRACT:
                arm.setRetract();
                retractFromChamberCase = retractFromChamberCases.TUCK;
                break;
            case TUCK:
                grabber.doTuck();
                shortcutCase = shortcutCases.NONE;
                retractFromChamberCase = retractFromChamberCases.OPEN;
                break;
        }
    }

    private boolean clipped = false;
    private boolean dPressed = false;

    public void shortcuts() {
        //Press gp2.a when grabber is aligned with specimen on wall.
        //This will grab specimen, lift arm slightly, tuck grabber, and retract arm.
        if (gamepad2.dpad_up) {
            shortcutCase = shortcutCases.GRAB_SPECIMEN;
        }

        //Press gp2.b when specimen is aligned with chamber, or after specimen has been hooked on chamber.
        else if (gamepad2.dpad_down) {
            //This will open the grabber, slightly raise the arm, retract the arm, and tuck grabber.
            if (clipped && !dPressed) {
                dPressed = true;
                shortcutCase = shortcutCases.RETRACT_FROM_CHAMBER;
                clipped = false;
            }

            //This will set the grabber to the hook position and lower arm slightly.
            else if (!dPressed) {
                dPressed = true;
                shortcutCase = shortcutCases.HOOK_SPECIMEN;
                clipped = true;
            }
        }
        else if (gamepad2.dpad_left) {
            clipped = false;
        }
        else {
            dPressed = false;
        }
    }

    public void grabberControl() {
        if (gamepad2.a) grabber.grab();
        else if (gamepad2.b) grabber.release();

        if (gamepad2.x) grabber.setGrabberState(Grabber.grabberStates.DOWN);
        else if (gamepad2.y) grabber.setGrabberState(Grabber.grabberStates.OUT);
        else if (gamepad2.dpad_right) grabber.doTuck();
        else if (gamepad2.right_stick_button) grabber.setGrabberState(Grabber.grabberStates.HOOK);

        else if (gamepad2.back) grabber.headStraight();
        else if (Math.abs(gamepad2.right_stick_x) > .35 || Math.abs(gamepad2.right_stick_y) > .35)
            grabber.rotate(gamepad2.right_stick_x, gamepad2.right_stick_y);
    }

    public void primaryArmControl() {
        if (gamepad2.right_bumper) arm.setExtend();
        else if (gamepad2.right_trigger > 0.35) arm.extend(gamepad2.right_trigger * 4);
        else if (gamepad2.left_bumper) arm.setRetract();
        else if (gamepad2.left_trigger > 0.35) arm.retract(gamepad2.left_trigger * 4);

        if (gamepad2.start) {
            grabber.setGrabberState(Grabber.grabberStates.MANUAL);
        }

        if (gamepad2.left_stick_y < -.35 && !gamepad2.start) arm.up(armSpeedMultiplier);
        else if (gamepad2.left_stick_y > .35 && !gamepad2.start) arm.down(armSpeedMultiplier);
        else if (gamepad2.left_stick_y < -.35 && gamepad2.start) {
            grabber.tiltUp(Math.abs(gamepad2.left_stick_y));
            arm.stopRotation();
        }
        else if (gamepad2.left_stick_y > -.35 && gamepad2.start) {
            grabber.tiltDown(Math.abs(gamepad2.left_stick_y));
            arm.stopRotation();
        }
        else arm.stopRotation();
    }

    public void telemetryOutput() {
        telemetry.addData("Front Left: ", Bot.frontLeftMotor.getCurrentPosition());
        telemetry.addData("Front Right: ", Bot.frontRightMotor.getCurrentPosition());
        telemetry.addData("Rear Left: ", Bot.rearLeftMotor.getCurrentPosition());
        telemetry.addData("Rear Right: ", Bot.rearRightMotor.getCurrentPosition());

        telemetry.update();
    }

}
