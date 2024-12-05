package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

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
    double armSpeedMultiplier = 1;

    public CompBot Bot = new CompBot();

    IMU imu = null;

    Grabber grabber = new Grabber();

    PrimaryArm arm = new PrimaryArm();

    ElapsedTime timer = new ElapsedTime();

    public void init() {
        Bot.initRobot(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        arm.initPrimaryArm(hardwareMap, Bot.LinearOp);
        grabber.initGrabber(hardwareMap);
        grabber.doTuck();
    }

    public void loop() {
        speedControl();
        driverProfileSwitcher();
        drive();
        primaryArmControl();
        grabberControl();
        grabber.tiltStateCheck();
        telemetryOutput();
    }

    public void speedControl() {
        if (gamepad1.left_trigger > 0.35) {
            speedMultiply = 0.3;
        } else {
            speedMultiply = 1;
        }

        if (gamepad2.dpad_left) {
            armSpeedMultiplier = 0.3;
        } else {
            armSpeedMultiplier = 1;
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
                Bot.rearLeftMotor.setPower(backRightPower);
                break;
        }
    }

    public void grabberControl() {
        if (gamepad2.a) grabber.close();
        else if (gamepad2.b) grabber.open();

        if (gamepad2.x) grabber.setGrabberState(Grabber.grabberStates.DOWN);
        else if (gamepad2.y) grabber.setGrabberState(Grabber.grabberStates.OUT);
        else if (gamepad2.start) grabber.doTuck();
        else if (gamepad2.left_stick_button) grabber.setGrabberState(Grabber.grabberStates.FHOOK);


        if (gamepad2.dpad_left && !(gamepad2.dpad_down || gamepad2.dpad_up)) grabber.headLeft();
        else if (gamepad2.dpad_right && !(gamepad2.dpad_down || gamepad2.dpad_up))
            grabber.headRight();
        else if (gamepad2.back) grabber.headStraight();
        else if (Math.abs(gamepad2.right_stick_x) > .35 || Math.abs(gamepad2.right_stick_y) > .35)
            grabber.rotate(gamepad2.right_stick_x, gamepad2.right_stick_y);
    }

    public void primaryArmControl() {
        if (gamepad2.right_bumper) arm.setExtend();
        else if (gamepad2.right_trigger > 0.35) arm.extend(gamepad2.right_trigger * 4);
        else if (gamepad2.left_bumper) arm.setRetract();
        else if (gamepad2.left_trigger > 0.35) arm.retract(gamepad2.left_trigger * 4);

        if (gamepad2.dpad_up && gamepad2.right_bumper) {
            arm.up(true);
        } else if (gamepad2.dpad_up) arm.up(false);
        else if (gamepad2.dpad_down && gamepad2.right_bumper) arm.down(true);
        else if (gamepad2.dpad_down) arm.down(false);
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
