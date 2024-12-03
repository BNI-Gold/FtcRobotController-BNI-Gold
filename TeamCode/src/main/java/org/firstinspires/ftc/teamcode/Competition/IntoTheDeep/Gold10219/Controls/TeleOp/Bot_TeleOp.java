package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

    Grabber grabber = new Grabber();

    PrimaryArm arm = new PrimaryArm();

    ElapsedTime timer = new ElapsedTime();

    public void init() {
        Bot.initRobot(hardwareMap);
        arm.initPrimaryArm(hardwareMap, Bot.LinearOp);
        grabber.initGrabber(hardwareMap);
        grabber.doTuck();
    }

    public void loop() {
        speedControl();
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

    public void drive() {
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
    }

    public void grabberControl() {
        if (gamepad2.a) grabber.close();
        else if (gamepad2.b) grabber.open();

        if (gamepad2.x) grabber.setGrabberState(Grabber.grabberStates.DOWN);
        else if (gamepad2.y) grabber.setGrabberState(Grabber.grabberStates.OUT);


        if (gamepad2.dpad_left && !(gamepad2.dpad_down || gamepad2.dpad_up)) grabber.headLeft();
        else if (gamepad2.dpad_right && !(gamepad2.dpad_down || gamepad2.dpad_up)) grabber.headRight();
        else if (gamepad2.back) grabber.headStraight();
        else if (Math.abs(gamepad2.right_stick_x) > .35 || Math.abs(gamepad2.right_stick_y) > .35) grabber.rotate(gamepad2.right_stick_x, gamepad2.right_stick_y);
    }

    public void primaryArmControl() {
        if (gamepad2.right_bumper) arm.setExtend();
        else if (gamepad2.left_bumper) arm.setRetract();

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

        telemetry.addLine();

//        double primaryArmPower = arm.arm.getPower();
//        if (primaryArmPower > 0) telemetry.addData("Primary Arm Extending: ", primaryArmPower);
//        else if (primaryArmPower < 0) telemetry.addData("Primary Arm Retracting: ", primaryArmPower);
//        else telemetry.addLine("Primary Arm Stopped");

//        telemetry.addLine();
//
//        double primaryArmRotatorPower = arm.rotator.getPower();
//        if (primaryArmRotatorPower > 0) telemetry.addData("Primary Arm Going Up: ", primaryArmRotatorPower);
//        else if (primaryArmRotatorPower < 0) telemetry.addData("Primary Arm Going Down: ", primaryArmRotatorPower);
//        else telemetry.addLine("Primary Arm Not Rotating");
//
//        telemetry.addLine();
//
//        double intakePower = intake.intake.getPower();
//        if (intakePower > 0) telemetry.addData("Intake Running Inwards: ", intakePower);
//        else if (intakePower < 0) telemetry.addData("Intake Running Outwards: ", intakePower);
//        else telemetry.addLine("Intake Stopped");

//        telemetry.addLine();
//
//        telemetry.addData("Intake Rotation Position: ", intake.rotator.getPosition());

        telemetry.update();
    }

}
