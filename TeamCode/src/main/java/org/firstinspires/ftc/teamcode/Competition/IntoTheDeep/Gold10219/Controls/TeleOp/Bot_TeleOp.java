package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake.IntakeDirections;
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

    PrimaryArm arm = new PrimaryArm();

    Intake intake = new Intake();

    ElapsedTime timer = new ElapsedTime();

    public void init() {
        Bot.initRobot(hardwareMap);
        arm.initPrimaryArm(hardwareMap, Bot.LinearOp);
        intake.initIntake(hardwareMap);
        intake.center();
    }

    public void loop() {
        speedControl();
        drive();
        primaryArmControl();
        intakeControl();
        intake.stateCheck();
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

    public void intakeControl() {
        if (gamepad2.a && gamepad2.b) {
            intake.start(IntakeDirections.IN);
        }
        else if (gamepad2.a) {
            intake.intakeUntilSample();
        }
        else if (gamepad2.x && gamepad2.b) {
            intake.start(IntakeDirections.OUT);
        }
        else if (gamepad2.x) {
            intake.dropSample();
        }
        else if (gamepad2.b) intake.stop();


        if (gamepad2.right_bumper && !(gamepad2.dpad_up || gamepad2.dpad_down)) intake.rotateRight();
        else if (gamepad2.left_bumper) intake.rotateLeft();
        else if (gamepad2.y) intake.center();
    }

    public void primaryArmControl() {
        //Multiply triggers by speed multiplier
        double rightSpeed = gamepad2.right_trigger * armSpeedMultiplier;
        double leftSpeed = gamepad2.left_trigger * armSpeedMultiplier;

        if (gamepad2.right_trigger > 0.35) arm.extend(rightSpeed);
        else if (gamepad2.left_trigger > 0.35) arm.retract(leftSpeed);
        else arm.stop();

        if (gamepad2.dpad_up && gamepad2.right_bumper) arm.up(true);
        else if (gamepad2.dpad_up) arm.up(false);
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

        double primaryArmPower = arm.arm.getPower();
        if (primaryArmPower > 0) telemetry.addData("Primary Arm Extending: ", primaryArmPower);
        else if (primaryArmPower < 0) telemetry.addData("Primary Arm Retracting: ", primaryArmPower);
        else telemetry.addLine("Primary Arm Stopped");

        telemetry.addLine();

        double primaryArmRotatorPower = arm.rotator.getPower();
        if (primaryArmRotatorPower > 0) telemetry.addData("Primary Arm Going Up: ", primaryArmRotatorPower);
        else if (primaryArmRotatorPower < 0) telemetry.addData("Primary Arm Going Down: ", primaryArmRotatorPower);
        else telemetry.addLine("Primary Arm Not Rotating");

        telemetry.addLine();

        double intakePower = intake.intake.getPower();
        if (intakePower > 0) telemetry.addData("Intake Running Inwards: ", intakePower);
        else if (intakePower < 0) telemetry.addData("Intake Running Outwards: ", intakePower);
        else telemetry.addLine("Intake Stopped");

        telemetry.addLine();

        telemetry.addData("Intake Rotation Position: ", intake.rotator.getPosition());

        telemetry.update();
    }

}
