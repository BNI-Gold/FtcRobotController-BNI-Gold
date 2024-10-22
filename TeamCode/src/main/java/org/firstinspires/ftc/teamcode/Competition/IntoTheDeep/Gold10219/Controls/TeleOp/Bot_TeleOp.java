package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

@Disabled
@TeleOp(name = "A - Into the Deep")
public class Bot_TeleOp extends OpMode {

    double leftStickXVal;
    double leftStickYVal;
    double rightStickXVal;
    double rightStickYVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double powerThreshold = 0;
    double speedMultiply = 1;

    public boolean slowMode = false;

    public CompBot Bot = new CompBot();

    ElapsedTime timer = new ElapsedTime();

    public void init() {
        Bot.initRobot(hardwareMap);
    }

    public void loop() {
        speedControl();
        drive();
        clawMechanismsControl();
        telemetryOutput();
    }

    public void speedControl() {

        if (gamepad1.left_trigger > 0.35) {

            slowMode = true;

        } else {

            slowMode = false;

        }

        if (slowMode) {

            speedMultiply = 0.3;

        } else {

            speedMultiply = 1;

        }
    }

    public void drive() {
        leftStickXVal = gamepad1.left_stick_x;
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
        //Using negative value here because for some stupid reason up is negative
        leftStickYVal = -gamepad1.left_stick_y;
        leftStickYVal = Range.clip(leftStickYVal, -1, 1);

        rightStickXVal = gamepad1.right_stick_x;
        rightStickXVal = Range.clip(rightStickXVal, -1, 1);
        //Using negative value here because for some stupid reason up is negative
        rightStickYVal = -gamepad1.right_stick_y;
        rightStickYVal = Range.clip(rightStickYVal, -1, 1);

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

    public void clawMechanismsControl() {
        if (gamepad2.x) Bot.prepCollection(); //Should be left button
        else if (gamepad2.a) Bot.collectSample(); //Should be bottom button
        else if (gamepad2.y) Bot.driveWithSample(); //Should be top button
        else if (gamepad2.b) Bot.retractAll(); //Should be right button
    }

    public void telemetryOutput() {
        telemetry.addData("Front Left: ", Bot.frontLeftMotor.getCurrentPosition());
        telemetry.addData("Front Right: ", Bot.frontRightMotor.getCurrentPosition());
        telemetry.addData("Rear Left: ", Bot.rearLeftMotor.getCurrentPosition());
        telemetry.addData("Rear Right: ", Bot.rearRightMotor.getCurrentPosition());

        telemetry.update();
    }

}
