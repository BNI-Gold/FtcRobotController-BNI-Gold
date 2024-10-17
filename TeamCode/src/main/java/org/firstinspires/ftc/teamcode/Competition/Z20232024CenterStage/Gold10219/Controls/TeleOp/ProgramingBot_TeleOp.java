package org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Controls.TeleOp;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Robots.ProgrammingBot;

@Disabled
@TeleOp(name = "A - RANGER RATTLE PROGRAMMING BOT")

public class ProgramingBot_TeleOp extends OpMode {



   // FtcDashboard dashboard = FtcDashboard.getInstance();
   // Telemetry dashboardTelemetry = dashboard.getTelemetry();
    double leftStickYVal;
    double leftStickXVal;
    double rightStickXVal;
    double rightStickYVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double powerThreshold = 0;
    double speedMultiply = 1;

    public boolean slowMode = false;

    public boolean variableSlowMode = false;

    public ProgrammingBot Bot = new ProgrammingBot();

    public void init () {
        Bot.initRobot(hardwareMap);
    }
    public void init_loop() {  }

    public void start() {

    }

    public void loop(){
        speedControl();
        drive();
        telemetryOutput();
    }


    public void speedControl() {

        if (gamepad1.left_trigger > 0.35) {

            slowMode = true;

        } else {

            slowMode = false;

        }

        if (gamepad1.right_trigger > 0.1) {

            variableSlowMode = true;

        } else {

            variableSlowMode = false;

        }

        if (slowMode) {

            speedMultiply = 0.3;

        } else if (variableSlowMode) {

            //experimental
            speedMultiply = gamepad1.right_trigger / 0.8;

        } else {

            speedMultiply = 1;

        }

    }

    public void drive() {

        leftStickYVal = gamepad1.left_stick_y;
        leftStickYVal = Range.clip(leftStickYVal, -1, 1);
        leftStickXVal = -gamepad1.left_stick_x;
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
        rightStickXVal = -gamepad1.right_stick_x;
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

        if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold){
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

        if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold){
            rearRightSpeed = 0;
            Bot.rearRightMotor.setPower(rearRightSpeed);
        } else {
            Bot.rearRightMotor.setPower(rearRightSpeed * speedMultiply);
        }
    }


    public void telemetryOutput() {

        telemetry.addData("Front Left: ", Bot.frontLeftMotor.getCurrentPosition());
        telemetry.addData("Front Right: ", Bot.frontRightMotor.getCurrentPosition());
        telemetry.addData("Rear Left: ", Bot.rearLeftMotor.getCurrentPosition());
        telemetry.addData("Rear Right: ", Bot.rearRightMotor.getCurrentPosition());

//        dashboardTelemetry.addData("FRONT LEFT: ", Bot.frontLeftMotor.getPower());
//        dashboardTelemetry.addData("FRONT RIGHT: ", Bot.frontRightMotor.getPower());
//
//        dashboardTelemetry.addData("REAR LEFT: ", Bot.rearLeftMotor.getPower());
//        dashboardTelemetry.addData("REAR RIGHT: ", Bot.rearRightMotor.getPower());
//        dashboardTelemetry.update();
        telemetry.update();
    }

}
