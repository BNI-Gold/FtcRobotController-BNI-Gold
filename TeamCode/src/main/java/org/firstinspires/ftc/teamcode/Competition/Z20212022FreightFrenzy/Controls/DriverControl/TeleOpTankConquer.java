package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.DriverControl;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.TankBot;
@Disabled
@TeleOp( name = "Tank Go Vroom", group = "1")

public class TeleOpTankConquer extends OpMode {

    public TankBot Bot = new TankBot();

    private float leftStickY1;
    private float rightStickY1;
    private float leftStickX1;
    private float rightStickX1;

    private float leftStickY2;
    private float leftStickX2;
    private float rightStickY2;
    private float rightStickX2;

    public float dpad_left;
    public float dpad_right;
    public float dpad_down;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double powerThreshold = 0;
    double speedMultiply = 1;

    public double leftMotorValue;
    public double rightMotorValue;

    boolean tankDrive = true;
//    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        Bot.stopMotors();

        leftStickY1 = 0;
        leftStickX1 = 0;
        rightStickY1 = 0;
        rightStickX1 = 0;

        leftStickY2 = 0;
        leftStickX2 = 0;
        rightStickY2 = 0;
        rightStickX2 = 0;
    }

    @Override
    public void loop() {
        getController();
        if (tankDrive == true) {
            tankDrive();
        }
        else {
//            tankDrive();
             arcadeDrive();
        }

        DuckSpinner();

        Intakecontroller();

        LyftExtender();

        BoxHolderControl();

        driveSpeed();

        telemtryOutput();

    }

    public void driveSpeed () {
        if (gamepad1.dpad_up) {
            speedMultiply = 1;
        }

        if (gamepad1.dpad_down) {
            speedMultiply = 0.5;
        }
    }
    public void getController () {
        leftStickY1 = -gamepad1.left_stick_y;
        leftStickX1 = -gamepad1.left_stick_x;
        rightStickY1 = -gamepad1.right_stick_y;
        rightStickX1 = -gamepad1.right_stick_x;

        leftStickY2 = -gamepad2.left_stick_y;
        leftStickX2 = -gamepad2.left_stick_x;
        rightStickY2 = -gamepad2.right_stick_y;
        rightStickX2 = -gamepad2.right_stick_x;

        if (gamepad1.b) {
            tankDrive = true;
        }
        else if (gamepad1.x) {
            tankDrive = false;
        }
    }



    public void tankDrive () {
        Bot.leftMotorA.setPower(leftStickY1 * speedMultiply);
        Bot.leftMotorB.setPower(leftStickY1 * speedMultiply);
        Bot.rightMotorA.setPower(rightStickY1 * speedMultiply);
        Bot.rightMotorB.setPower(rightStickY1 * speedMultiply);
    }



    public void arcadeDrive () {
        leftMotorValue = leftStickY1 - leftStickX1;
        rightMotorValue = leftStickY1 + leftStickX1;
        leftMotorValue = Range.clip(leftMotorValue,-1, 1);
        rightMotorValue = Range.clip(rightMotorValue, -1, 1);
        Bot.leftMotorA.setPower(leftMotorValue * speedMultiply);
        Bot.leftMotorB.setPower(leftMotorValue * speedMultiply);
        Bot.rightMotorA.setPower(rightMotorValue * speedMultiply);
        Bot.rightMotorB.setPower(rightMotorValue * speedMultiply);
    }



    public void DuckSpinner () {
        if (gamepad2.dpad_left == true) {
            Bot.duckspincounterclockwise();
        }
        else if (gamepad2.dpad_right == true) {
            Bot.duckspinclockwise();
        }
        else {
            Bot.duckspinstop();
        }
    }

    public void Intakecontroller () {
        if (leftStickY2 > 0.1){
            Bot.Intake(leftStickY2);
        }
        else if (leftStickY2 < -0.1){
            Bot.Intake(leftStickY2);
        }
        else {
            Bot.Intake (0);
        }

    }

    public void LyftExtender () {
        if (gamepad2.right_bumper == true) {
            Bot.LyftExtend();
            Bot.pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            Bot.blinkinLedDriver.setPattern(Bot.pattern);
        }
        else if (gamepad2.left_bumper == true){
            Bot.LyftRetract();
        }
        else {
            Bot.LyftStopMotors();
        }
    }


    public void BoxHolderControl () {
        if (gamepad2.a == true){
            Bot.setBoxHolder_Down();

        }
        if (gamepad2.y == true){
            Bot.setBoxHolder_Up();
            Bot.pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            Bot.blinkinLedDriver.setPattern(Bot.pattern);
        }
        if (gamepad2.dpad_down) {
            Bot.setBoxHolder_Release();
            Bot.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            Bot.blinkinLedDriver.setPattern(Bot.pattern);
        }

    }




    public void telemtryOutput () {
        telemetry.addData("Tank Drive? (x/b to swap) ", tankDrive);

//        telemetry.addData("gp1 left stick: ", gamepad1.left_stick_y);
//        telemetry.addData("gp1 right stick: ", gamepad1.right_stick_y);

//        telemetry.addData("left stick value Y 1: ", leftStickY1);
//        telemetry.addData("right stick value Y 1: ", rightStickY1);
telemetry.addData("left motor ticks ", Bot.leftMotorA.getCurrentPosition());
telemetry.addData("right motor ticks", Bot.rightMotorA.getCurrentPosition());

        telemetry.addData("power output Left A:",Bot.leftMotorA.getPower());
        telemetry.addData("power output Left B:",Bot.leftMotorB.getPower());
        telemetry.addData("power output Right A:",Bot.rightMotorA.getPower());
        telemetry.addData("power output Left B:",Bot.rightMotorB.getPower());
    }
}
