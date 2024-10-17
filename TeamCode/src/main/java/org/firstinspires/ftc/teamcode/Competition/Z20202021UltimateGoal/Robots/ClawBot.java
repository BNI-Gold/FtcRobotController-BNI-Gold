package org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.DriveTrains.MecanumDrive;

public class ClawBot extends MecanumDrive {
    public HardwareMap hwBot = null;
    public Orientation angles;
    public Acceleration gravity;
    public final double SPEED = .3;
    public final double TOLERANCE = .4;
    public DcMotor SideToSide = null;
    public DcMotor BackAndForth = null;
    public DcMotor UpAndDown = null;
    public Servo Claw = null;

    public ClawBot() {

    }

    public void initRobot(HardwareMap hardwareMap, String startPosition, String mode) {
        hwBot = hardwareMap;
//        frontLeftMotor = hwBot.dcMotor.get("front_left_motor");
//        frontRightMotor = hwBot.dcMotor.get("front_right_motor");
//        rearLeftMotor = hwBot.dcMotor.get("rear_left_motor");
//        rearRightMotor = hwBot.dcMotor.get("rear_right_motor");
//
//        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
//        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
//
//        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BackAndForth = hwBot.dcMotor.get("Back_And_Forth");
        SideToSide = hwBot.dcMotor.get("Side_To_Side");
        UpAndDown = hwBot.dcMotor.get("Up_And_Down");
        Claw = hwBot.get(Servo.class, "claw");
        BackAndForth.setDirection(DcMotorSimple.Direction.FORWARD);
        SideToSide.setDirection(DcMotorSimple.Direction.FORWARD);
        UpAndDown.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void ClawDown(double power) {
        UpAndDown.setPower(-power);
    }

    public void ClawUp(double power) {
        UpAndDown.setPower(power);
    }

    public void ClawLeftSide(double power) {
        SideToSide.setPower(power);
    }

    public void ClawRightSide(double power) {
        SideToSide.setPower(-power);
    }

    public void ClawBackward(double power) {
        BackAndForth.setPower(power);
    }

    public void ClawForward(double power) {
        BackAndForth.setPower(-power);
    }

    public void ClawOpen() {
        Claw.setPosition(0.1);
    }

    public void ClawClose() {
        Claw.setPosition(0.9);
    }
}

