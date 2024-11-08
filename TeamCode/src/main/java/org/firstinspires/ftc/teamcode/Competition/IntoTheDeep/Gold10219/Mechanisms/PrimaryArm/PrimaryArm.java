package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Drivetrains.MecanumDrive;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

public class PrimaryArm {
    public HardwareMap hwBot = null;
    public LinearOpMode LinearOp = null;
    public DcMotor rotator = null;
    public DcMotor arm = null;
    public double rotationUpPower = .75;
    public double rotationUpSuperPower = 1.25;
    public double rotationDownPower = .5;
    
    public PrimaryArm(LinearOpMode LinearOp) {
        this.LinearOp = LinearOp;
    }

    public void initPrimaryArm(HardwareMap hwMap) {
        hwBot = hwMap;

        rotator = hwBot.dcMotor.get("primary_arm_rotator");
        arm = hwBot.dcMotor.get("viper_slide");

        rotator.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void up(boolean s) {
        if (s) {
            rotator.setPower(rotationUpSuperPower);
        } else {
            rotator.setPower(rotationUpPower);
        }
    }
    public void down() {
        rotator.setPower(-rotationDownPower);
    }
    public void stopRotation() {
        rotator.setPower(0);
    }
    public void up(double rotations) {
        double ticks = rotations * MecanumDrive.WORMGEAR_TICKS_PER_ROTATION;
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(rotator.getCurrentPosition()) < ticks && LinearOp.opModeIsActive()) {
            up(false);
        }
        stopRotation();
    }
    public void down(double rotations) {
        double ticks = rotations * MecanumDrive.WORMGEAR_TICKS_PER_ROTATION;
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(rotator.getCurrentPosition()) < ticks && LinearOp.opModeIsActive()) {
            down();
        }
        stopRotation();
    }
    public void extend(double power) {
        arm.setPower(Math.abs(power));
    }
    public void retract(double power) {
        arm.setPower(-Math.abs(power));
    }
    public void stop() {
        arm.setPower(0);
    }
    public void extend(double power, double rotations) {
        double ticks = rotations * MecanumDrive.MOTOR_TICKS_PER_ROTATION;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(Math.abs(arm.getCurrentPosition()) < ticks && LinearOp.opModeIsActive()) {
            extend(power);
        }
        stop();
    }
    public void retract(double power, double rotations) {
        double ticks = rotations * (1) * MecanumDrive.MOTOR_TICKS_PER_ROTATION;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(Math.abs(arm.getCurrentPosition()) < ticks && LinearOp.opModeIsActive()) {
            retract(power);
        }
        stop();
    }
}
