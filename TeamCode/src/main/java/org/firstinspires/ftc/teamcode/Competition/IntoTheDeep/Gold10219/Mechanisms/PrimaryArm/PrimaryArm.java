package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Drivetrains.MecanumDrive;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

public class PrimaryArm {
    public CompBot Bot = null;
    private LinearOpMode LinearOp = null;
    public DcMotor rotator = null;
    public DcMotor arm = null;
    public double rotationUpPower = 0;
    public double rotationDownPower = 0;
    
    public PrimaryArm(CompBot Bot, LinearOpMode LinearOp) {
        this.Bot = Bot;
        this.LinearOp = LinearOp;
        rotator = Bot.primaryArmRotator;
        arm = Bot.primaryArm;
    }

    public void up(double multiplier) {
        rotator.setPower(multiplier * rotationUpPower);
    }
    public void down(double multiplier) {
        rotator.setPower(-multiplier * rotationDownPower);
    }
    public void stopRotation() {
        rotator.setPower(0);
    }
    public void up(double multiplier, double rotations) {
        double ticks = rotations * MecanumDrive.WORMGEAR_TICKS_PER_ROTATION;
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(rotator.getCurrentPosition()) < ticks && LinearOp.opModeIsActive()) {
            up(multiplier);
        }
        stopRotation();
    }
    public void down(double multiplier, double rotations) {
        double ticks = rotations * MecanumDrive.WORMGEAR_TICKS_PER_ROTATION;
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(rotator.getCurrentPosition()) < ticks && LinearOp.opModeIsActive()) {
            down(multiplier);
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
