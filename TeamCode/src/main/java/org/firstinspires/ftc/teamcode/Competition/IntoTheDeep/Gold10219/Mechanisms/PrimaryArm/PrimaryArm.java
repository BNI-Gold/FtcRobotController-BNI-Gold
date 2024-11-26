package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Drivetrains.MecanumDrive;

public class PrimaryArm {
    public HardwareMap hwBot = null;
    public OpMode OpMode = null;
    public DcMotor rotator = null;
    public double rotationUpPower = .75;
    public double rotationUpSuperPower = 1.25;
    public double rotationDownPower = .5;
    public double rotationDownSuperPower = .75;
    
    public PrimaryArm() {}

    public void initPrimaryArm(HardwareMap hwMap, OpMode OpMode) {
        hwBot = hwMap;
        this.OpMode = OpMode;

        rotator = hwBot.dcMotor.get("primary_arm_rotator");

        rotator.setDirection(DcMotor.Direction.REVERSE);

        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private boolean isOpModeActive() {
        if (OpMode instanceof LinearOpMode) {
            return ((LinearOpMode) OpMode).opModeIsActive();
        }
        return true;  // Default for regular OpMode
    }

    public void up(boolean s) {
        if (s) {
            rotator.setPower(rotationUpSuperPower);
        } else {
            rotator.setPower(rotationUpPower);
        }
    }
    public void down(boolean s) {
        if (s) {
            rotator.setPower(-rotationDownSuperPower);
        } else {
            rotator.setPower(-rotationDownPower);
        }
    }
    public void stopRotation() {
        rotator.setPower(0);
    }
    public void up(double rotations, boolean s) {
        double ticks = rotations * MecanumDrive.WORMGEAR_TICKS_PER_ROTATION;
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(rotator.getCurrentPosition()) < ticks && isOpModeActive()) {
            up(s);
        }
        stopRotation();
    }
    public void down(double rotations, boolean s) {
        double ticks = rotations * MecanumDrive.WORMGEAR_TICKS_PER_ROTATION;
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(rotator.getCurrentPosition()) < ticks && isOpModeActive()) {
            down(s);
        }
        stopRotation();
    }
}
