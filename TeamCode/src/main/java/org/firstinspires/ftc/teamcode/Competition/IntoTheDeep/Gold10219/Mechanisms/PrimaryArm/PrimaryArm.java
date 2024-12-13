package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Drivetrains.MecanumDrive;

public class PrimaryArm {
    public HardwareMap hwBot = null;
    public OpMode OpMode = null;
    public DcMotor rotator = null;
    public Servo extender = null;

    public double rotationUpPower = .5;
    public double rotationUpSuperPower = .75;
    public double rotationDownPower = .5;
    public double rotationDownSuperPower = .75;

    public double retractedPosition = .2989;
    public double extendedPosition = .865;

    public double extenderAdjust = .001;
    
    public PrimaryArm() {}

    public void initPrimaryArm(HardwareMap hwMap, OpMode OpMode) {
        hwBot = hwMap;
        this.OpMode = OpMode;

        rotator = hwBot.dcMotor.get("primary_arm_rotator");
        extender = hwBot.servo.get("primary_arm_extender");

        rotator.setDirection(DcMotor.Direction.REVERSE);
        extender.setDirection(Servo.Direction.FORWARD);

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

    public enum rotationStates {
        STOPPED, UP, GOING_UP, DOWN, GOING_DOWN
    }

    private rotationStates rotationState = rotationStates.STOPPED;
    public double rotations = 0;
    public boolean s = false;

    public rotationStates getRotationState() {
        return rotationState;
    }

    public boolean isStopped() {
        return rotationState == rotationStates.STOPPED;
    }

    public void setRotation(rotationStates state, double rotations, boolean s) {
        if (state == rotationStates.GOING_UP || state == rotationStates.GOING_DOWN) return;
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationState = state;
        this.rotations = rotations;
        this.s = s;
    }

    private void setState(rotationStates state) {
        rotationState = state;
    }

    private double getCurrentRotations() {
        return Math.abs(rotator.getCurrentPosition()) / MecanumDrive.WORMGEAR_TICKS_PER_ROTATION;
    }

    public void rotationChecker() {
        switch (rotationState) {
            case STOPPED:
                break;
            case UP:
                up(s);
                setState(rotationStates.GOING_UP);
                break;
            case GOING_UP:
                double currentRotations = getCurrentRotations();
                if (currentRotations > rotations) {
                    stopRotation();
                    setState(rotationStates.STOPPED);
                }
                break;
            case DOWN:
                down(s);
                setState(rotationStates.GOING_DOWN);
                break;
            case GOING_DOWN:
                double currentRotations2 = getCurrentRotations();
                if (currentRotations2 > rotations) {
                    stopRotation();
                    setState(rotationStates.STOPPED);
                }
                break;
        }
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

    public void setExtend() {
        extender.setPosition(extendedPosition);
    }

    public void setRetract() {
        extender.setPosition(retractedPosition);
    }

    public void extend() {
        double pos = extender.getPosition();
        extender.setPosition(pos + extenderAdjust);
    }

    public void retract() {
        double pos = extender.getPosition();
        extender.setPosition(pos - extenderAdjust);
    }

    public void extend(double val) {
        double pos = extender.getPosition();
        extender.setPosition(Math.min(pos + (extenderAdjust * val), extendedPosition));
    }

    public void retract(double val) {
        double pos = extender.getPosition();
        extender.setPosition(Math.max(pos - (extenderAdjust * val), retractedPosition));
    }
}
