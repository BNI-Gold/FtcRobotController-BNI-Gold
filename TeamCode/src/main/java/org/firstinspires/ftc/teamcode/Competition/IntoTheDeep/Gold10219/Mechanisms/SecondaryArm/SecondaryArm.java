package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.SecondaryArm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SecondaryArm {
    public HardwareMap hwBot = null;
    public OpMode OpMode = null;
    public Servo extender = null;

    public double rotationUpPower = .5;
    public double rotationUpSuperPower = .75;
    public double rotationDownPower = .5;
    public double rotationDownSuperPower = .75;

    public double retractedPosition = .2989;
    public double extendedPosition = .865;

    public double extenderAdjust = .001;

    public SecondaryArm() {}

    public void initSecondaryArm(HardwareMap hwMap, OpMode OpMode) {
        hwBot = hwMap;
        this.OpMode = OpMode;

        extender = hwBot.servo.get("secondary_arm_extender");

        extender.setDirection(Servo.Direction.FORWARD);
    }

    private boolean isOpModeActive() {
        if (OpMode instanceof LinearOpMode) {
            return ((LinearOpMode) OpMode).opModeIsActive();
        }
        return true;  // Default for regular OpMode
    }

    public enum rotationStates {
        STOPPED, UP, GOING_UP, DOWN, GOING_DOWN
    }

    public double rotations = 0;
    public boolean s = false;

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
        extender.setPosition(Math.min(pos + (extenderAdjust * val * 8), extendedPosition));
    }

    public void retract(double val) {
        double pos = extender.getPosition();
        extender.setPosition(Math.max(pos - (extenderAdjust * val * 8), retractedPosition));
    }
}
