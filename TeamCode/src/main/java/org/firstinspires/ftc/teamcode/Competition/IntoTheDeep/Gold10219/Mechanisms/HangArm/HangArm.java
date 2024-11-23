package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.HangArm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangArm {
    public HardwareMap hwBot = null;
    public OpMode OpMode = null;
    public DcMotor motor = null;

    public HangArm() {}

    public void initHangArm(HardwareMap hwMap, OpMode OpMode) {
        hwBot = hwMap;
        this.OpMode = OpMode;

        motor = hwBot.dcMotor.get("hang_arm_motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private boolean isOpModeActive() {
        if (OpMode instanceof LinearOpMode) {
            return ((LinearOpMode) OpMode).opModeIsActive();
        }
        return true;
    }

    public void extend() {
        motor.setPower(1);
    }

    public void retract() {
        motor.setPower(-1);
    }

    public void stopRotation() {motor.setPower(0);}
}
