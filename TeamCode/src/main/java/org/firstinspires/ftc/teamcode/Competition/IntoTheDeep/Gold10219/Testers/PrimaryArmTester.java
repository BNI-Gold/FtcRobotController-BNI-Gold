package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

@TeleOp(name = "B - Primary Arm Tester")
public class PrimaryArmTester extends OpMode {

    public CompBot Bot = new CompBot();
    public PrimaryArm arm = new PrimaryArm(Bot.LinearOp);

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        arm.initPrimaryArm(hardwareMap);
    }

    double speedMultiplier = 1;

    @Override
    public void loop() {
        speedControl();
        primaryArmControl();
        telemetry();
    }

    public void speedControl() {
        if (gamepad1.a) {
            speedMultiplier = 0.35;
        } else {
            speedMultiplier = 1;
        }
    }

    public void primaryArmControl() {
        //Multiply triggers by speed multiplier
        double rightSpeed = gamepad1.right_trigger * speedMultiplier;
        double leftSpeed = gamepad1.left_trigger * speedMultiplier;

        if (gamepad1.right_trigger > 0.35) arm.extend(rightSpeed);
        else if (gamepad1.left_trigger > 0.35) arm.retract(leftSpeed);
        else arm.stop();

        if (gamepad1.dpad_up) arm.up(false);
        else if (gamepad1.dpad_down) arm.down(false);
        else arm.stopRotation();
    }

    public void telemetry() {
        double primaryArmPower = arm.arm.getPower();
        if (primaryArmPower > 0) telemetry.addData("Primary Arm Extending: ", primaryArmPower);
        else if (primaryArmPower < 0) telemetry.addData("Primary Arm Retracting: ", primaryArmPower);
        else telemetry.addLine("Primary Arm Stopped");

        double primaryArmRotatorPower = arm.rotator.getPower();
        if (primaryArmRotatorPower > 0) telemetry.addData("Primary Arm Going Up: ", primaryArmRotatorPower);
        else if (primaryArmRotatorPower < 0) telemetry.addData("Primary Arm Going Down: ", primaryArmRotatorPower);
        else telemetry.addLine("Primary Arm Not Rotating");

        telemetry.update();
    }

}
