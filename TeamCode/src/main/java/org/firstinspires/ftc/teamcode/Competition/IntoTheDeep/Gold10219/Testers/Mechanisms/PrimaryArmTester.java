package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Testers.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

@TeleOp(name = "B - Primary Arm Tester", group = "testers")
public class PrimaryArmTester extends OpMode {

    public CompBot Bot = new CompBot();
    public PrimaryArm arm = new PrimaryArm();

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        arm.initPrimaryArm(hardwareMap, Bot.LinearOp);
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
        if (gamepad1.right_bumper) arm.extend();
        else if (gamepad1.left_bumper) arm.retract();

        if (gamepad1.dpad_up) arm.up(false);
        else if (gamepad1.dpad_down) arm.down(false);
        else arm.stopRotation();
    }

    public void telemetry() {
        double extenderPos = arm.extender.getPosition();
        telemetry.addData("Extender Position: ", extenderPos);

        double primaryArmRotatorPower = arm.rotator.getPower();
        if (primaryArmRotatorPower > 0) telemetry.addData("Primary Arm Going Up: ", primaryArmRotatorPower);
        else if (primaryArmRotatorPower < 0) telemetry.addData("Primary Arm Going Down: ", primaryArmRotatorPower);
        else telemetry.addLine("Primary Arm Not Rotating");

        double rotatorPos = arm.rotator.getCurrentPosition();
        telemetry.addData("Primary Arm Position", rotatorPos);

        telemetry.update();
    }

}
