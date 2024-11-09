package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake.IntakeDirections;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

public abstract class AutoMain extends LinearOpMode {

    // Constructor for the Competition Robot Class
    public CompBot Bot = new CompBot();

    public PrimaryArm arm = new PrimaryArm();

    public Intake intake = new Intake();

    // Initialization, LinearOp, and Telemetry Update for all Auto Paths
    public void autoStart(){
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);
        arm.initPrimaryArm(hardwareMap, Bot.LinearOp);
        intake.initIntake(hardwareMap);
        intake.center();
        telemetry.addLine("Awaiting Start");
        telemetry.update();
    }

    //Helper Method to Raise, Extend, and finish Raising Arm
    public void safeExtendAndRaise() {
        arm.up(3, false);
        sleep(500);
        //stop for 1000ms=1sec
        arm.extend(.75, 5);
        sleep(500);
        arm.up(5.6, true);
    }

    // Helper Method to Drop Sample in Net Area
    public void dropSampleAndRetreat() {
        // Must be called once intake is positioned at top bucket.
        // After dropping, will rotate, back, then strafe robot to be against wall after dropping, and retract arm. Arm stays up.
        intake.start(IntakeDirections.OUT);
        sleep(1000);
        intake.stop();
        sleep(500);
        Bot.driveBack(0.5, 0.5);
        sleep(500);
        Bot.rotateRight(0.5, 1);
        sleep(500);
        Bot.strafeLeft(0.5, 1.25);
        sleep(500);
        Bot.driveBack(0.5, 0.4);
        sleep(500);
        arm.retract(.75, 5.1);
    }
}
