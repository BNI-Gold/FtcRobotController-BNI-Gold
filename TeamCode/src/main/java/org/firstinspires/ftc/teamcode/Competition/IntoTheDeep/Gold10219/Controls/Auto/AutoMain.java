package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

public abstract class AutoMain extends LinearOpMode {

    // Constructor for the Competition Robot Class
    public CompBot Bot = new CompBot();

    PrimaryArm arm = new PrimaryArm(Bot.LinearOp);

    Intake intake = new Intake();

    // Initialization, LinearOp, and Telemetry Update for all Auto Paths
    public void autoStart(){
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);
        arm.initPrimaryArm(hardwareMap);
        intake.initIntake(hardwareMap);
        intake.center();
        telemetry.addLine("Awaiting Start");
        telemetry.update();

    }


}
