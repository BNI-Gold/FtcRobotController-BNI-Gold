package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

public abstract class AutoMain extends LinearOpMode {

    // Constructor for the Competition Robot Class
    public CompBot Bot = new CompBot();

    // Initialization, LinearOp, and Telemetry Update for all Auto Paths
    public void autoStart(){
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);
        telemetry.addLine("Awaiting Start");
        telemetry.update();

    }


}
