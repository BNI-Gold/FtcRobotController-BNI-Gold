package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.clawOptions;

public abstract class AutoMain extends LinearOpMode {

        public CompBot Bot = new CompBot();

public void autoStart(){
    Bot.initRobot(hardwareMap);
    Bot.setLinearOp(this);
    telemetry.addLine("Awaiting Start");
    telemetry.update();


}


}
