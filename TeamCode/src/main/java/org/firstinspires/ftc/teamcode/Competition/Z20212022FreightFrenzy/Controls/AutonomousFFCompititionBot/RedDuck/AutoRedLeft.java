package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.AutonomousFFCompititionBot.RedDuck;

import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.StartPosition;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.TargetZone;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.EightWheelBot;

public class AutoRedLeft extends RedLeft {

    public EightWheelBot Bot = new EightWheelBot();
    public StartPosition startPosition = null;
    public TargetZone targetZone = null;
    public int sleepTime = 250;


    @Override
    public void runOpMode() throws InterruptedException {


        requestOpModeStop();
    }
}
