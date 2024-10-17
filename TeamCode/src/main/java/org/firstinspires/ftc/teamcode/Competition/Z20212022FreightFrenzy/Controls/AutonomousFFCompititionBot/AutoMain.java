package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.AutonomousFFCompititionBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.TargetZone;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.TankBot;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.mechanisms.TSELocation;

public abstract class AutoMain extends LinearOpMode {


    public int sleepTimeDrive = 200;

    public TargetZone zone = null;
    public TSELocation tselocation;
    //CAMERA METHODS - emma


    public void collectTSE (TankBot Bot) {
        switch (tselocation) {
            case barcode1:
                telemetry.addLine("barcode1");
            case barcode2:
                telemetry.addLine("barcode2");
            case barcode3:
                telemetry.addLine("barcode3");
        }
    }





//    public void DriveForwardOffWall (EightWheelBot) {
//
//    }
}
