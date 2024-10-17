package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.Tank;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.TankBot;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.mechanisms.TSELocation;

public abstract class AutoMain extends LinearOpMode {
    public long sleepTime = 250;

    //90 degree LEFT turn =
    //         Bot.rotateLeft(.5, 1.68);
    private double straightSpd = 0.6;
    private double turnEncoderSpd = 0.5;
    //        Speed .2 == too low for gyro turn
    private double turnGyro1 = 0.25;
    private double turnGyro2 = 0.3;



    /*

    NOW COLLECTING CAMERA DATA INSIDE TankBot.Java > TSELocation detectBarcode()

    public TSELocation locator (TankBot Bot) {

//        Bot.detectBarcode(Bot);

        return TSELocation.barcode2;
    }

     */

    public void TestAuto(TankBot Bot) {
        Bot.driveForward(1, 10);
        sleep(sleepTime);
        Bot.driveBackward(1, 6);
        sleep(sleepTime);
        Bot.rotateLeft(0.5, 10);
        sleep(sleepTime);
        Bot.rotateRight(0.5, 10);
        sleep(sleepTime);
    }

    public void spinDuckBlue(TankBot Bot) {
        Bot.duckspincounterclockwiseAuto();
        //Time to spin duck spinner!
        sleep(6000);
        Bot.duckspinstop();
        sleep(sleepTime);
    }

    public void spinDuckRed(TankBot Bot) {
        Bot.duckspinclockwiseAuto();
        //Time to spin duck spinner!
        sleep(6000);
        Bot.duckspinstop();
        sleep(sleepTime);
    }

    public void ScoreFreight (TankBot Bot, TSELocation location) {

    }
}
