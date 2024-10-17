package org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.ComptitionAutoPaths.RedRightPark;

import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.Robots.CompetionBot;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.Robots.StraferBot;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.ComptitionAutoPaths.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.ComptitionAutoPaths.AutoTargetZone;

public abstract class RedRightPark extends AutoMain {
    public void parkplace (CompetionBot Bot, AutoTargetZone target) throws InterruptedException{
        switch (target) {
            case A:
                blueRightStack(Bot);
                sleep(sleepTime);

                Bot.driveForward(.5,1.2);
                sleep(sleepTime);

                break;

            case B:
                blueRightStack(Bot);
                sleep(sleepTime);

                Bot.driveForward(.5,.1);
                sleep(sleepTime);


                break;

            case C:
                blueRightStack(Bot);
                sleep(sleepTime);

                Bot.driveBackward(.5,1.15);
                sleep(sleepTime);

                break;

//            case None:
//                Bot.driveForward(0.5, 2.75);
//                sleep(sleepTime);
            //(not tested yet)
        }
    }

    public void parkplace (StraferBot BotStrafer, AutoTargetZone target) throws InterruptedException{
        switch (target) {
            case A:
                BotStrafer.driveForward(.5,3);
                sleep(sleepTime);

                BotStrafer.gyroCorrection(.2,0);
                sleep(sleepTime);

                BotStrafer.strafeLeft(.5,2.85);
                sleep(sleepTime);

                BotStrafer.gyroCorrection(.2,0);
                sleep(sleepTime);
                break;

            case B:
                BotStrafer.driveForward(.5,3);
                sleep(sleepTime);

                BotStrafer.gyroCorrection(.2,0);
                sleep(sleepTime);
                break;

            case C:
                BotStrafer.driveForward(.5,3);
                sleep(sleepTime);

                BotStrafer.gyroCorrection(.2,0);
                sleep(sleepTime);

                BotStrafer.strafeRight(.5,2.85);
                sleep(sleepTime);

                BotStrafer.gyroCorrection(.2,0);
                sleep(sleepTime);
                break;

        }
    }

    public void park (CompetionBot Bot) {


    }

    public void blueRightStack (CompetionBot Bot) {

        Bot.driveForward(.5,1.4);
        sleep(sleepTime);

        Bot.gyroCorrection(.3,0);
        sleep(sleepTime);

        Bot.rotateLeft(0.3, 0.4);
        sleep(sleepTime);

        Bot.gyroCorrection(0.3, 50);
        sleep(sleepTime);

        Bot.extendGrabberLift(0.7);
        sleep(1600);

        Bot.stopGrabberLift();
        sleep(sleepTime);

        Bot.extendGrabberLift(.2);
        sleep(700);

        Bot.driveForward(.3,.25);
        sleep(sleepTime);

        Bot.gyroCorrection(.2,50);
        sleep(600);

        Bot.openGrabberArms();
        sleep(sleepTime);

        Bot.stopGrabberLift();
        sleep(sleepTime);

        Bot.driveBackward(.2,.3);
        sleep(sleepTime);

        Bot.rotateLeft(.5,.4);
        sleep(sleepTime);

        Bot.gyroCorrection(.2,90);
        sleep(sleepTime);
    }
}
