package org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.ComptitionAutoPaths.BlueLeftPark;

import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.Robots.CompetionBot;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.Robots.StraferBot;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.ComptitionAutoPaths.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.ComptitionAutoPaths.AutoTargetZone;

public abstract class BlueLeftPark extends AutoMain {
    public void parkplace (CompetionBot Bot, AutoTargetZone target) throws InterruptedException{
        switch (target) {
            case A:
                blueleftStack(Bot);
                sleep(sleepTime);

                Bot.strafeRight(0.6, 0.1);
                sleep(sleepTime);

                Bot.driveBackward(0.4, 2.8);
                sleep(sleepTime);

                break;

            case B:
                blueleftStack(Bot);
                sleep(sleepTime);

                Bot.driveBackward(0.5, 0.1);
                sleep(sleepTime);

                Bot.rotateLeft(0.4, 2.2);
                sleep(sleepTime);

                break;

            case C:
                blueleftStack(Bot);
                sleep(sleepTime);

                Bot.strafeRight(0.6, 0.1);
                sleep(sleepTime);

                Bot.driveForward(0.4, 2.2);
                sleep(sleepTime);

                Bot.rotateLeft(0.4, 2.2);
                sleep(sleepTime);

                break;

            case None:
                blueleftStack(Bot);
                sleep(sleepTime);

                Bot.driveBackward(0.5, 0.1);
                sleep(sleepTime);

                Bot.rotateLeft(0.4, 2.2);
                sleep(sleepTime);

                break;
        }
    }

    public void parkplace (StraferBot BotStrafer, AutoTargetZone target) throws InterruptedException{
        switch (target) {
            case A:
                BotStrafer.driveForward(.5,2);
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

    public void blueleftStack (CompetionBot Bot) {

        Bot.driveForward(.5,3);
        sleep(sleepTime);

        Bot.gyroCorrection(.3,0);
        sleep(sleepTime);

        Bot.rotateRight(0.3, 0.5);
        sleep(sleepTime);

        Bot.gyroCorrection(0.3, -45);
        sleep(sleepTime);

        Bot.extendGrabberLift(0.7);
        sleep(2000);

        Bot.stopGrabberLift();
        sleep(sleepTime);

        Bot.extendGrabberLift(.2);
        sleep(700);

        Bot.driveForward(.3,.35);
        sleep(sleepTime);

        Bot.gyroCorrection(.2,-45);
        sleep(600);

        Bot.retractGrabberLift(0.2);
        sleep(500);

        Bot.openGrabberArms();
        sleep(sleepTime);

        Bot.stopGrabberLift();
        sleep(sleepTime);


        Bot.driveBackward(.2,.6);
        sleep(sleepTime);

        Bot.retractGrabberLift(0.2);
        sleep(450);

        Bot.stopGrabberLift();
        sleep(sleepTime);

        Bot.rotateRight(.5,.4);
        sleep(sleepTime);

        Bot.gyroCorrection(.2,-90);
        sleep(sleepTime);

    }
}
