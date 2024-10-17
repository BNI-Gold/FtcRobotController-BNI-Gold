package org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.Test.GyroDriveTest.RedLeftPark;

import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.Robots.CompetionBot;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.Robots.StraferBot;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.Test.GyroDriveTest.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.Test.GyroDriveTest.AutoTargetZone;

public abstract class RedRightPark extends AutoMain {
    public void parkplace (CompetionBot Bot, AutoTargetZone target) throws InterruptedException{
        switch (target) {
            case A:

                blueRightStack_PIDtest(Bot);
                sleep(sleepTime);

                Bot.strafeLeft_PID(0.4, 3);
                sleep(sleepTime);

                Bot.driveBackward_PID(0.4, 0.3);
                sleep(sleepTime);

                break;

            case B:

                blueRightStack_PIDtest(Bot);
                sleep(sleepTime);

                Bot.driveBackward_PID(0.4, 0.3);
                sleep(sleepTime);

                break;

            case C:

                blueRightStack_PIDtest(Bot);
                sleep(sleepTime);

                Bot.strafeRight_PID(0.4, 2.4);
                sleep(sleepTime);

                Bot.driveBackward_PID(0.4, 0.3);
                sleep(sleepTime);

                break;

            case None:

                blueRightStack_PIDtest(Bot);
                sleep(sleepTime);

                Bot.driveBackward_PID(0.4, 0.3);
                sleep(sleepTime);

                break;
        }
    }

    public void parkplace (StraferBot BotStrafer, AutoTargetZone target) throws InterruptedException{
        switch (target) {
            case A:
                BotStrafer.driveForward_PID(.5,2);
                sleep(sleepTime);

                BotStrafer.gyroCorrection(.2,0);
                sleep(sleepTime);

                BotStrafer.strafeLeft_PID(.5,2.85);
                sleep(sleepTime);

                BotStrafer.gyroCorrection(.2,0);
                sleep(sleepTime);
                break;

            case B:
                BotStrafer.driveForward_PID(.5,3);
                sleep(sleepTime);

                BotStrafer.gyroCorrection(.2,0);
                sleep(sleepTime);
                break;

            case C:
                BotStrafer.driveForward_PID(.5,3);
                sleep(sleepTime);

                BotStrafer.gyroCorrection(.2,0);
                sleep(sleepTime);

                BotStrafer.strafeRight_PID(.5,2.85);
                sleep(sleepTime);

                BotStrafer.gyroCorrection(.2,0);
                sleep(sleepTime);
                break;

        }
    }

    public void park (CompetionBot Bot) {


    }



    public void blueleftStack (CompetionBot Bot) {

        Bot.driveForward_PID(.5,3);
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

        Bot.driveForward_PID(.3,.35);
        sleep(sleepTime);

        Bot.gyroCorrection(.2,-45);
        sleep(600);

        Bot.retractGrabberLift(0.2);
        sleep(500);

        Bot.openGrabberArms();
        sleep(sleepTime);

        Bot.stopGrabberLift();
        sleep(sleepTime);


        Bot.driveBackward_PID(.2,.6);
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

    // CODE BELOW IS WHAT NEEDS TO BE CHANGED FOR TESTING PID AUTONOMOUS FOR LEAGUE CHAMPIONSHIP!

    public void blueRightStack_PIDtest (CompetionBot Bot) {

        Bot.driveForward_PID(0.6,5.2);
        sleep(sleepTime);

        Bot.rotateLeft(0.2, 0.9);
        sleep(sleepTime);

        Bot.gyroCorrection(0.2, 44);
        sleep(sleepTime);

//        Bot.driveForward_PID(0.6, 0.15);
//        sleep(sleepTime);

        Bot.extendGrabberLift(1);
        sleep(3700);

        Bot.extendGrabberLift(0.4);
        sleep(sleepTime);

        Bot.driveForward_PID(0.2, 0.52);
        sleep(sleepTime);

        Bot.stopGrabberLift();
        sleep(1200);

        Bot.openGrabberArms();
        sleep(sleepTime * 2);

        Bot.extendGrabberLift(0.4);
        sleep(sleepTime);

        Bot.driveBackward_PID(0.2, 0.6);
        sleep(sleepTime);

        Bot.stopGrabberLift();
        sleep(sleepTime);

        Bot.retractGrabberLift(0.4);
        sleep(1500);

        Bot.stopGrabberLift();
        sleep(sleepTime);

        Bot.driveBackward_PID(0.3, 0.1);
        sleep(sleepTime);

        Bot.rotateRight(0.4, 1);
        sleep(sleepTime);

        Bot.gyroCorrection(0.2, 0);
        sleep(sleepTime);

//        Bot.rotateLeft(0.4, 2.8);
//        sleep(sleepTime);
//
//        Bot.strafeRight(0.5, 0.25);
//        sleep(sleepTime);
//
//        Bot.driveForward_PID(0.4, 1.75);
//        sleep(sleepTime);
//
//        Bot.retractGrabberLift(0.3);
//        sleep(400);
//        Bot.stopGrabberLift();
//        sleep(sleepTime);
//
//        Bot.driveForward_PID(0.3, 0.3);
//        sleep(sleepTime);
//
//        Bot.closeGrabberArms();
//        sleep(sleepTime);
//
//        Bot.extendGrabberLift(0.7);
//        sleep(800);
//        Bot.stopGrabberLift();
//        sleep(sleepTime);


    }

}
