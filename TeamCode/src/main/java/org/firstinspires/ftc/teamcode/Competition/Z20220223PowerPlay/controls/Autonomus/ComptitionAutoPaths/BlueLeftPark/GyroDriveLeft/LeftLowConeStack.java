package org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.ComptitionAutoPaths.BlueLeftPark.GyroDriveLeft;

import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.Robots.CompetionBot;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.ComptitionAutoPaths.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.ComptitionAutoPaths.AutoTargetZone;


public abstract class LeftLowConeStack extends AutoMain {
    public void parkplace (CompetionBot Bot, AutoTargetZone target) throws InterruptedException{
        fiveConeAuto(Bot);

        switch (target) {
            case A:
                fiveConeAuto(Bot);

                break;

            case B:
                fiveConeAuto(Bot);
                break;

            case C:
                fiveConeAuto(Bot);
                break;

            case None:

                fiveConeAuto(Bot);

                break;
        }
    }

    public void fiveConeAuto(CompetionBot Bot) throws InterruptedException {

        sleep(sleepTime);
        //puts everything together

        //scores preloaded cone
        scoreHigh(Bot);

        sleep(sleepTime);

        //stacks first cone

        sleep(sleepTime);

//        scoreConeOne(Bot);

    }

    public void scoreHigh(CompetionBot Bot) throws InterruptedException {

        //start of auto - goes to low goal, scores preloaded cone, gets in position for stacking on low

        Bot.extendGrabberLift(1);
        sleep(50);
        Bot.extendGrabberLift(Bot.stallPower);
        sleep(sleepTime);

        Bot.driveForwardGyro(0.6,5.1);
        sleep(sleepTime);

        Bot.rotateRight(0.2, 1);
        sleep(sleepTime);

        Bot.gyroCorrection(0.2, -45);
        sleep(sleepTime);
//        Bot.gyroCorrection(0.2, -45);
//        sleep(sleepTime);

        Bot.extendGrabberLift(1);
        sleep(3800);
        Bot.extendGrabberLift(Bot.stallPower);
        sleep(sleepTime);

        Bot.driveForwardGyro(0.45, 0.31);
        sleep(sleepTime);

        Bot.retractGrabberLift(0.15);
        sleep(290);
        Bot.extendGrabberLift(Bot.stallPower);
        sleep(1000); //extra time as robot wobbles when retracting

        Bot.openGrabberArms();
        sleep(sleepTime * 2);

        Bot.extendGrabberLift(Bot.stallPower);
        sleep(sleepTime);

        Bot.driveBackwardGyro(0.3, 0.6);
        sleep(sleepTime);

        Bot.retractGrabberLift(0.4);
        sleep(1500);
        Bot.extendGrabberLift(Bot.stallPower);
        sleep(500); //extra time as robot wobbles when retracting

//        Bot.stopGrabberLift();
//        sleep(sleepTime);

//        Bot.retractGrabberLift(0.2);
//        sleep(1500);

//        Bot.extendGrabberLift(Bot.stallPower);
//        sleep(sleepTime);

//        Bot.driveBackwardGyro(0.3, 0.1);
//        sleep(sleepTime);
//
//        Bot.rotateLeft(0.4, 1);
//        sleep(sleepTime);
//
//        Bot.gyroCorrection(0.3, 0);
//        sleep(sleepTime);
//
//        Bot.driveForwardGyro(0.2, 0.15);
//        sleep(sleepTime);
//
//        Bot.rotateLeft(0.4, 2);
//        sleep(sleepTime);
//
//        Bot.gyroCorrection(0.3, 90);
//        sleep(sleepTime);
//
//        Bot.driveForwardGyro(0.3, 1);
//        sleep(sleepTime);
//
//        Bot.gyroCorrection(0.3, 90);
//        sleep(sleepTime);

    }

    public void parkZone (CompetionBot Bot, AutoTargetZone target) throws InterruptedException {

        switch (target) {
            case A:
                Bot.rotateRight(0.4, 1);
                sleep(sleepTime);

                Bot.gyroCorrection(0.3, -90);
                sleep(sleepTime);

                Bot.driveBackwardGyro(.7, 1.59);
                sleep(sleepTime);

                Bot.rotateLeft(0.4, 1.8);
                sleep(sleepTime);

                Bot.strafeLeft(0.3, 0.32);
                sleep(sleepTime);

                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTime);

                Bot.driveBackwardGyro(0.5, 0.2);
                sleep(sleepTime);

                break;
            case B:
                Bot.rotateLeft(0.4, 1);
                sleep(sleepTime);

                Bot.gyroCorrection(0.3, 0);
                sleep(sleepTime);

                Bot.driveBackwardGyro(0.7, 0.5);
                break;
            case C:
                Bot.rotateRight(0.4, 1);
                sleep(sleepTime);

                Bot.gyroCorrection(0.3, -90);
                sleep(sleepTime);

                Bot.driveForwardGyro(0.7, 2.2);
                sleep(sleepTime);




                Bot.rotateRight(0.4, 1.9);
                sleep(sleepTime);

//                Bot.driveForward(0.7, 0.5);
//                sleep(sleepTime);

//                Bot.gyroCorrection(0.3, -180);
//                sleep(sleepTime);
//
//                Bot.driveForward(0.7, 4);
//                sleep(sleepTime);


                break;
            case None:
//                Uses "B"
                Bot.rotateLeft(0.4, 1);
                sleep(sleepTime);

                Bot.gyroCorrection(0.3, 0);
                sleep(sleepTime);

                Bot.driveBackwardGyro(1, 0.5);
                sleep(sleepTime);

                break;
        }
    }

    public void parkFromLow(CompetionBot Bot) {

        //moves from position of stacking final cone --> parking position 1 (can go from there into zone 2 or 3)

    }

    public void scoreConeOne(CompetionBot Bot) throws InterruptedException {

        //grabs first cone (highest cone)

        Bot.driveForwardGyro(0.35, 1.6);
        sleep(sleepTime);

        Bot.closeGrabberArms();
        sleep(sleepTime);

//        Bot.extendGrabberLift(0.8);
//        sleep(600);

//        Bot.extendGrabberLift(0.4);
//        sleep(sleepTime);

        Bot.driveBackwardGyro(0.3, 1.5);
        sleep(sleepTime);

    }

    public void grabConeTwo(CompetionBot Bot) {

        //grabs second cone

    }

    public void grabConeThree(CompetionBot Bot) {

        //grabs third cone

    }

    public void grabConeFour(CompetionBot Bot) {

        //grabs fourth cone

    }

    public void grabConeFive(CompetionBot Bot) {

        //grabs fifth cone

    }

}
