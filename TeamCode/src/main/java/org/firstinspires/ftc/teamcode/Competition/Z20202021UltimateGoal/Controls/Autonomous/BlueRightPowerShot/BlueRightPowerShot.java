package org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.BlueRightPowerShot;

import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.TargetZone;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots.CompetitionBot;

public abstract class BlueRightPowerShot extends AutoMain {
    public void driveToZoneOne(CompetitionBot Bot, TargetZone target) throws InterruptedException {
        switch (target) {
            case A:
//                Line 13 is an example of an "encoder turn"  Use that, and then a gyroCorrection.
                Bot.rotateRight(1, 2.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, -130);
                sleep(sleepTimeDrive);
                Bot.driveBackward(.3);
                sleep(600);
                Bot.stopMotors();
                sleep(sleepTimeDrive);
                break;
            case B:
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                Bot.driveForward(1,3.4);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                Bot.strafeRight(.78,0.85);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                break;
            case C:
                Bot.rotateRight(1,2.8);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,-166);
                sleep(sleepTimeDrive);
                Bot.driveBackward(.7,3.8);
                sleep(sleepTimeDrive);
//                Bot.driveForward(0.5, 3.6);
//                sleep(sleepTimeDrive);
//                Bot.gyroCorrection(0.5, -120);
//                sleep(sleepTimeDrive);
//                Bot.gyroCorrection(0.2, -120);
//                sleep(sleepTimeDrive);
//                Bot.driveBackward(.4);
//                sleep(230);
//                Bot.stopMotors();
                break;
        }
    }

    public void ScoreRings (CompetitionBot Bot, TargetZone target) throws InterruptedException {
//        launch 1
        int launchSleep = 750;
//        double launchPower = 0.65;
        double launchVelocity = 1800;
        Bot.LauncherOn(1800);
        sleep(2500);
        Bot.gyroCorrection(.2,3);
        sleep(sleepTimeDrive);
        Bot.gyroCorrection(.2,3);
        sleep(sleepTimeDrive);
        Bot.RingPull();
        sleep(launchSleep);
        Bot.RingPush();
//        Launch 2
        Bot.LauncherOn(launchVelocity);
        sleep(launchSleep);
        Bot.gyroCorrection(.2,12);
        sleep(sleepTimeDrive);
        Bot.gyroCorrection(.2,12);
        sleep(sleepTimeDrive);
        Bot.RingPull();
        sleep(launchSleep);
        Bot.RingPush();
//        Launch 3
        Bot.LauncherOn(launchVelocity);
        sleep(launchSleep);
        Bot.gyroCorrection(.2,15);
        sleep(sleepTimeDrive);
        Bot.gyroCorrection(.2,15);
        sleep(sleepTimeDrive);
        Bot.RingPull();
        sleep(launchSleep);
        Bot.RingPush();
        sleep(50);  //here so the ring mag doesn't get caught on the pusher.
//        ADD THIS SLEEP BACK IN IF NEEDED
//        sleep(launchSleep);
        Bot.LauncherOff(0);
        Bot.RingMagDown();
        sleep(launchSleep);

    }

    public void driveToLeftWobble (CompetitionBot Bot, TargetZone target) throws InterruptedException {
        switch (target) {
            case A:
//                was 1.8
//                IF SPINNING, MAKE LOWER ROTATIONS

                Bot.driveForward(0.8,1.7);
                sleep(sleepTimeDrive);
                Bot.rotateLeft(1, 1.4);
                sleep(sleepTimeDrive);
                // was at -18 - too far to the left changed to -24 to go to the right
                Bot.gyroCorrection(0.3, -24);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, -24);
                sleep(sleepTimeDrive);
//                Bot.WobbleArmLower(1);
//                sleep(sleepTimeDrive);
                Bot.driveBackward(1,.65);
                sleep(sleepTimeDrive);
                Bot.driveBackward(.2,.53);
                sleep(sleepTimeDrive);
                Bot.WobbleClosed();
                sleep(400);
                Bot.WobbleArmRaised(1);
                sleep(200);
                Bot.WobbleArmStopMotors();
                Bot.rotateRight(.5,2.7);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,-160);
                sleep(sleepTimeDrive);
                Bot.driveBackward(.8,2.8);
                sleep(sleepTimeDrive);
                Bot.WobbleOpen();
                sleep(sleepTimeDrive);

                break;
            case B:
                Bot.strafeLeft(.85,2.3);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                Bot.driveBackward(1,5.78);
                sleep(sleepTimeDrive);
//                7.0 - was slightly to left
//                7.5 went more to left - updating to 6.5.
                Bot.gyroCorrection(.3,10);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,10);
                sleep(sleepTimeDrive);
                Bot.driveBackward(.3,.5);
                sleep(sleepTimeDrive);
                Bot.WobbleClosed();
                sleep(600);
                Bot.WobbleArmRaised(1);
                sleep(200);
                Bot.WobbleArmStopMotors();
                Bot.driveForward(1,3);
                sleep(sleepTimeDrive);
                Bot.rotateLeft(1,2.5);
                sleep(sleepTimeDrive);
                Bot.driveBackward(0.6,.8);
                sleep(sleepTimeDrive);
                Bot.WobbleOpen();
                sleep(sleepTimeDrive);

                break;
            case C:
                Bot.gyroCorrection(.2,-164);
                sleep(sleepTimeDrive);
                Bot.driveForward(1,4.2);
                sleep(sleepTimeDrive);
                Bot.rotateRight(.7,4);
                sleep(sleepTimeDrive);
//                to far to right - was angle 4
                Bot.gyroCorrection(0.3,9);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2,9);
                sleep(sleepTimeDrive);
                Bot.driveBackward(.4,1.2);
                sleep(sleepTimeDrive);
                Bot.WobbleClosed();
                sleep(600);
                Bot.WobbleArmRaised(1);
                sleep(200);
                Bot.WobbleArmStopMotors();
                sleep(200);
                Bot.driveForward(1,1.4);
                sleep(sleepTimeDrive);
                Bot.rotateRight(1,3.04);
                sleep(sleepTimeDrive);
                Bot.driveBackward(1,4);
                sleep(sleepTimeDrive);
                Bot.WobbleOpen();
                sleep(sleepTimeDrive);
                break;
    }
}

    public void driveToZoneTwo (CompetitionBot Bot, TargetZone target) throws InterruptedException {
        switch (target) {
            case A:
                Bot.gyroCorrection(0.2,360);
                sleep(sleepTimeDrive);
                Bot.driveGyroBackward(.5,5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2,0);
                sleep(sleepTimeDrive);
                Bot.strafeRight(0.3,0.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2,0);
                sleep(sleepTimeDrive);
                break;
            case B:
                Bot.driveGyroBackward(0.5,2);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2,360);
                sleep(sleepTimeDrive);
                Bot.driveGyroForward(0.5,3);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2,0);
                sleep(sleepTimeDrive);
                Bot.strafeLeft(0.3,1);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2,0);
                sleep(sleepTimeDrive);
                break;
            case C:
                Bot.gyroCorrection(0.2,360);
                sleep(sleepTimeDrive);
                Bot.driveGyroBackward(0.5,9.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2,0);
                sleep(sleepTimeDrive);
                break;
        }
    }

    public void ParkLaunchLine(CompetitionBot Bot, TargetZone target) throws InterruptedException {
        switch (target) {
            case A:
                Bot.driveForward(1,2);
                sleep(sleepTimeDrive);
                Bot.rotateLeft(1,.5);
                sleep(sleepTimeDrive);
                Bot.strafeLeft(1,5);
                sleep(sleepTimeDrive);
                break;
            case B:
                Bot.driveForward(1,.8);
                sleep(sleepTimeDrive);

                break;
            case C:
                Bot.driveForward(1,2.54);

                break;
        }
    }
}