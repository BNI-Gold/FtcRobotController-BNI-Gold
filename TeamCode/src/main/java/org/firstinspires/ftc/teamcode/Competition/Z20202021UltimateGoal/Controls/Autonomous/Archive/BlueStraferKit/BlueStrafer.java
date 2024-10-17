package org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.Archive.BlueStraferKit;

import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.TargetZone;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots.StraferKit;

public abstract class BlueStrafer extends AutoMain {



    public void driveToTargetZone (StraferKit Bot, TargetZone target) throws InterruptedException {
        switch (target) {
            case A:
                Bot.gyroCorrection(0.2,-120);
                sleep(sleepTimeDrive);
                Bot.driveBackward(.25);
                sleep(600);
                Bot.gyroCorrection(0.2,-135);
                sleep(1000);
                Bot.gyroCorrection(0.2,0);
                sleep(sleepTimeDrive);
                Bot.strafeRight(0.3,1.5);
                sleep(sleepTimeDrive);
                Bot.driveForward(0.3,0.75);
                sleep(sleepTimeDrive);
//                Bot.driveGyroBackward(0.5,5.5);
//                sleep(sleepTimeDrive);
//                Bot.gyroCorrection(0.2, 0);
//                sleep(sleepTimeDrive);
////                was 0.75
//                Bot.strafeLeft(.5,1.1);
//                sleep(sleepTimeDrive);
//                Bot.gyroCorrection(0.2, 0);
//                sleep(sleepTimeDrive);
                break;
            case B:
                Bot.driveForward(.3,6);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2,0);
                sleep(1000);
//                Bot.gyroCorrection(0.2,179.4);
//                sleep(sleepTimeDrive);
                Bot.strafeRight(.3,3);
                sleep(sleepTimeDrive);
//                Bot.gyroCorrection(0.2, 179);
//                sleep(sleepTimeDrive);
//                Bot.strafeRight(0.5,2.5);
//                sleep(sleepTimeDrive);
//                Bot.gyroCorrection(0.2, 0);
//                sleep(sleepTimeDrive);
//                Bot.driveBackward(.3,.5);
//                sleep(sleepTimeDrive);
                Bot.driveBackward(.5);
                sleep(300);
                break;
            case C:
                Bot.driveForward(0.3,5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2,-120);
                sleep(sleepTimeDrive);
                Bot.driveBackward(0.25);
                sleep(600);
                Bot.gyroCorrection(0.2,-135);
                sleep(1000);
                Bot.gyroCorrection(0.2,0);
                sleep(sleepTimeDrive);
                Bot.driveBackward(.25);
                sleep(1800);
                Bot.gyroCorrection(0.2,0);
                sleep(sleepTimeDrive);
//                Bot.driveGyroBackward(0.7, 9.3);
//                sleep(sleepTimeDrive);
//                Bot.gyroCorrection(0.2, 0);
//                sleep(sleepTimeDrive);
////                was 1.0
//                Bot.strafeLeft(0.7,1.1);
//                sleep(sleepTimeDrive);
//                Bot.gyroCorrection(0.2, 0);
//                sleep(sleepTimeDrive);
                break;
        }
    }

    public void ParkLaunchLine (StraferKit Bot, TargetZone target) throws InterruptedException {
        switch (target) {
            case A:
                Bot.strafeLeft(.5,1);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
//                No need to move - already parked on line ot score Wobble.
                break;
            case B:
                Bot.strafeLeft(.5,.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2,0);
                sleep(sleepTimeDrive);
                Bot.driveGyroForward(0.5, 1.7);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
                break;
            case C:
                Bot.strafeLeft(.5,1);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
                Bot.driveGyroForward(0.4, 3.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
                break;
        }
    }

    public void driveToTargetZoneDouble (StraferKit Bot, TargetZone target) throws InterruptedException {
        switch (target){
            case A:
                Bot.driveGyroBackward(.4,4.8);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                Bot.strafeRight(.4,2.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                break;
            case B:
                Bot.driveGyroBackward(.5,6.1);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                Bot.strafeRight(.5,0.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                break;
            case C:
                Bot.driveGyroBackward(.5,8.1);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                Bot.strafeRight(.5,2.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                break;


        }
    }

    public void driveToWobble (StraferKit Bot, TargetZone target) throws InterruptedException {
        switch (target){
            case A:
                Bot.strafeLeft(.5,3.8);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
                Bot.driveForward(.5, 4.1);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
                break;
            case B:
                Bot.strafeLeft(.5,1.7);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                Bot.driveGyroForward(.5,6.2);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                break;
            case C:
                Bot.strafeLeft(.7,3.8);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                Bot.driveGyroForward(.7,7.9);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                break;
        }
    }
}
