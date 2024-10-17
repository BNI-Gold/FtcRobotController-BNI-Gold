package org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.Archive.Lab.DuValTest;

import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Controls.Autonomous.TargetZone;
import org.firstinspires.ftc.teamcode.Competition.Z20202021UltimateGoal.Robots.CompetitionBot;

public abstract class BlueLeftTEST extends AutoMain {
    public void driveToTargetZone (CompetitionBot Bot, TargetZone target) throws InterruptedException {
        switch (target) {
            case A:
                Bot.driveGyroBackward(0.5,5.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
                Bot.strafeLeft(.5,1);
                sleep(sleepTimeDrive);
                break;
            case B:
                Bot.driveGyroBackward(.5,7.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
                Bot.strafeLeft(0.5,3.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
                break;
            case C:
//                Bot.driveBackward(.5,5);
                Bot.driveGyroBackward(0.5, 9.3);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
                Bot.strafeLeft(0.5,1);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
                break;
        }
    }

    public void ParkLaunchLine (CompetitionBot Bot, TargetZone target) throws InterruptedException {
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
                Bot.driveGyroForward(0.5, 1.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
                break;
            case C:
//                Bot.driveForward(0.5, 4.5);
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
    public void driveToTargetZoneDouble (CompetitionBot Bot, TargetZone target) throws InterruptedException {
        switch (target){
            case A:
                Bot.driveGyroBackward(.5,4.7);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                Bot.strafeRight(.5,4);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                break;
            case B:
                Bot.driveGyroBackward(.5,7.2);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                Bot.strafeRight(.5,1.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                break;
            case C:
                Bot.driveGyroBackward(.5,8.1);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                Bot.strafeRight(.5,4);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                break;


        }

    }
    public void driveToWobble (CompetitionBot Bot, TargetZone target) throws InterruptedException {
        switch (target){
            case A:
                Bot.strafeLeft(.5,4);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
//                Bot.driveGyroForward(.5,4.5);
                Bot.driveForward(.5, 4.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(0.2, 0);
                sleep(sleepTimeDrive);
                break;
            case B:
                Bot.strafeLeft(.5,1.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                Bot.driveGyroForward(.5,7);
                sleep(sleepTimeDrive);
                break;
            case C:
                Bot.strafeLeft(.5,1.5);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                Bot.driveGyroForward(.5,8.3);
                sleep(sleepTimeDrive);
                Bot.gyroCorrection(.2,0);
                sleep(sleepTimeDrive);
                break;
        }
    }

    public void gyroTEST (CompetitionBot Bot) throws InterruptedException {
        Bot.driveGyroBackward(.5,5.5);
        sleep(sleepTimeDrive);
        Bot.gyroCorrection(0.2, 0);
        sleep(sleepTimeDrive);
        Bot.strafeLeft(.5,2);
        sleep(sleepTimeDrive);
        Bot.driveGyroForward( 0.5, 4);
//        Bot.driveGyroStraight(2000, 0.4);
    }

}
