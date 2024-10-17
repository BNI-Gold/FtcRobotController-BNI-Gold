package org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.Test.GyroDriveTest.BlueLeftPark;

import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.Robots.CompetionBot;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.Test.GyroDriveTest.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.Test.GyroDriveTest.AutoTargetZone;

public abstract class BlueLeftPark extends AutoMain {

    boolean gyroTest = false;

    public void parkplace(CompetionBot Bot, AutoTargetZone target) throws InterruptedException {
        switch (target) {
            case A:

                break;

            case B:

                break;

            case C:

                break;

            case None:

                if (gyroTest == true) {

                    competitionGyroDriveTest(Bot);

                } else if (gyroTest == false) {

                    competitionEncoderDriveTest(Bot);

                } else {

                    telemetry.addLine("Error, test mode not defined.");

                }

                break;
        }
    }


    public void competitionGyroDriveTest (CompetionBot Bot) throws InterruptedException {

        Bot.driveForwardGyro(0.4, 7);
        sleep(sleepTime);

        Bot.gyroCorrection(0.25, 0);
        sleep(sleepTime);

        Bot.driveBackwardGyro(0.4, 7);
        sleep(sleepTime);

        Bot.gyroCorrection(0.25, 0);
        sleep(sleepTime);

    }

    public void competitionEncoderDriveTest (CompetionBot Bot) {

        Bot.driveForward(0.4, 7);
        sleep(sleepTime);

        Bot.gyroCorrection(0.25, 0);
        sleep(sleepTime);

        Bot.driveBackward(0.4, 7);
        sleep(sleepTime);

        Bot.gyroCorrection(0.25, 0);
        sleep(sleepTime);

    }

}
