package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.BlueWarehouse;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.EightWheelBot;

public abstract class BlueWarehouse extends AutoMain {
    public long sleepTime = 100;

//    public void DriveToWarehouse (EightWheelBot Bot) {
//        Bot.driveForward(1,1.45);
//        sleep(sleepTime);
//
//        Bot.rotateLeft(1,1.3);
//        sleep(sleepTime);
//
//        Bot.driveForward(1,6);
//        sleep(sleepTime);
//    }

    public void driveShippingHub (EightWheelBot Bot) {
        Bot.driveBackward(.5,1.4);
        sleep(sleepTime);
    }

    public void shipToWare (EightWheelBot Bot){

        Bot.rotateRight(1,1.63);
        sleep(sleepTime);

        Bot.driveForward(.75,7);
        sleep(sleepTime);
    }

}
