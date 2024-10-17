package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.RedWarehouse;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.EightWheelBot;

public abstract class RedWarehouse extends AutoMain {
    public long sleepTime = 100;

    public void driveToWare (EightWheelBot Bot) {
        Bot.driveForward(1,1.45);
        sleep(sleepTime);

        Bot.rotateLeft(1,1.3);
        sleep(sleepTime);

        Bot.driveForward(1,6);
        sleep(sleepTime);

    }



}
