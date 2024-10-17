package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.RedShippingHub;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.EightWheelBot;

public abstract class RedShippingHub extends AutoMain {
    public long sleepTime = 100;


    //public void Example (EightWheelBot Bot) {
    //    Bot.driveForward(1, 1);
    //    sleep(sleepTime);
    //}

    public void CarouselToSHubRed (EightWheelBot Bot) {
        Bot.driveForward(0.5, 0.75);
        sleep(sleepTime);

        Bot.rotateRight(0.5, 0.5);
        sleep(sleepTime);

        Bot.driveForward(0.5, 0.75);
        sleep(sleepTime);

        Bot.rotateLeft(0.5, 0.75);
        sleep(sleepTime);

        Bot.driveForward(0.5, 0.5);
        sleep(sleepTime);
    }

    public void SHubToWarehouseRed (EightWheelBot Bot) {
        Bot.driveBackward(0.5, 1);
        sleep(sleepTime);

        Bot.rotateRight(0.5, 1);
        sleep(sleepTime);

        Bot.driveForward(1, 7);
        sleep(sleepTime);
    }


        // Drive forward to the carousel

    }

