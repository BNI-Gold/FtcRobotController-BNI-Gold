package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.BlueShippingHub;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.EightWheelBot;

public abstract class BlueShippingHubTop extends AutoMain {
    public long sleepTime = 100;

    public void CarouselToSHubBlue (EightWheelBot Bot) {
        Bot.driveBackward(0.5, 0.5);
        sleep(sleepTime);

        Bot.rotateLeft(1, 1);
        sleep(sleepTime);
//
//        Bot.driveForward(0.5, 0.75);
//        sleep(sleepTime);
//
//        Bot.rotateRight(0.5, 0.75);
//        sleep(sleepTime);
//
//        Bot.driveForward(0.5, 0.5);
//        sleep(sleepTime);
    }

    public void SHubToWarehouseBlue (EightWheelBot Bot) {
        Bot.driveBackward(0.5, 1);
        sleep(sleepTime);

        Bot.rotateLeft(0.5, 1);
        sleep(sleepTime);

        Bot.driveForward(1, 7);
        sleep(sleepTime);
    }


}
