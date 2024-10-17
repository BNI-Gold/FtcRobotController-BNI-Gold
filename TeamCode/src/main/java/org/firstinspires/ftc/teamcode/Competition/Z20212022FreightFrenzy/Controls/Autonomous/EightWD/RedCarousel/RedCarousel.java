package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.RedCarousel;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.EightWheelBot;

public abstract class RedCarousel extends AutoMain {
    public long sleepTime = 100;

    public void CarouselToWarehouseRed (EightWheelBot Bot) {

        Bot.driveForward(.5,5);
        sleep(sleepTime);

        Bot.rotateRight(.5, 0.22);
        sleep(sleepTime);

        Bot.driveForward(1,7.5);
        sleep(sleepTime);

    }
}
