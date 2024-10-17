package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.BlueCarousel;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.EightWheelBot;

public abstract class BlueCarousel extends AutoMain {
    public long sleepTime = 100;

    public void CarouselToWarehouseBlue (EightWheelBot Bot) {

        Bot.driveForward(.5,5);
        sleep(sleepTime);

        Bot.rotateLeft(.5, 0.22);
        sleep(sleepTime);

        Bot.driveForward(1,7.5);
        sleep(sleepTime);

    }

}
