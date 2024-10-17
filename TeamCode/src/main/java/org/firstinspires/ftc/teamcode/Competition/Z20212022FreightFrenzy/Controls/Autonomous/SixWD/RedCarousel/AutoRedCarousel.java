package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.SixWD.RedCarousel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.SixWheelBot;

@Autonomous(name = "Red Carousel", group = "Programming Bot")
@Disabled

public class AutoRedCarousel extends RedCarousel {


    public SixWheelBot Bot = new SixWheelBot();

    public long sleepTime = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        Bot.initRobot(hardwareMap);
//        Bot.initCamera();
        Bot.setLinearOp(this);



        waitForStart();

        while (opModeIsActive()) {

           //red_carousel_spinDuck(Bot);

            Bot.driveForward(.5,5   );
            sleep(sleepTime);

            Bot.rotateRight(.5, 0.22);
            sleep(sleepTime);

            Bot.driveForward(1,7.5);
            sleep(sleepTime);

            idle();
            requestOpModeStop();

        }
        idle();
    }
}
