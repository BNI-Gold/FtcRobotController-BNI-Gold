package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.SixWD.BlueShippingHub;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.SixWheelBot;

@Autonomous(name = "Blue Carousel Shipping Hub 6WD", group = "Programming Bot")
@Disabled

public class AutoBlueShippingHubTop extends BlueShippingHubTop {


    public SixWheelBot Bot = new SixWheelBot();

    public long sleepTime = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        Bot.initRobot(hardwareMap);
//        Bot.initCamera();
        Bot.setLinearOp(this);


        waitForStart();

        while (opModeIsActive()) {
            Bot.boxHolder2.setPosition(Bot.boxHolder2up);


            //blue_carousel_spinDuck(Bot);
//                                                                                                                                                                        prosciutto
//             ^ in AutoMain ^

            Bot.driveForward(0.5, 0.5);
            sleep(sleepTime);

            Bot.rotateLeft(0.5, 0.2);
            sleep(sleepTime);

            Bot.driveForward(0.6, 4.2);
            sleep(sleepTime);

            Bot.rotateRight(0.5, 1.35);
            sleep(sleepTime);

            Bot.driveForward(0.6, 1.5);
            sleep(sleepTime);

            //Shipping_Hub_Score (Bot);

            Bot.driveBackward(0.6, 0.35);
            sleep(sleepTime);

            Bot.rotateLeft(0.5, 1.15);
            sleep(sleepTime);

            Bot.driveForward(0.6, 1.2);
            sleep(sleepTime);

            Bot.rotateLeft(0.5, 0.18);
            sleep(sleepTime);

            Bot.driveForward(1, 7);
            sleep(sleepTime);

            Bot.driveForward(0.3, 3);
            sleep(sleepTime);




            idle();
            requestOpModeStop();

            idle();

            requestOpModeStop();

        }
    }
}
