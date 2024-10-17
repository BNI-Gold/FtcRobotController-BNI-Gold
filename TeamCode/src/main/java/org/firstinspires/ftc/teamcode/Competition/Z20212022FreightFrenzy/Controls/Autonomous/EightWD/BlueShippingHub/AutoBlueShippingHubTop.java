package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.BlueShippingHub;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.EightWheelBot;

@Autonomous(name = "Blue Carousel Shipping Hub 8WD", group = "Programming Bot")
@Disabled
public class AutoBlueShippingHubTop extends BlueShippingHubTop {


    public EightWheelBot Bot = new EightWheelBot();

    public long sleepTime = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        Bot.initRobot(hardwareMap);
//        Bot.initCamera();
        Bot.setLinearOp(this);


        waitForStart();

        while (opModeIsActive()) {
            Bot.boxHolder2.setPosition(Bot.boxHolder2up);



            blue_carousel_spinDuck(Bot);

//            CarouselToSHubBlue(Bot);
//
//            Shipping_Hub_Score(Bot);
//
//            SHubToWarehouseBlue(Bot);
//                                                                                                                                                                        too good
            idle();
            requestOpModeStop();

            idle();

            requestOpModeStop();

        }
    }
}
