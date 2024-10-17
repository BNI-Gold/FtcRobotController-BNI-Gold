package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.SixWD.BlueWarehouse;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.SixWheelBot;

@Autonomous(name = "Blue Warhouse 6WD", group = "Programming Bot")
@Disabled

public class AutoBlueWarehouse extends BlueWarehouse {

    public SixWheelBot Bot = new SixWheelBot();

    public long sleepTime = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        Bot.initRobot(hardwareMap);
//        Bot.initCamera();
        Bot.setLinearOp(this);



        waitForStart();

        while (opModeIsActive()) {

            Bot.driveForward(.5,2.4);
            sleep(sleepTime);

            Bot.rotateLeft(.5,1.5);
            sleep(sleepTime);

            Bot.driveForward(.5,6.5);
            sleep(sleepTime);

            idle();
            requestOpModeStop();
        }
        idle();
    }
}
