package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.EightWD.RedWarehouse;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.EightWheelBot;

@Autonomous(name = "Red Warehouse 8WD", group = "Programming Bot")

@Disabled
public class AutoRedWarehouse extends RedWarehouse {

    public EightWheelBot Bot = new EightWheelBot();

    public long sleepTime = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        Bot.initRobot(hardwareMap);
//        Bot.initCamera();
        Bot.setLinearOp(this);



        waitForStart();

        while (opModeIsActive()) {

            driveToWare(Bot);
//
            idle();
            requestOpModeStop();
        }
        idle();
    }
}
