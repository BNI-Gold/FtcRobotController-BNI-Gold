package org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Blue17241.Controls.Autonomous.StartPositions.NoCamera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Blue17241.Controls.Autonomous.AutoRedAlliance;

@Disabled
@Autonomous(name="Red:Backstage:Start")
public class RedBackstageStart extends AutoRedAlliance {

    @Override
    public void runOpMode() throws InterruptedException{
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);

        Bot.closePixelClaw();

        telemetry.addLine("Awaiting Start");
        telemetry.update();



        waitForStart();

        while(opModeIsActive()){


//            Bot.driveForward(.5,4);
//            Bot.extendPixelArm(.5);
//            sleep(500);
//            Bot.stopPixelArm();
//            Bot.openPixelClaw();

            requestOpModeStop();

        }

        idle();
    }

}
