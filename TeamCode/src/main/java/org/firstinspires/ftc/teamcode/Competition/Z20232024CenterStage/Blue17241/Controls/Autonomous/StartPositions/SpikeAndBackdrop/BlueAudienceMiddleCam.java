package org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Blue17241.Controls.Autonomous.StartPositions.SpikeAndBackdrop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Blue17241.Controls.Autonomous.AutoBlueAlliance;
@Disabled
@Autonomous (name = "Blue:Audience:Middle:Cam")
public class BlueAudienceMiddleCam extends AutoBlueAlliance {
    @Override
    public void runOpMode() throws InterruptedException {

        Bot.initRobot(hardwareMap);
        initCamera();
        Bot.setLinearOp(this);
        startPipeline(pipeline);
        sleep(5000);
        telemetry.addData("POSITION:", propPosition);
        telemetry.update();
        waitForStart();
        Bot.resetHeading();

        while (opModeIsActive()) {

            CameraDetection();
            //propPosition = TeamPropPosition.ONE;
            positionToDropBlueAudMiddle();


            requestOpModeStop();

        }
        idle();
    }
}
