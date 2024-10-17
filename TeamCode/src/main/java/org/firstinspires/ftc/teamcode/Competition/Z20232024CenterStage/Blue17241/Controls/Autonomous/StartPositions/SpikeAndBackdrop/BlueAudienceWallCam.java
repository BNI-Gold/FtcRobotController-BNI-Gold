package org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Blue17241.Controls.Autonomous.StartPositions.SpikeAndBackdrop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Blue17241.Controls.Autonomous.AutoBlueAlliance;
@Disabled
@Autonomous (name = "Blue:Audience:Wall:Cam")
public class BlueAudienceWallCam extends AutoBlueAlliance {
    @Override
    public void runOpMode() throws InterruptedException {

        Bot.initRobot(hardwareMap);
        initCamera();
        Bot.setLinearOp(this);
        startPipeline(pipeline);
//        telemetry.addLine("Turning on Camera, please wait...");
//        telemetry.update();
//        sleep(1000);
//        CameraDetection();
//        sleep(3000);
//        telemetry.addData("POSITION: ", propPosition);
//        telemetry.addLine("WAITING FOR START >");
//        telemetry.update();
        waitForStart();
        Bot.resetHeading();

        while (opModeIsActive()) {

            CameraDetection();
            //propPosition = TeamPropPosition.THREE;
            positionToDropBlueAudWall();


            requestOpModeStop();

        }
        idle();
    }
}
