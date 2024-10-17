package org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Blue17241.Controls.Autonomous.StartPositions.SpikeAndBackdrop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Blue17241.Controls.Autonomous.AutoRedAlliance;

@Disabled
@Autonomous(name = "Red:Audience:Wall:Cam")
public class RedAudienceWallCam extends AutoRedAlliance {
    @Override
    public void runOpMode() throws InterruptedException {
        Bot.initRobot(hardwareMap);
        initCamera();
        Bot.setLinearOp(this);
        startPipeline(pipeline);

        waitForStart();
        Bot.resetHeading();

        while(opModeIsActive()){
            CameraDetection();
            //propPosition = TeamPropPosition.FOUR;
            positionToDropRedAudWall();

            requestOpModeStop();
        }
        idle();
    }
}
