package org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Controls.Auto;

import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Robots.CompBot;
import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Vision.TeamPropPosition;
import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Vision.TeamPropPositionPipeline_Gold;

public abstract class AutoRedAlliance extends AutoMain{

    public CompBot Bot = new CompBot();

    public TeamPropPositionPipeline_Gold pipeline = new TeamPropPositionPipeline_Gold("RED", 140);

    public void CameraDetection () {
        teamPropPosition = pipeline.getAnalysis();
        telemetry.addData("Position Detected: ", teamPropPosition);
        telemetry.update();
        sleep(1000);




        stopCamera();
        telemetry.addLine("Stopping Camera");
        telemetry.update();
        sleep(1000);

    }


    public void spikeMarkPlaceClose(){
        if (teamPropPosition == TeamPropPosition.RED_LEFT) {
            Bot.rotateLeft(0.3,1.1);
            sleep(1000);
            Bot.leftPixelClawOpen();
            sleep(1500);
            Bot.rotateRight(0.3,1.1);
            sleep(1000);
            Bot.driveBack(0.5,1.7);
            sleep(500);
            Bot.leftPixelClawClose();
            sleep(500);
            Bot.driveForward(0.5,0.2);
            sleep(1000);
            Bot.strafeRight(0.5,5);
        }
        else if (teamPropPosition == TeamPropPosition.RED_MIDDLE) {
            Bot.driveForward(0.5,0.65);
            sleep(1000);
            Bot.leftPixelClawOpen();
            sleep(1500);
            Bot.driveBack(0.5,1.7);
            sleep(500);
            Bot.leftPixelClawClose();
            sleep(500);
            Bot.driveForward(0.5,0.2);
            sleep(1000);
            Bot.strafeRight(0.5,5);
        }
        else if (teamPropPosition == TeamPropPosition.RED_RIGHT){
            Bot.rotateRight(0.3,1.3);
            sleep(1000);
            Bot.driveForward(0.3,0.3);
            sleep(500);
            Bot.leftPixelClawOpen();
            sleep(1500);
            Bot.driveBack(0.5,0.4);
            sleep(500);
            Bot.leftPixelClawClose();
            sleep(500);
            Bot.rotateLeft(0.3,1.3);
            sleep(1000);
            Bot.driveBack(0.5,1.7);
            sleep(500);
            Bot.driveForward(0.5,0.2);
            sleep(1000);
            Bot.strafeRight(0.5,5);
        }
//            else {
//                telemetryUpdate("No Position Detected");
//                Bot.driveForward(0.5,0.55);
//            sleep(1000);
//            Bot.rotateLeft(0.4,2.2);
//            sleep(1000);
//            Bot.driveForward(0.5,3.0);
//            sleep(1000);
////            }

    }



    public void spikeMarkPlaceFar(){
        if (teamPropPosition == TeamPropPosition.RED_LEFT) {
            Bot.rotateLeft(0.3,1.1);
            sleep(1000);
            Bot.leftPixelClawOpen();
            sleep(1500);
            Bot.rotateRight(0.3,1.1);
            sleep(1000);
            Bot.leftPixelClawClose();
            sleep(500);



        }
        else if (teamPropPosition == TeamPropPosition.RED_MIDDLE) {
            Bot.driveForward(0.5,0.7);
            sleep(1000);
            Bot.leftPixelClawOpen();
            sleep(1500);
            Bot.driveBack(0.5,0.3);
            sleep(1000);
            Bot.leftPixelClawClose();
            sleep(500);
//            Bot.strafeLeft(0.5,2.3);
//            sleep(500);
//            Bot.driveForward(0.5,2.7);
//            sleep(500);

        }
        else if (teamPropPosition == TeamPropPosition.RED_RIGHT){
            Bot.rotateRight(0.3,1.3);
            sleep(1000);
            Bot.driveForward(0.3,0.3);
            sleep(500);
            Bot.leftPixelClawOpen();
            sleep(1500);
            Bot.driveBack(0.5,0.4);
            sleep(500);
            Bot.leftPixelClawClose();
            sleep(500);
            Bot.rotateLeft(0.3,1.3);
            sleep(1000);


        }
    }




}
