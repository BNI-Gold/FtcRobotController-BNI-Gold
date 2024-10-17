package org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Controls.Auto;

import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Robots.CompBot;
import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Vision.TeamPropPosition;
import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Vision.TeamPropPositionPipeline_Gold;

public abstract class AutoBlueAlliance extends AutoMain{

    public CompBot Bot = new CompBot();
    public TeamPropPositionPipeline_Gold pipeline = new TeamPropPositionPipeline_Gold("BLUE", 140);

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

        if (teamPropPosition == TeamPropPosition.BLUE_LEFT) {
            Bot.rotateLeft(0.3,1.1);
            sleep(1000);
            Bot.leftPixelClawOpen();
            sleep(1500);
            Bot.rotateRight(0.3,1.1);
            sleep(1000);
            Bot.leftPixelClawClose();
            sleep(500);
            Bot.driveBack(0.5,1.7);
            sleep(500);
            Bot.driveForward(0.5,0.2);
            sleep(1000);
            Bot.strafeLeft(0.5,5);
        }
        else if (teamPropPosition == TeamPropPosition.BLUE_MIDDLE) {
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
            Bot.strafeLeft(0.5,5);
        }
        else if (teamPropPosition == TeamPropPosition.BLUE_RIGHT){
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
            Bot.strafeLeft(0.5,5);
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
        teamPropPosition = TeamPropPosition.BLUE_MIDDLE;
        if (teamPropPosition == TeamPropPosition.BLUE_LEFT) {
            Bot.rotateLeft(0.3,1.3);
            sleep(1000);
            Bot.driveForward(0.5,0.2);
            sleep(500);
            Bot.leftPixelClawOpen();
            sleep(1500);
            Bot.driveBack(0.5,0.45);
            sleep(100);
            Bot.rotateRight(0.3,1.4);
            sleep(1000);
            Bot.leftPixelClawClose();
            sleep(500);




        }
        else if (teamPropPosition == TeamPropPosition.BLUE_MIDDLE) {

//              CONNOR'S CODE
            Bot.driveForward(0.5,0.8);
            sleep(1000);
            Bot.leftPixelClawOpen();
            sleep(1500);
            Bot.leftPixelClawClose();
            sleep(500);




/*  DUVAL'S CODE
            Bot.strafeRight(0.5, 0.3);
            sleep(100);
            Bot.rotateRight(0.3,1.2);
            sleep(1000);
            Bot.driveForward(0.5, 1.8);
            sleep(100);
            Bot.rotateLeft(0.3, 2);
            sleep(100);

 */

        }
        else if (teamPropPosition == TeamPropPosition.BLUE_RIGHT){
            Bot.strafeRight(0.5, 0.3);
            sleep(100);
            Bot.rotateRight(0.3,1.2);
            sleep(1000);
            Bot.driveForward(0.3,0.2);
            sleep(100);
            Bot.driveBack(0.3,0.1);
            sleep(100);
            Bot.leftPixelClawOpen();
            sleep(1500);
//            Bot.driveBack(0.3,0.1);
            sleep(500);
            Bot.leftPixelClawClose();
            sleep(500);




        }
    }

    public void driveToBackdropFar () {
        if (teamPropPosition == TeamPropPosition.BLUE_LEFT) {
            Bot.driveForward(.5, 3.25);
            sleep(200);
            Bot.rotateLeft(.4, 2.5);
            sleep(100);
            Bot.driveForward(.5, 6);
            sleep(100);

        }
        else if (teamPropPosition == TeamPropPosition.BLUE_MIDDLE) {


        }
        else if (teamPropPosition == TeamPropPosition.BLUE_RIGHT){
            Bot.rotateLeft(0.3,0.7);
            sleep(100);
            Bot.driveBack(0.3, 0.2);
            sleep(100);
            Bot.rotateLeft(0.3, 0.575);
            sleep(100);
            Bot.strafeLeft(0.5, 0.2);
            sleep(100);
            Bot.driveForward(0.5, 3.2);
            sleep(100);
            Bot.rotateLeft(.4, 2.2);
            sleep(100);
            Bot.driveForward(0.5, 6);


        }
    }



}
