package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.Tank.BlueShippingHubParkWarehouse;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.Tank.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.TankBot;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.mechanisms.TSELocation;

public abstract class BlueShippingHubPark extends AutoMain {

//BlueShippingHubPark
    private double straightSpd = 0.6;
    private double turnEncoderSpd = 0.5;
    //        Speed .2 == too low for gyro turn
    private double turnGyro1 = 0.25;
    private double turnGyro2 = 0.3;


    public void DriveShippingHubScore (TankBot Bot, String Alliance, TSELocation location) {

        telemetry.addData("RectArea: ", Bot.myPipeline.getRectArea());
        telemetry.addData("Rect Midpoint X", Bot.myPipeline.getRectMidpointX());
        telemetry.addData("Rect Midpoint Y", Bot.myPipeline.getRectMidpointY());
        telemetry.addData("Barcode Location: ", location);
//        telemetry.addData("Barcode: ", Bot.barcode);
//        telemetry.update();
//        telemetry.addData("BARCODE:" , location);
        telemetry.update();
        sleep(1000);
        Bot.webcam.closeCameraDevice();

        switch (location) {


            case barcode1:
//                Bot.pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
//                Bot.blinkinLedDriver.setPattern(Bot.pattern);
//                telemetry.addData("LED: ", Bot.pattern);

                Bot.driveForward(straightSpd, 1.9);
                sleep(1000);
                Bot.LyftExtend();
                sleep(1600);
                Bot.setBoxHolder_Release();
                sleep(100);  //this allows the servo to lower while the motor is still engaged.
                Bot.LyftStopMotors();
                // Motor retracts easily, so may slide back from sleep.
//                sleep(100);
                sleep(500);
                Bot.setBoxHolder_Up();
                sleep(200);
                Bot.LyftRetract();
                sleep(800);
                Bot.LyftStopMotors();
//                Bot.setBoxHolder_Down();
                sleep(sleepTime);
                break;
            case barcode2:
//                Bot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
//                Bot.blinkinLedDriver.setPattern(Bot.pattern);
//                telemetry.addData("LED: ", Bot.pattern);
                Bot.driveForward(straightSpd, 2.8);
                sleep(1000);
                Bot.LyftExtend();
                sleep(700);
                Bot.setBoxHolder_Release();
                sleep(100);  //this allows the servo to lower while the motor is still engaged.
                Bot.LyftStopMotors();
                // Motor retracts easily, so may slide back from sleep.
//                sleep(100);
                sleep(500);
                Bot.setBoxHolder_Up();
                sleep(100);
                Bot.LyftRetract();
                sleep(500);
                Bot.LyftStopMotors();
//                Bot.setBoxHolder_Down();
                sleep(sleepTime);
                break;
            case barcode3:
//                Bot.pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
//                Bot.blinkinLedDriver.setPattern(Bot.pattern);
//                telemetry.addData("LED: ", Bot.pattern);
                Bot.driveForward(straightSpd, 2.9);
                sleep(1000);
                Bot.LyftExtend();
                sleep(475);
                Bot.setBoxHolder_Release();
                sleep(100);  //this allows the servo to lower while the motor is still engaged.
                Bot.LyftStopMotors();
                // Motor retracts easily, so may slide back from sleep.
//                sleep(100);
                sleep(500);
                Bot.setBoxHolder_Up();
                sleep(100);
                Bot.LyftRetract();
                sleep(250);
                Bot.LyftStopMotors();
//                Bot.setBoxHolder_Down();
                sleep(sleepTime);
                break;
            default:
                Bot.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                Bot.blinkinLedDriver.setPattern(Bot.pattern);
                telemetry.addData("LED: ", Bot.pattern);
                break;
        }


    }

    public void ShippingHubToWarehosue (TankBot Bot, String Alliance, TSELocation location) {

        switch (location) {
            case barcode1:
                // THIS IS HERE SO THE BOT CAN GO STRAIGHT OVER BARRIER INSTEAD OF GOING OVER TRIANGLE AND GETTING STUCK!
                Bot.driveForward(straightSpd, 0.4);
                sleep(sleepTime);
                Bot.rotateLeft(turnEncoderSpd, 1.49);
                sleep(sleepTime);
                Bot.driveForward(1);
                sleep(1600);
                Bot.stopMotors();
                sleep(sleepTime);

                break;
            case barcode2:
                // THIS IS HERE SO THE BOT CAN GO STRAIGHT OVER BARRIER INSTEAD OF GOING OVER TRIANGLE AND GETTING STUCK!
                Bot.driveForward(straightSpd, 0.6);
                sleep(sleepTime);
                Bot.rotateLeft(turnEncoderSpd, 1.50);
                sleep(sleepTime);
                Bot.driveForward(1);
                sleep(1700);
                Bot.stopMotors();
                sleep(sleepTime);
                break;
            case barcode3:
                // THIS IS HERE SO THE BOT CAN GO STRAIGHT OVER BARRIER INSTEAD OF GOING OVER TRIANGLE AND GETTING STUCK!
//                Bot.driveForward(straightSpd, 0.2);
                sleep(sleepTime);
                Bot.rotateLeft(turnEncoderSpd, 1.58);
                sleep(sleepTime);
                Bot.driveForward(1);
                sleep(1700);
                Bot.stopMotors();
                sleep(sleepTime);
                break;
            default:
                break;
        }


    }





/*
    public void DriveToShippingHub (TankBot Bot, TSELocation barcode) {


//        switch (barcode) {
//            case barcode1:
//
//        }

        Bot.driveForward(1, 2);
        sleep(sleepTime);
        Bot.rotateLeft(1, 1);
        sleep(sleepTime);
        Bot.driveBackward(1, 3);
        sleep(sleepTime);
        Bot.duckspinclockwise();
        sleep(sleepTime);
        Bot.driveForward(1, 2);
        sleep(sleepTime);
        Bot.rotateLeft(1, 0.5);
        sleep(sleepTime);
        Bot.driveForward(1, 3);
        sleep(sleepTime);
        Bot.rotateRight(1, 1);
        sleep(sleepTime);
        Bot.driveForward(1, 0.5);
        sleep(sleepTime);
        Bot.senseLyftExtend();
        sleep(500);
        Bot.setBoxHolder_Release();
        sleep(1000);
        Bot.setBoxHolder_Up();
        sleep(sleepTime);
        Bot.senseLyftColapse();
        sleep(500);
        Bot.driveBackward(1, 2);
        sleep(sleepTime);
        Bot.rotateLeft(1, 1);
        sleep(sleepTime);
        Bot.driveForward(1, 5);
        sleep(sleepTime);

    }


 */

}
