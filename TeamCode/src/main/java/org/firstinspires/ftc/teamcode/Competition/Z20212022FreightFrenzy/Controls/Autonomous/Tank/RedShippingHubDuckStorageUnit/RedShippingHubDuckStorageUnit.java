package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.Tank.RedShippingHubDuckStorageUnit;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Controls.Autonomous.Tank.AutoMain;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots.TankBot;
import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.mechanisms.TSELocation;

public abstract class RedShippingHubDuckStorageUnit extends AutoMain {

    // Red
    private double straightSpd = 0.6;
    private double turnEncoderSpd = 0.5;
    //        Speed .2 == too low for gyro turn
    private double turnGyro1 = 0.25;
    private double turnGyro2 = 0.3;


    private double ShippingHubDistance = 2.3;


    public void DriveShippingHubScore (TankBot Bot, String Alliance, TSELocation location) {

        telemetry.addData("RectArea: ", Bot.myPipeline.getRectArea());
        telemetry.addData("Rect Midpoint X", Bot.myPipeline.getRectMidpointX());
        telemetry.addData("Rect Midpoint Y", Bot.myPipeline.getRectMidpointY());
        telemetry.addData("Barcode Location: ", location);
//        telemetry.addData("Barcode: ", Bot.barcode);
        telemetry.update();
        sleep(1000);
        Bot.webcam.closeCameraDevice();


        switch (location) {
            case barcode1:
//                Bot.pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
//                Bot.blinkinLedDriver.setPattern(Bot.pattern);
//                telemetry.addData("LED: ", Bot.pattern);


                Bot.driveForward(straightSpd, ShippingHubDistance);  //2.0 for variable distances

                // just commented out below so not raising lift every time.

                sleep(1000);
                Bot.LyftExtend();
                sleep(1600);  // TIME TO EXTEND LYFT - 1600 for var distances
                Bot.setBoxHolder_Release();
                sleep(100);  //this allows the servo to lower while the motor is still engaged.
                Bot.LyftStopMotors();
                // Motor retracts easily, so may slide back from sleep.
//                sleep(100);
                sleep(500);
                Bot.setBoxHolder_Up();
                sleep(100);
                Bot.LyftRetract();
                sleep(800);
                Bot.LyftStopMotors();
                Bot.setBoxHolder_Down();




                sleep(sleepTime);

//                Bot.driveForward(straightSpd, 0.7);   // forward 0.7 for var distances

                break;
            case barcode2:
//                Bot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
//                Bot.blinkinLedDriver.setPattern(Bot.pattern);
//                telemetry.addData("LED: ", Bot.pattern);
                Bot.driveForward(straightSpd, ShippingHubDistance);   // 2.8 for variable distances
                sleep(1000);
                Bot.LyftExtend();
                sleep(750);  // TIME TO EXTEND LYFT - 700 for var distances
                Bot.setBoxHolder_Release();
                sleep(100);  //this allows the servo to lower while the motor is still engaged.
                Bot.LyftStopMotors();
                // Motor retracts easily, so may slide back from sleep.
//                sleep(100);
                sleep(500);
                Bot.setBoxHolder_Up();
                sleep(100);
                Bot.LyftRetract();
                sleep(700);
                Bot.LyftStopMotors();
//                Bot.setBoxHolder_Down();
                sleep(sleepTime);
//                Bot.driveBackward(straightSpd, 0.2); // DO NOT USE FOR ANYTHING.  barcade 2 is the "control"
                sleep(sleepTime);

                break;
            case barcode3:
//                Bot.pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
//                Bot.blinkinLedDriver.setPattern(Bot.pattern);
//                telemetry.addData("LED: ", Bot.pattern);
                Bot.driveForward(straightSpd, ShippingHubDistance);   //2.6 for var distances
                sleep(1000);
                Bot.LyftExtend();
                sleep(550);  // TIME TO EXTEND LYFT - 550 for var distances
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
//                Bot.driveForward(straightSpd, 0.18);  // forward 1.8 for var distances
                sleep(sleepTime);

                break;
            default:
                Bot.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                Bot.blinkinLedDriver.setPattern(Bot.pattern);
                telemetry.addData("LED: ", Bot.pattern);
                break;
        }
        telemetry.addData("BARCADE: ", location);
        telemetry.update();
    }

    public void ShippingHubToDuck (TankBot Bot, String Alliance, TSELocation location) {

        Bot.rotateRight(turnEncoderSpd, .9);
        sleep(sleepTime);
        Bot.gyroCorrection(turnGyro1, -90);
        sleep(sleepTime);
        Bot.driveBackward(straightSpd, 5.2);
        sleep(sleepTime);
        Bot.rotateLeft(turnEncoderSpd, 0.8);
        sleep(sleepTime);
        Bot.gyroCorrection(turnGyro1, -45);
        sleep(sleepTime);

//            Drive to duck spinner.  Should be at it after this.
        Bot.driveBackward(0.4, 0.6, 1500);
        sleep(sleepTime);
        Bot.driveBackward(0.2, 0.3, 800);
        sleep(sleepTime);
        Bot.driveBackward(0.15, 0.2, 500);
        sleep(sleepTime);
        Bot.driveForward(0.15, .03);   //SPINNY IS GETTING CAUGHT ON DUCK SPINNER.  THIS IS TO GIVE A LITTLE SLACK.
        sleep(sleepTime);
        switch (location) {
            case barcode1:

                break;
            case barcode2:

                break;
            case barcode3:
                break;
        }


    }

    public void DuckSpinnerToStorageUnit (TankBot Bot) {
        Bot.driveForward(straightSpd, 0.4);
        sleep(sleepTime);
        Bot.rotateLeft(turnEncoderSpd, 1.1);
        sleep(sleepTime);
        Bot.driveForward(straightSpd, 2.7);
        sleep(sleepTime);
        Bot.rotateRight(turnEncoderSpd, .18);
        sleep(sleepTime);
        Bot.driveForward(straightSpd, 1.2);
        sleep(sleepTime);
    }
}
