package org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.OpMode;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.Robots.CompetionBot;
@Disabled
@TeleOp (name = "Colonel Clap",group = "1")

public class TeleOp_CompetitionBot extends OpMode {

//    FtcDashboard dashboard;

    // (toggles between telemetry for encoders and a big BNI logo!)
    boolean showSeriousTelemetry = true;

    double leftStickYVal;
    double leftStickXVal;
    double rightStickXVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double powerThreshold = 0;

    // below var is used for driver 1's slow mode
    double speedMultiply = 1;

    int turretPosition;

    boolean driveNormal = true;

    boolean turretSlowMode = false;

    boolean driveSlowMode = false;
    
    boolean liftStopAllow = true;
    
    boolean turretStopAllow = true;

    int turretClockwise = 410;
    int turretCounterclocwise = -405;

    int turret90CW = 650;
    int turret90CCW = -650;
    int turretCenter = 0;

    int turret180CW = 1300;
    int turret180CCW = -1300;

    int liftRest = 0;
    int liftLow = 500;
    int liftMid = 1000;
    int liftHigh = 1500;

    boolean turretMoveAllow = false;

    int liftLevel = 0;
    boolean liftLevelAllow = true;
    boolean liftToggle = true;
    boolean turretEncoderCW = false;
    boolean turretEncoderCCW = false;
    boolean turretEncoderCollect = false;
    boolean turrentEncoder180 = false;

    double turretPowerEncoder = 0.3;
    double turretPowerManual = 0.1;
    double liftPowerUp = 1.0;
    double liftPowerDown = 0.2;

    double turretSpeedMultiply = 0.55;

    public DcMotorEx turretPlatform = null;


    public CompetionBot Bot = new CompetionBot();

    @Override
    public void init() {

        Bot.initRobot(hardwareMap);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        driveNormal = true;

        grabberArmControl();

        liftControlManual();
        turretControlManual();

        turretSpeed();
        turretSlowModeControl();

        drive();
        driveSpeed();
        driveSlowModeControl();


//        liftControlEncoder();
//        liftMechanismEncoderNew();
//
//        turretEncoderNew();
//        turretEncoderControlNew();

        updateTelemetry();

    }

    public void updateTelemetry() {

        if (showSeriousTelemetry == true) {

//            dashboard = FtcDashboard.getInstance();
//
//            telemetry = dashboard.getTelemetry();

            telemetry.addLine("Encoder Values: ");

//            telemetry.addData("Turret: ", Bot.turretPlatform.getCurrentPosition());
//
//            telemetry.addData("Lift One: ", Bot.grabberLiftOne.getCurrentPosition());
//            telemetry.addData("Lift Two: ", Bot.grabberLiftTwo.getCurrentPosition());
//            telemetry.addData("Lift Average", Bot.LiftEncoderAvg);

            telemetry.addData("leftFront: ", Bot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("leftRear: ", Bot.rearLeftMotor.getCurrentPosition());
            telemetry.addData("rightFront: ", Bot.frontRightMotor.getCurrentPosition());
            telemetry.addData("rightRear: ", Bot.rearRightMotor.getCurrentPosition());

//            telemetry.addData("Lift Level Selected: ", liftLevel);
//            telemetry.addData("Turret Position Selected: ", turretPosition);

//            telemetry.addData("Lift One Velo: ", Bot.grabberLiftOne.getVelocity());
//            telemetry.addData("Lift Two Velo: ", Bot.grabberLiftTwo.getVelocity());


        } else {

            telemetry.addLine(

                                    " ######     #     #    ### \n" +
                                    " #     #    ##    #     #  \n" +
                                    " #     #    # #   #     #  \n" +
                                    " ######     #  #  #     #  \n" +
                                    " #     #    #   # #     #  \n" +
                                    " #     #    #    ##     #  \n" +
                                    " ######     #     #    ### "

            );

        }
    }

    public void driveSlowModeControl() {

        if (gamepad1.left_trigger >= 0.2 || gamepad1.right_trigger >= 0.2) {

            driveSlowMode = true;

        } else {

            driveSlowMode = false;

        }

    }

    public void driveSpeed() {

        if (driveSlowMode == true) {

            speedMultiply = 0.6;

        } else {

            speedMultiply = 1;

        }

    }

    public void turretSlowModeControl() {

        if (gamepad2.left_trigger >= 0.2) {

            turretSlowMode = true;

        } else {

            turretSlowMode = false;

        }

    }

    public void turretSpeed() {

        if (turretSlowMode == true) {

            turretSpeedMultiply = 0.35;

        } else {

            turretSpeedMultiply = 0.7;

        }

    }


    public void liftMechanismEncoderNew() {
//        Only go to target position when press 'y'.
//        Allows P2 to get lift "target" position ready.
        Bot.LiftEncoderAvg = (Bot.grabberLiftOne.getCurrentPosition() + Bot.grabberLiftTwo.getCurrentPosition()) / 2;
        if (gamepad2.left_stick_button) {
            switch (liftLevel) {
                case 0:
                    if (Bot.grabberLiftOne.getCurrentPosition() > liftRest) {
                        Bot.grabberLiftOne.setPower(liftPowerDown);
                        Bot.grabberLiftTwo.setPower(liftPowerDown);
                    } else {
                        Bot.grabberLiftOne.setPower(liftPowerUp);
                        Bot.grabberLiftTwo.setPower(liftPowerUp);
                    }
                    Bot.grabberLiftOne.setTargetPosition(liftRest);
                    Bot.grabberLiftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    Bot.grabberLiftTwo.setTargetPosition(liftRest);
                    Bot.grabberLiftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;

                case 1:
//                    if (Bot.grabberLiftOne.getCurrentPosition() > liftLow) {
//                        Bot.grabberLiftOne.setPower(liftPowerDown);
//                        Bot.grabberLiftTwo.setPower(liftPowerDown);
//                    } else {
//                        Bot.grabberLiftOne.setPower(liftPowerUp);
//                        Bot.grabberLiftTwo.setPower(liftPowerUp);
//                    }
                    telemetry.addLine("I AM CASE 1");
                    Bot.grabberLiftOne.setPower(liftPowerUp*.5);
                    Bot.grabberLiftTwo.setPower(liftPowerUp*.5);
                    Bot.grabberLiftOne.setTargetPosition(liftLow);
                    Bot.grabberLiftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    Bot.grabberLiftTwo.setTargetPosition(liftLow);
                    Bot.grabberLiftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;

                case 2:
                    if (Bot.grabberLiftOne.getCurrentPosition() > liftMid) {
                        Bot.grabberLiftOne.setPower(liftPowerDown);
                        Bot.grabberLiftTwo.setPower(liftPowerDown);
                    } else {
                        Bot.grabberLiftOne.setPower(liftPowerUp);
                        Bot.grabberLiftTwo.setPower(liftPowerUp);
                    }
                    Bot.grabberLiftOne.setTargetPosition(liftMid);
                    Bot.grabberLiftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    Bot.grabberLiftTwo.setTargetPosition(liftMid);
                    Bot.grabberLiftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;

                case 3:
                    Bot.grabberLiftOne.setPower(liftPowerUp);
                    Bot.grabberLiftOne.setTargetPosition(liftHigh);
                    Bot.grabberLiftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    Bot.grabberLiftTwo.setPower(liftPowerUp);
                    Bot.grabberLiftTwo.setTargetPosition(liftHigh);
                    Bot.grabberLiftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;

                default:
                    break;
            }
        }

//        if (liftLevel == 0 && (Bot.grabberLiftOne.getCurrentPosition() <= 100 || Bot.grabberLiftTwo.getCurrentPosition() <= 100)) {
//
//            Bot.grabberLiftOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            Bot.grabberLiftTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        }

    }

    public void liftControlEncoder() {
        if (gamepad2.b == true && liftLevelAllow == true) {
            if (liftLevel < 3) {
                liftLevel += 1;
            }
            liftLevelAllow = false;
            liftToggle = false;
        } else if (gamepad2.x == true && liftLevelAllow == true) {
            if (liftLevel > 0) {
                liftLevel -= 1;
            }
            liftToggle = false;
            liftLevelAllow = false;
        } else {  // The IF here makes it so lift* goes back to default 'false' ONLY when not pressing Trigger.
            if (gamepad2.b == false && gamepad2.x == false) {
                liftLevelAllow = true;
                liftToggle = true;
            }
        }
    }

    public void grabberArmControl() {
//        if (gamepad2.b) {
//
//            Bot.openGrabberArms();
//
//        }
//
//        if (gamepad2.x) {
//
//            Bot.closeGrabberArms();
//
//        }

        if (gamepad2.a) {

            Bot.coneIntake();
            Bot.intakeGrabberArms();

        } else if (gamepad2.y) {

            Bot.coneOuttake();
            Bot.intakeGrabberArms();

        } else if (gamepad2.x) {

            Bot.closeGrabberArms();
            Bot.intakeStop();

        } else if (gamepad2.b) {

            Bot.intakeStop();
            Bot.openGrabberArms();

        }

    }

    public void liftControlManual() {

        if (gamepad2.left_stick_y >= 0.2) {


            Bot.grabberLiftOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            Bot.grabberLiftOne.setPower(-gamepad2.left_stick_y * 1);

            Bot.grabberLiftTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            Bot.grabberLiftTwo.setPower(-gamepad2.left_stick_y * 1);

            liftStopAllow = true;

        } else if (gamepad2.left_stick_y <= -0.2) {

            Bot.grabberLiftOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Bot.grabberLiftOne.setPower(-gamepad2.left_stick_y * 0.8);

            Bot.grabberLiftTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Bot.grabberLiftTwo.setPower(-gamepad2.left_stick_y * 0.8);

            liftStopAllow = true;

        } else {

            if (liftStopAllow == true) {

                Bot.grabberLiftOne.setPower(0);
                Bot.grabberLiftTwo.setPower(0);

            }

            liftStopAllow = false;

        }

    }

    public void turretControlManual() {

        if (gamepad2.right_stick_x >= 0.2) {

            Bot.turretPlatform.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            Bot.turretPlatform.setPower((gamepad2.right_stick_x * turretSpeedMultiply));

            turretStopAllow = true;

        } else if (gamepad2.right_stick_x <= -0.2) {

            Bot.turretPlatform.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Bot.turretPlatform.setPower((gamepad2.right_stick_x * turretSpeedMultiply));

            turretStopAllow = true;

        } else {

            if (turretStopAllow == true) {

                Bot.turretPlatform.setPower(0);

            }

            turretStopAllow = false;

        }

    }

    public void turretEncoderControlNew() {

        if (gamepad2.dpad_up) {

            turretPosition = 1;

            turretMoveAllow = true;

        } else if (gamepad2.dpad_left) {

            turretMoveAllow = true;

            if (Bot.turretPlatform.getCurrentPosition() < 0) {

                turretPosition = 3;

            } else if (Bot.turretPlatform.getCurrentPosition() >= 0) {

                turretPosition = 4;

            }

        } else {

            turretMoveAllow = false;

        }
    }

    public void turretEncoderNew() {

        if (turretMoveAllow) {
            switch (turretPosition) {
                case 0:
                    if (Bot.turretPlatform.getCurrentPosition() > turret90CCW) {
                        Bot.turretPlatform.setPower(-turretPowerEncoder);
                    } else {
                        Bot.turretPlatform.setPower(turretPowerEncoder);
                    }
                    Bot.turretPlatform.setTargetPosition(turret90CCW);
                    Bot.turretPlatform.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;

                case 1:
                    if (Bot.turretPlatform.getCurrentPosition() > turretCenter) {
                        Bot.turretPlatform.setPower(-turretPowerEncoder);
                    } else {
                        Bot.turretPlatform.setPower(-turretPowerEncoder);
                    }

                    Bot.turretPlatform.setTargetPosition(turretCenter);
                    Bot.turretPlatform.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;

                case 2:
                    if (Bot.turretPlatform.getCurrentPosition() > turret90CW) {
                        Bot.turretPlatform.setPower(-turretPowerEncoder);
                    } else {
                        Bot.turretPlatform.setPower(turretPowerEncoder);
                    }
                    Bot.turretPlatform.setTargetPosition(turret90CW);
                    Bot.turretPlatform.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;

                case 3:

                    if (Bot.turretPlatform.getCurrentPosition() > turret180CCW) {
                        Bot.turretPlatform.setPower(-turretPowerEncoder);
                    } else {
                        Bot.turretPlatform.setPower(turretPowerEncoder);
                    }
                    Bot.turretPlatform.setTargetPosition(turret180CCW);
                    Bot.turretPlatform.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;

                case 4:

                    if (Bot.turretPlatform.getCurrentPosition() > turret180CW) {
                        Bot.turretPlatform.setPower(-turretPowerEncoder);
                    } else {
                        Bot.turretPlatform.setPower(turretPowerEncoder);
                    }
                    Bot.turretPlatform.setTargetPosition(turret180CW);
                    Bot.turretPlatform.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;

                default:
                    break;

            }
        }

    }

    @Override
    public void stop() {
    }

    public void drive() {

        leftStickYVal = -gamepad1.left_stick_y;
        leftStickYVal = Range.clip(leftStickYVal, -1, 1);
        leftStickXVal = gamepad1.left_stick_x;
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
        rightStickXVal = gamepad1.right_stick_x;
        rightStickXVal = Range.clip(rightStickXVal, -1, 1);

        frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
        frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

        frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
        frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

        rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
        rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

        rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
        rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);


        if (frontLeftSpeed <= powerThreshold && frontLeftSpeed >= -powerThreshold) {
            frontLeftSpeed = 0;
            Bot.frontLeftMotor.setPower(frontLeftSpeed * speedMultiply);
        } else {
            Bot.frontLeftMotor.setPower(frontLeftSpeed * speedMultiply);
        }

        if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold) {
            frontRightSpeed = 0;
            Bot.frontRightMotor.setPower(frontRightSpeed * speedMultiply);
        } else {
            Bot.frontRightMotor.setPower(frontRightSpeed * speedMultiply);
        }

        if (rearLeftSpeed <= powerThreshold && rearLeftSpeed >= -powerThreshold) {
            rearLeftSpeed = 0;
            Bot.rearLeftMotor.setPower(rearLeftSpeed * speedMultiply);
        } else {
            Bot.rearLeftMotor.setPower(rearLeftSpeed * speedMultiply);
        }

        if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold) {
            rearRightSpeed = 0;
            Bot.rearRightMotor.setPower(rearRightSpeed * speedMultiply);
        } else {
            Bot.rearRightMotor.setPower(rearRightSpeed * speedMultiply);
        }
    }

    public void driveButton() {
        double buttonPower = 1;
        if (gamepad1.y) {
            Bot.frontLeftMotor.setPower(buttonPower);
            Bot.frontRightMotor.setPower(buttonPower);
            Bot.rearLeftMotor.setPower(buttonPower);
            Bot.rearRightMotor.setPower(buttonPower);
        } else if (gamepad1.a) {
            Bot.frontLeftMotor.setPower(-buttonPower);
            Bot.frontRightMotor.setPower(-buttonPower);
            Bot.rearLeftMotor.setPower(-buttonPower);
            Bot.rearRightMotor.setPower(-buttonPower);
        } else if (gamepad1.x) {
            Bot.frontLeftMotor.setPower(-buttonPower);
            Bot.frontRightMotor.setPower(buttonPower);
            Bot.rearLeftMotor.setPower(buttonPower);
            Bot.rearRightMotor.setPower(-buttonPower);
        } else if (gamepad1.b) {
            Bot.frontLeftMotor.setPower(buttonPower);
            Bot.frontRightMotor.setPower(-buttonPower);
            Bot.rearLeftMotor.setPower(-buttonPower);
            Bot.rearRightMotor.setPower(buttonPower);
        } else {
            Bot.frontLeftMotor.setPower(0);
            Bot.frontRightMotor.setPower(0);
            Bot.rearLeftMotor.setPower(0);
            Bot.rearRightMotor.setPower(0);
        }

    }

}