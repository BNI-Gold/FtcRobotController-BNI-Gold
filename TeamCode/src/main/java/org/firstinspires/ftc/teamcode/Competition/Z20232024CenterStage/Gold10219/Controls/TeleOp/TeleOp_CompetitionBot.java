package org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Controls.TeleOp;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Competition.Z20232024CenterStage.Gold10219.Robots.CompBot;

import java.util.Locale;

@Disabled
@TeleOp (name = "A - Center Stage - 'RANGER RATTLE'")
public class TeleOp_CompetitionBot extends OpMode {

   // FtcDashboard dashboard = FtcDashboard.getInstance();
   // Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public double rotationPos = 0.5;
    double incValue = 0.05;

    boolean planeLauncherOn = false;

    double pixelRotationUp = 1.0;

    double pixelRotationMiddle = 0.5;

    double pixelRotationDown = 0.0;

    double leftStickYVal;
    double leftStickXVal;
    double rightStickXVal;
    double rightStickYVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double powerThreshold = 0;
    double speedMultiply = 1;

    public double wormgearPower = 1;

    public double viperSlidePower = 0.65;

    public double viperSlideMaxTicks = 250;
    public double viperSlideMinTicks = 1;
    public double wormgearMaxTicks = 100;
    public double wormgearMinTicks = 1;

    public boolean slowMode = false;

    public boolean variableSlowMode = false;


    public CompBot Bot = new CompBot();

    ElapsedTime timer = new ElapsedTime();

    public void init() {
        Bot.initRobot(hardwareMap);
    }

    public void init_loop() {
    }

    public void start() {}

    public void loop() {
        speedControl();
        endgameArm();
        pixelMechanismControl();
        LEDControl();
        planeLauncher();
        drive();
        telemetryOutput();
    }

    public void speedControl() {

        if (gamepad1.left_trigger > 0.35) {

            slowMode = true;

        } else {

            slowMode = false;

        }

        if (gamepad1.right_trigger > 0.1) {

            variableSlowMode = true;

        } else {

            variableSlowMode = false;

        }

        if (slowMode) {

            speedMultiply = 0.3;

        } else if (variableSlowMode) {

            //experimental
            speedMultiply = gamepad1.right_trigger / 0.8;

        } else {

            speedMultiply = 1;

        }

    }

    public void drive() {

        leftStickYVal = gamepad1.left_stick_y;
        leftStickYVal = Range.clip(leftStickYVal, -1, 1);
        leftStickXVal = -gamepad1.left_stick_x;
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
        rightStickXVal = -gamepad1.right_stick_x;
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
            Bot.frontLeftMotor.setPower(frontLeftSpeed);
        } else {
            Bot.frontLeftMotor.setPower(frontLeftSpeed * speedMultiply);
        }

        if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold) {
            frontRightSpeed = 0;
            Bot.frontRightMotor.setPower(frontRightSpeed);
        } else {
            Bot.frontRightMotor.setPower(frontRightSpeed * speedMultiply);
        }

        if (rearLeftSpeed <= powerThreshold && rearLeftSpeed >= -powerThreshold) {
            rearLeftSpeed = 0;
            Bot.rearLeftMotor.setPower(rearLeftSpeed);
        } else {
            Bot.rearLeftMotor.setPower(rearLeftSpeed * speedMultiply);
        }

        if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold) {
            rearRightSpeed = 0;
            Bot.rearRightMotor.setPower(rearRightSpeed);
        } else {
            Bot.rearRightMotor.setPower(rearRightSpeed * speedMultiply);
        }
    }

    public void pixelMechanismControl() {

        boolean hangPos = false;
//       if (gamepad2.y){
//
//           Bot.collectorPosition();
//
//       } else if (gamepad2.b) {
//
//            Bot.automousPosition();
//
//        } else {
//
//           Bot.drivePosition();
//
//       }




        if (gamepad2.a) {
            Bot.collectorPosition();
        }
       // else if (gamepad2.y) {
        //   Bot.hangPosition();
       // }
        else if (gamepad2.b) {
            Bot.tuckPosition();
        }
        else if (gamepad2.x){
            Bot.drivePosition();
        }



//       if (gamepad2.a) {
//
//           Bot.drivePosition();
//
//       }

        if (gamepad2.right_trigger > 0.2) {
            Bot.linearSlideExtend(viperSlidePower * 1.0);
        } else if (gamepad2.left_trigger > 0.2) {
            Bot.linearSlideRetract(viperSlidePower * 0.6);
        } else {
            Bot.stopLinearSlide();
        }

//        if (Math.abs(Bot.viperSlideRight.getCurrentPosition()) > viperSlideMaxTicks) {
//            Bot.viperSlideRight.setPower(0);
//        } else if (Math.abs(Bot.viperSlideRight.getCurrentPosition()) <= viperSlideMinTicks) {
//            Bot.viperSlideRight.setPower(0);
//        }
        if (gamepad2.left_stick_y < -0.1) {
            Bot.rightWormgearDown(wormgearPower * 0.9);
        } else if (gamepad2.left_stick_y > 0.1) {
            Bot.rightWormgearUp(wormgearPower * 0.6);
        } else {
            Bot.wormgearRight.setPower(0);
        }


        if (gamepad2.left_bumper) {

            Bot.leftPixelClawOpen();

        } else {

            Bot.leftPixelClawClose();

        }

        if (gamepad2.right_bumper) {
            Bot.rightPixelClawClose();
        }
        else {
            Bot.rightPixelClawOpen();
        }


    }

    public void LEDControl () {
        if (Bot.pixelDistanceSensor1.getDistance(DistanceUnit.INCH) > 0.8) {
            Bot.leftPixelLEDNone();
        }
        else if (Bot.pixelDistanceSensor1.getDistance(DistanceUnit.INCH) > 0.4 && Bot.pixelDistanceSensor1.getDistance(DistanceUnit.INCH) < 0.8) {
            Bot.leftPixelLEDIn();
        }
        else if (Bot.pixelDistanceSensor1.getDistance(DistanceUnit.INCH) < 0.4) {
            Bot.leftPixelLEDCaptured();
        }


        if (Bot.pixelDistanceSensor2.getDistance(DistanceUnit.INCH ) > 0.8) {
            Bot.rightPixelLEDNone();
        }
        else if (Bot.pixelDistanceSensor2.getDistance(DistanceUnit.INCH) > 0.4 && Bot.pixelDistanceSensor2.getDistance(DistanceUnit.INCH) < 0.8) {
            Bot.rightPixelLEDIn();
        }
        else if (Bot.pixelDistanceSensor2.getDistance(DistanceUnit.INCH) < 0.4) {
            Bot.rightPixelLEDCaptured();
        }

    }

    public void endgameArm() {

        if (gamepad2.right_stick_y < -0.1) {
            Bot.endgameArmRetract();
        } else if (gamepad2.right_stick_y > 0.1) {
            Bot.endgameArmExtend();
        } else {
            Bot.endgameArmStop();
        }

        /*if (gamepad2.x) {
           Bot.endgameArmRotator.setDirection(Servo.Direction.REVERSE);
           Bot.endgameArmRotator.setPosition(0.1);
//        } else if (gamepad2.dpad_right) {
//            Bot.endgameArmRotator.setPosition(0.4);
        } else if (gamepad2.b) {
            Bot.endgameArmRotator.setDirection(Servo.Direction.FORWARD);
            Bot.endgameArmRotator.setPosition(0.1);
        }
        else if (gamepad2.back) {
            Bot.endgameArmRotator.setPosition(0.5);
        }*/


//        if (gamepad2.a) {
//            CompetitionBot.upTimer.reset();
//            if (CompetitionBot.upTimer.seconds() >= 2.5) {
//                CompetitionBot.endgameArmStop();
//            } else if (CompetitionBot.upTimer.seconds() < 2.5) {
//                CompetitionBot.endgameArmExtend();
//
//            }
//
//        } else if (gamepad2.b) {
//            CompetitionBot.downTimer.reset();
//            if (CompetitionBot.downTimer.seconds() >= 2.4) {
//                CompetitionBot.endgameArmStop();
//            } else if (CompetitionBot.downTimer.seconds() < 2.4) {
//                CompetitionBot.endgameArmRetract();
//            }
//        }


    }

    public void planeLauncher() {

        if (gamepad2.dpad_down) {
            Bot.planeLauncherServo.setPosition(1);//launch
        }
        else if (gamepad2.dpad_up) {
            Bot.planeLauncherServo.setPosition(0);   //reset
        }
    }
        public void telemetryOutput () {

           // dashboardTelemetry.addData("worm gear encoder: ", Bot.wormgearRight.getCurrentPosition());

            telemetry.addData("Front Left: ", Bot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right: ", Bot.frontRightMotor.getCurrentPosition());
            telemetry.addData("Rear Left: ", Bot.rearLeftMotor.getCurrentPosition());
            telemetry.addData("Rear Right: ", Bot.rearRightMotor.getCurrentPosition());

            telemetry.addLine("");

            telemetry.addData("DPAD SERVO ", Bot.endgameArmRotator.getPosition());

          //  dashboardTelemetry.addData("FRONT LEFT: ", Bot.frontLeftMotor.getPower());
          //  dashboardTelemetry.addData("FRONT RIGHT: ", Bot.frontRightMotor.getPower());

          //  dashboardTelemetry.addData("REAR LEFT: ", Bot.rearLeftMotor.getPower());
         //   dashboardTelemetry.addData("REAR RIGHT: ", Bot.rearRightMotor.getPower());

//            dashboardTelemetry.addData("worm gear encoder: ", Bot.wormgearRight.getCurrentPosition());
         //   dashboardTelemetry.update();

            telemetry.addData("Distance 1 (in)",
                    String.format(Locale.US, "%.02f", Bot.pixelDistanceSensor1.getDistance(DistanceUnit.INCH)));

            telemetry.addData("Distance 2 (in)",
                    String.format(Locale.US, "%.02f", Bot.pixelDistanceSensor2.getDistance(DistanceUnit.INCH)));

            telemetry.update();
        }


    }



