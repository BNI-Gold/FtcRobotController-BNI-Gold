package org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.Robots;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Z20212022FreightFrenzy.DriveTrains.SixWD;

public class SixWheelBot extends SixWD {

    public HardwareMap hwBot = null;


    public CRServo DuckTurnerright = null;
    public CRServo DuckTurnerleft = null;

    public DcMotor LyftExtender;

    public Servo boxHolder2 = null;

    // Higher value = gate higher
    // Lower value == gate goes down more.
    public double boxHolder2up = 0.1 ;

    public double boxHolder2down = 0.4 ;
    public CRServo LyftServo = null;

// ^ !careful while changing! ^

    public DcMotor intake;

    public DcMotor lyft;

    double power;
    double powerControl = 0.8 ;

    public ColorSensor senseLyftColor;

    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    /*
     HUE values for the linear extension slide to know when to stop.
     On testing in BNI lab:

    HUE:
    Blue Gaffers Tape: 220 [closing!]
    Red Gaffers Tape: 20   [opening!]
    No Gaffers Tape: 120

     */
    final double RED_THRESHOLD_HUE = 60;
    final double BLUE_THRESHOLD_HUE = 180;

    //time to close before STOP
    // 1000 == 1 second
    public static final int CLOSE_TIME_THRESHOLD = 500;
    public static final int OPEN_TIME_THRESHOLD = 1000;

    ElapsedTime timer;


    public SixWheelBot() {


    }

    public void initRobot(HardwareMap hardwareMap){
        hwBot = hardwareMap;


        //LyftExtender = hwBot.get(DcMotorEx.class, "Lyft_Extender");

        leftMotorA = hwBot.get(DcMotorEx.class, "left_motor_a");
        leftMotorB = hwBot.get(DcMotorEx.class, "left_motor_b");
        rightMotorA = hwBot.get(DcMotorEx.class, "right_motor_a");
        rightMotorB = hwBot.get(DcMotorEx.class, "right_motor_b");

//        leftMotorA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //LyftExtender.setDirection(DcMotorEx.Direction.FORWARD);

        leftMotorA.setDirection(DcMotorEx.Direction.FORWARD);
        leftMotorB.setDirection(DcMotorEx.Direction.FORWARD);
        rightMotorA.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotorB.setDirection(DcMotorEx.Direction.REVERSE);

//        intakeLyft = hwBot.get(DcMotorEx.class,"Intake_Lyft");
        //intake = hardwareMap.dcMotor.get ("Intake");
//        intake
//        motor = hardwareMap.dcMotor.get("motor");


        //intake.setDirection(DcMotor.Direction.FORWARD);
//        intakeLyft.setDirection(DcMotorEx.Direction.FORWARD);

//        rightMotorA.setDirection(DcMotorSimple.Direction.FORWARD);


//Needed to fix this - only A was here, so "a" was having issues.
//        Commented these out and works:
//        leftMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Causing issues 0 not sure why yet.

        //LyftExtender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftMotorA.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotorA.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intakeLyft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
/*
        DuckTurnerleft = hardwareMap.crservo.get ("Duck_Turner_Left");
        DuckTurnerleft.setDirection(DcMotorSimple.Direction.REVERSE);
        DuckTurnerleft.setPower(0);

        boxHolder2 = hwBot.get(Servo.class,"Box_Holder");
        boxHolder2.setPosition(boxHolder2up);

        DuckTurnerright = hardwareMap.crservo.get ("Duck_Turner_Right");
        DuckTurnerright.setDirection(DcMotorSimple.Direction.REVERSE);
        DuckTurnerright.setPower(0);


        senseLyftColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance_1");

 */

        timer = new ElapsedTime();


    }
/*
    public void setboxHolder2up (){
        boxHolder2.setPosition(boxHolder2up);
    }
    public void setboxHolder2down (){
        boxHolder2.setPosition(boxHolder2down);
    }


    public void SpinDuckBRight(){
        DuckTurnerleft.setPower(-1);
    }
    public void SpinDuckBLeft(){
        DuckTurnerleft.setPower(1);
    }
    public void SpinDuckALeft(){
        DuckTurnerright.setPower(1);
    }
    public void SpinDuckARight(){ DuckTurnerright.setPower(-1); }
    public void StopSpinningDuckRight(){
        DuckTurnerright.setPower(0);
    }
    public void StopSpinningDuckLeft() {DuckTurnerleft.setPower(0);}

    public void Intake (double speed){
        intake.setPower(speed);
    }

    public void LyftUp() {LyftServo.setPower(1);}
    public void LyftDown() {LyftServo.setPower(1);}
    public void Lyft (double speed){
        intake.setPower(speed);
    }

    public void LyftExtend(double speed) {LyftExtender.setPower(speed);}
    public void LyftRetract(double speed) {LyftExtender.setPower(-speed);}

    public void SpinIntake (double speed){
        intake.setPower(1);
    }
    public void StopIntake (double speed){
        intake.setPower(0);
    }
    public void ReverseIntake (double speed){
        intake.setPower(-1);
    }

    public void senseLyftExtend () {
        timer.reset();
        while (timer.milliseconds() < OPEN_TIME_THRESHOLD && linearOp.opModeIsActive()) {
//            Color.RGBToHSV((int) (senseLyftColor.red() * SCALE_FACTOR),
//                    (int) (senseLyftColor.green() * SCALE_FACTOR),
//                    (int) (senseLyftColor.blue() * SCALE_FACTOR),
//                    hsvValues);
            power = powerControl;
            LyftExtender.setPower(power);
            linearOp.telemetry.addLine("OPENING EXTENDER");
            linearOp.telemetry.addData("Hue", hsvValues[0]);
            linearOp.telemetry.addData("TIME (ms)", timer.milliseconds());
            linearOp.telemetry.update();
        }
        LyftExtender.setPower(0);
    }

    public void senseLyftcolapse () {
        timer.reset();
        //was in the compound conditional
//        hsvValues[0] < BLUE_THRESHOLD_HUE &&
        while (timer.milliseconds() < CLOSE_TIME_THRESHOLD && linearOp.opModeIsActive()) {
//            Color.RGBToHSV((int) (senseLyftColor.red() * SCALE_FACTOR),
//                    (int) (senseLyftColor.green() * SCALE_FACTOR),
//                    (int) (senseLyftColor.blue() * SCALE_FACTOR),
//                    hsvValues);
            power = -powerControl;
            LyftExtender.setPower(power);
            linearOp.telemetry.addLine("CLOSING EXTENDER");
            linearOp.telemetry.addData("Hue", hsvValues[0]);
            linearOp.telemetry.addData("TIME (ms)", timer.milliseconds());
            linearOp.telemetry.update();
        }
        LyftExtender.setPower(0);
    }

 */



}
