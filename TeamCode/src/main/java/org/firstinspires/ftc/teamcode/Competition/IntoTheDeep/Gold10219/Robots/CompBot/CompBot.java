//This is the primary bot, hence it only being named 'Bot'. All other bots should use a name
//referencing what is special about them, i.e. AckerBot or ProgrammingBot or StrafeBot

package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Drivetrains.MecanumDrive;

public class CompBot extends MecanumDrive {

    public HardwareMap hwBot = null;
    public CompBotVars vars = new CompBotVars();

    public CompBot() {}

    public void initRobot(HardwareMap hwMap) {
        hwBot = hwMap;

        //********** DRIVETRAIN CONFIG **********
        //Get motor ports & info from control hub configuration
        frontLeftMotor = hwBot.dcMotor.get(vars.Motors.FrontLeft.name);
        frontRightMotor = hwBot.dcMotor.get(vars.Motors.FrontRight.name);
        rearLeftMotor = hwBot.dcMotor.get(vars.Motors.RearLeft.name);
        rearRightMotor = hwBot.dcMotor.get(vars.Motors.RearRight.name);

        //Assign direction to motors
        frontLeftMotor.setDirection(vars.Motors.FrontLeft.direction);
        frontRightMotor.setDirection(vars.Motors.FrontRight.direction);
        rearLeftMotor.setDirection(vars.Motors.RearLeft.direction);
        rearRightMotor.setDirection(vars.Motors.RearRight.direction);

        //Initialize motor run mode
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set motor zero power behavior
        frontLeftMotor.setZeroPowerBehavior(vars.Motors.FrontLeft.zeroPowerBehavior);
        frontRightMotor.setZeroPowerBehavior(vars.Motors.FrontRight.zeroPowerBehavior);
        rearRightMotor.setZeroPowerBehavior(vars.Motors.RearLeft.zeroPowerBehavior);
        rearLeftMotor.setZeroPowerBehavior(vars.Motors.RearRight.zeroPowerBehavior);
        //****************************************

        //********** INIT IMU **********
        RevHubOrientationOnRobot orientationOnRobot = vars.IMU.orientationOnRobot;

        imu = hwBot.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //****************************************
    }

}
