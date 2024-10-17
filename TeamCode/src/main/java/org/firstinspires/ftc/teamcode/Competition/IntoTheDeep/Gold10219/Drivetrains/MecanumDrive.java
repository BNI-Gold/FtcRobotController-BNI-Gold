package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MecanumDrive {
    //Motor vars
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearLeftMotor;
    public DcMotor rearRightMotor;

    //LinearOpMode required by SDK to run auto
    public LinearOpMode LinearOp = null;

    //Motor ticks per rotation (for encoders in motor)
    public static final double MOTOR_TICKS_PER_ROTATION = 386.3;

    //Odometry ticks per rotation (for encoders in odometry pods)
    public static final double ODO_TICKS_PER_ROTATION = 0;

    //IMU variable for REV Robotics Control Hub Gyroscope
    public IMU imu = null;

    //Heading tolerance for roadrunner
    public double headingTolerance = 2;

    //Current heading for roadrunner
    public double currentHeading = 0;



    //********** AnyDrive Enums **********
    public enum anyDriveDirections {
        STOP, FORWARD, BACKWARD, LEFT, RIGHT
    }
    public enum anyDriveMode {
        LINEAR, ROTATION, DIAGONAL
    }
    //****************************************



    //Default constructor
    public MecanumDrive() {}



    //********** Class Helper Methods **********
    //Assign autonomous linear op mode to drivetrain linear op
    public void setLinearOp(LinearOpMode LinearOp) {this.LinearOp = LinearOp;}

    //Set motor mode (encoder, no encoder, run to position, run to target, etc)
    public void setMotorRunModes(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        rearLeftMotor.setMode(mode);
        rearRightMotor.setMode(mode);
    }

    //Stop all motors
    public void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }
    //****************************************



    //********** AnyDrive Methods **********
    //Basic anyDrive method.
    //Takes speed and direction.
    //Linearly applies direction, so will strafe for LEFT and RIGHT
    //instead of rotate
    public void anyDrive(double speed, anyDriveDirections direction) {
        switch (direction) {
            case FORWARD:
                driveForward(speed);
                break;
            case BACKWARD:
                driveBack(speed);
                break;
            case LEFT:
                strafeLeft(speed);
                break;
            case RIGHT:
                strafeRight(speed);
                break;
        }
    }
    //anyDrive method that takes anyDriveMode parameter as well.
    //Only works for LINEAR or ROTATION. DIAGONAL only works on third
    //overloaded method.
    public void anyDrive(double speed, anyDriveDirections direction, anyDriveMode mode) {
        switch (mode) {
            case LINEAR:
                anyDrive(speed, direction);
                break;
            case ROTATION:
                switch (direction) {
                    case LEFT:
                        rotateLeft(speed);
                        break;
                    case RIGHT:
                        rotateRight(speed);
                        break;
                }
        }
    }
    //anyDrive method that takes anyDriveMode parameter for diagonal strafing.
    //Only works for DIAGONAL.
    //Must pass in FORWARD or BACKWARD for direction1.
    //Must pass in LEFT or RIGHT for direction2.
    public void anyDrive(double speed, anyDriveDirections direction1, anyDriveDirections direction2, anyDriveMode mode) {
        if (mode == anyDriveMode.DIAGONAL) {
            if (direction1 == anyDriveDirections.FORWARD) {
                if (direction2 == anyDriveDirections.LEFT) {
                    diagonalLeftForward(speed);
                } else if (direction2 == anyDriveDirections.RIGHT) {
                    diagonalRightForward(speed);
                }
            } else if (direction1 == anyDriveDirections.BACKWARD) {
                if (direction2 == anyDriveDirections.LEFT) {
                    diagonalLeftBack(speed);
                } else if (direction2 == anyDriveDirections.RIGHT) {
                    diagonalRightBack(speed);
                }
            }
        } else {
            anyDrive(speed, direction1, mode);
        }
    }
    //****************************************



    //********** Basic Drive Methods **********
    //Linear drive methods (Connor take note of respective speed!)
    public void driveForward(double speed) {
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
        rearRightMotor.setPower(speed);
    }
    public void driveBack(double speed) {
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
        rearLeftMotor.setPower(-speed);
        rearRightMotor.setPower(-speed);
    }

    //Rotating methods
    public void rotateLeft(double speed) {
        //Setting left wheels to go backwards; setting right wheels
        //to go forwards
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        rearLeftMotor.setPower(-speed);
        rearRightMotor.setPower(speed);
    }
    public void rotateRight(double speed) {
        //Setting left wheels to go forwards; setting right wheels
        //to go backwards
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        rearLeftMotor.setPower(speed);
        rearRightMotor.setPower(-speed);
    }

    //Strafing methods
    public void strafeLeft(double speed) {
        //Setting front left and rear right wheels to go backwards,
        //setting front right and rear left wheels to go forwards
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
        rearRightMotor.setPower(-speed);
    }
    public void strafeRight(double speed) {
        //Setting front left and rear right wheels to go forwards,
        //setting front right and rear left wheels to go backwards
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        rearLeftMotor.setPower(-speed);
        rearRightMotor.setPower(speed);
    }

    //Diagonal strafing methods
    public void diagonalLeftForward(double speed) {
        frontRightMotor.setPower(-speed);
        rearLeftMotor.setPower(-speed);
    }
    public void diagonalRightForward(double speed) {
        frontLeftMotor.setPower(-speed);
        rearRightMotor.setPower(-speed);
    }
    public void diagonalLeftBack(double speed) {
        frontLeftMotor.setPower(speed);
        rearRightMotor.setPower(speed);
    }
    public void diagonalRightBack(double speed) {
        frontRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
    }
    //****************************************



    //********** Drive Methods with Rotations **********
    //Linear drive with rotations methods
    public void driveForward(double speed, double rotations) {
        double ticks = rotations * MOTOR_TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition()) < ticks && LinearOp.opModeIsActive())) {
            driveForward(speed);
        }
        stopMotors();
    }
    public void driveBack (double speed, double rotations) {
        double ticks = rotations  * MOTOR_TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition() ) < ticks && LinearOp.opModeIsActive() ) ){
            driveBack(speed);
        }
        stopMotors();

    }

    //Rotating with rotations methods
    public void rotateRight(double speed, double rotations) {
        double ticks = rotations * MOTOR_TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition() ) < ticks) && LinearOp.opModeIsActive()) {
            rotateRight(speed);
        }
        stopMotors();
    }
    public void rotateLeft(double speed, double rotations) {
        double ticks = rotations * MOTOR_TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition() ) < ticks ) && LinearOp.opModeIsActive()) {
            rotateLeft(speed);
        }
        stopMotors();


    }

    //Strafing with rotations methods
    public void strafeLeft(double speed, double rotations) {
        double ticks = rotations * MOTOR_TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition() ) < ticks && LinearOp.opModeIsActive()) ){
            strafeLeft(speed);
        }
        stopMotors();
    }
    public void strafeRight(double speed, double rotations) {
        double ticks = rotations * MOTOR_TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition() ) < ticks && LinearOp.opModeIsActive()) ) {
            strafeRight(speed);
        }
        stopMotors();

    }

    //Diagonal strafing with rotations methods
    public void diagonalLeftForward(double speed, double rotations) {

        double ticks = rotations  * MOTOR_TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition() ) < ticks && LinearOp.opModeIsActive()) ) {
            diagonalLeftForward(speed);
        }
        stopMotors();
    }
    public void diagonalRightForward(double speed, double rotations) {

        double ticks = rotations  * MOTOR_TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition() ) < ticks && LinearOp.opModeIsActive()) ) {
            diagonalRightForward(speed);
        }
        stopMotors();
    }
    public void diagonalLeftBack(double speed, double rotations) {

        double ticks = rotations  * MOTOR_TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition() ) < ticks && LinearOp.opModeIsActive()) ) {
            diagonalLeftBack(speed);
        }
        stopMotors();
    }
    public void diagonalRightBack (double speed, double rotations) {

        double ticks = rotations  * MOTOR_TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(frontLeftMotor.getCurrentPosition() ) < ticks && LinearOp.opModeIsActive()) ) {
            diagonalRightBack(speed);
        }
        stopMotors();
    }
    //****************************************



    //********** Methods using IMU & Gyro **********
    //Get IMU Heading
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    //Reset IMU Heading
    public void resetHeading() {imu.resetYaw();}

    //Adjusts heading based on target angle
    public void headingCorrection(double speed, double targetAngle) {
        currentHeading = getHeading();
        if (currentHeading >= targetAngle + headingTolerance && LinearOp.opModeIsActive()) {
            while (currentHeading >= targetAngle + headingTolerance && LinearOp.opModeIsActive()) {
                rotateRight(speed);

                currentHeading = getHeading();
                LinearOp.telemetry.addData("Current Angle: ", currentHeading);
                LinearOp.telemetry.addData("Target Angle: ", targetAngle);
                LinearOp.telemetry.update();
            }
        } else if (currentHeading <= targetAngle - headingTolerance && LinearOp.opModeIsActive()) {
            while (currentHeading <= targetAngle - headingTolerance && LinearOp.opModeIsActive()) {
                rotateLeft(speed);

                currentHeading = getHeading();
                LinearOp.telemetry.addData("Current Angle: ", currentHeading);
                LinearOp.telemetry.addData("Target Angle: ", targetAngle);
                LinearOp.telemetry.update();
            }
        }
        stopMotors();
        currentHeading = getHeading();
    }
    //****************************************
}
