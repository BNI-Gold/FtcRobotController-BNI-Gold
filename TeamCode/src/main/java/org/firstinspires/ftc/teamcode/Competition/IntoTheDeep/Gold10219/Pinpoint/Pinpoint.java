package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pinpoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Pinpoint {
    public HardwareMap hwBot = null;
    public PinpointDriver pinpoint = null;
    public LinearOpMode LinearOp = null;
    public PinpointVars vars = new PinpointVars();
    public Telemetry telemetry = null;

    public Pinpoint() {
    }

    public void setLinearOp(LinearOpMode LinearOp) {
        this.LinearOp = LinearOp;
        telemetry = LinearOp.telemetry;
    }

    public void initPinpoint(HardwareMap hwMap) {
        hwBot = hwMap;

        pinpoint = hwBot.get(PinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(vars.Offsets.x, vars.Offsets.y);
        pinpoint.setEncoderResolution(vars.resolution);
        pinpoint.setEncoderDirections(vars.Directions.x, vars.Directions.y);

        pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", pinpoint.getXOffset());
        telemetry.addData("Y offset", pinpoint.getYOffset());
        telemetry.addData("Device Version Number:", pinpoint.getDeviceVersion());
        telemetry.addData("Device Scalar", pinpoint.getYawScalar());
        telemetry.update();
    }

    public void update() {
        pinpoint.update();
    }

    public Pose2D getPosition() {
        return pinpoint.getPosition();
    }

    public void updatePosition(Pose2D position) {
        pinpoint.setPosition(position);
    }

    public void updateXYPosition(double x, double y) {
        Pose2D oldPosition = getPosition();
        double yaw = oldPosition.getHeading(AngleUnit.DEGREES);

        Pose2D newPosition = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, yaw);
    }

    public void updateHeading(double heading) {
        Pose2D oldPosition = getPosition();
        double x = oldPosition.getX(DistanceUnit.INCH);
        double y = oldPosition.getY(DistanceUnit.INCH);

        Pose2D newPosition = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
        updatePosition(newPosition);
    }

}
