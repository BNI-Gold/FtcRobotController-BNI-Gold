package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Vision {
    public HardwareMap hwBot = null;
    public Limelight3A cam = null;
    public LinearOpMode LinearOp = null;

    public double errorOffset = 4;

    public boolean captureSnapshots = false;
    public int snapshotLimit = 0;
    public String snapshotPrefix = "pov_";
    private int numSnapshots = 0;

    public Vision() {}

    public void setLinearOp(LinearOpMode LinearOp) {this.LinearOp = LinearOp;}

    public void initVision(HardwareMap hwMap, boolean captureSnapshots, int snapshotLimit, String snapshotPrefix) {
        hwBot = hwMap;
        this.captureSnapshots = captureSnapshots;
        this.snapshotLimit = snapshotLimit;
        this.snapshotPrefix = snapshotPrefix;

        cam = hwBot.get(Limelight3A.class, "limelight");
        cam.pipelineSwitch(0);
    }

    public void start() {
        cam.start();

        LinearOp.telemetry.addLine("Vision Started");
        LinearOp.telemetry.update();
    }

    public void setPipeline(int pipeline) {
        cam.pipelineSwitch(pipeline);
    }

    public LLResult getResult() {
        return cam.getLatestResult();
    }

    public Pose3D getPose() {
        LLResult res = cam.getLatestResult();
        if (res != null) {
            return res.getBotpose_MT2();
        }
        return null;
    }

    public double[] getOffsets() {
        LLResult res = cam.getLatestResult();
        if (res != null && res.isValid()) {
            if (captureSnapshots && numSnapshots < snapshotLimit) {
                captureSnapshot();
                numSnapshots ++;
            }

            double[] coords = new double[2];
            coords[0] = res.getTx();
            coords[1] = res.getTy();
            return coords;
        }
        return null;
    }

    public void captureSnapshot() {
        LLResult res = cam.getLatestResult();
        if (res != null && res.isValid()) {
            cam.captureSnapshot("pov_" + snapshotPrefix);
        }
    }

    public void stop() {
        cam.stop();
    }
}
