package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BNO055;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class IMUHelper {
    public HardwareMap hwBot = null;
    public BNO055 imu = null;

    public IMUHelper() {}

    public void initImu(HardwareMap hwMap) {
        hwBot = hwMap;

        imu = hwBot.get(BNO055.class, "bno055");
        imu.init();
    }

    public void onStart() {
        imu.calibrate();
    }
}
