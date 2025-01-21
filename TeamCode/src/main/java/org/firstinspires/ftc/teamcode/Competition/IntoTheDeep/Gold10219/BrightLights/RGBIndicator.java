package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BrightLights;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RGBIndicator {
    public HardwareMap hwBot = null;
    public Servo indicator = null;

    public enum LightOptions {
        RED(0.279),
        ORANGE(.333),
        YELLOW(.388),
        GREEN(.5),
        BLUE(.611),
        PURPLE(.722),
        OFF(0);

        private final double value;

        LightOptions(double value) {
            this.value = value;
        }

        public double getValue() {
            return this.value;
        }
    }

    public void initIndicator(HardwareMap hwMap) {
        hwBot = hwMap;

        indicator = hwBot.servo.get("indicator1");
        indicator.setDirection(Servo.Direction.FORWARD);
    }

    public void setColor(LightOptions option) {
        indicator.setPosition(option.getValue());
    }
}
