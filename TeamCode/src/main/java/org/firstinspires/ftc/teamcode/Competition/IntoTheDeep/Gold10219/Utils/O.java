package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Utils;

public class O{
    public static <T> T req(T obj) {
        if (obj == null)
            throw new NullPointerException();
        return obj;
    }
}
