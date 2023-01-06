package org.firstinspires.ftc.teamcode.configs;


import com.acmerobotics.dashboard.config.Config;

/**
 * stores the encoder value for certain arm positions
 */
@Config
public class ArmPosStorage {
    public static int ArmPos0 = 0;
    public static int ArmPos1 = 1000;
    public static int ArmPos2 = 3000;
    public static int ArmPos3 = 4100;

    /**
     * stores the positions required to traverse the stack
     * TODO: TUNE
     */
    public static int[] stackArmPoses = {564, 292, 142, 42, 0};
}
