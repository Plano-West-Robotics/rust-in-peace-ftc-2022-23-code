package org.firstinspires.ftc.teamcode.configs;


import com.acmerobotics.dashboard.config.Config;

/**
 * stores the encoder value for certain arm positions
 */
@Config
public class ArmPosStorage {
    //ground
    public static int ArmPos0 = 0;

    //low TODO: check accuracy
    public static int ArmPos1 = 200;

    //medium TODO: check accuracy
    public static int ArmPos2 = 3000;

    //high TODO: check accuracy
    public static int ArmPos3 = 4000;

    /**
     * stores the positions required to traverse the stack
     * TODO: RE-TUNE
     */
    public static int[] stackArmPoses = {564, 292, 142, 42, 0};
}
