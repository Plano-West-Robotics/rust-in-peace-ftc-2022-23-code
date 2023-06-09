package org.firstinspires.ftc.teamcode.configs;


import com.acmerobotics.dashboard.config.Config;

/**
 * stores the encoder value for certain arm positions
 */
@Config
public class ArmPosStorage {
    //ground
    public static int ARM_POS_0 = 0;

    //low TODO: check accuracy
    public static int ARM_POS_1 = -200;

    //medium TODO: check accuracy
    public static int ARM_POS_2 = -3000;

    //high TODO: check accuracy
    public static int ARM_POS_3 = -4100;

    /**
     * stores the positions required to traverse the stack
     * TODO: RE-TUNE
     */
    public static int[] stackArmPoses = {-700, -600, -400, -250, -100};
}
