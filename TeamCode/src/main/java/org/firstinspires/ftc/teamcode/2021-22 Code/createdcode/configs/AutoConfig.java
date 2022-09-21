package org.firstinspires.ftc.teamcode.createdcode.configs;


import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoConfig {
    //REMEMBER THAT LEFT IS POS 1, RIGHT IS POS 3

    public static int OBJECT_CENTER_Y = 135;
    public static int MIN_H = 100;
    public static int MIN_S = 225;
    public static int MIN_V = 100;
    public static int MAX_H = 120;
    public static int MAX_S = 255;
    public static int MAX_V = 255;

    //carousel values
    public static double CAROUSEL_POWER = 0.5;
    public static int CAROUSEL_WAIT = 6000;

    //carousel positions
    //blue
    public static double BLUE_CAROUSEL_X = -60;
    public static double BLUE_CAROUSEL_Y = 58;
    public static double BLUE_CAROUSEL_ANGLE = 180;

    //red
    public static double RED_CAROUSEL_X = -60;
    public static double RED_CAROUSEL_Y = -58;
    public static double RED_CAROUSEL_ANGLE = 180;


    //arm positions
    public static int ARM_MAX_DIST = 415;
    public static int SLIGHTLY_OFF_GROUND = 40;
    public static int LAYER_ONE_BACK_ARM_POS = 340;
    public static int LAYER_TWO_BACK_ARM_POS = 375;
    public static int LAYER_THREE_BACK_ARM_POS = 415;
    public static int LAYER_ONE_FRONT_ARM_POS = 130;
    public static int LAYER_TWO_FRONT_ARM_POS = 70;
    public static int LAYER_THREE_FRONT_ARM_POS = 40;

    //grabber constants
    public static int GRAB_TIME = 1000;
    public static double GRABBER_CLOSE = 0.0;
    public static double GRABBER_OPEN = 0.6;

    //storage coords
    //blue
    public static double BLUE_STORAGE_X = -60;
    public static double BLUE_STORAGE_Y = 36;
    public static double BLUE_STORAGE_ANGLE = 360;

    //red
    public static double RED_STORAGE_X = -60;
    public static double RED_STORAGE_Y = -36;
    public static double RED_STORAGE_ANGLE = 0;


    //warehouse coords
    //blue
    public static double BLUE_WAREHOUSE_X = 48;
    public static double BLUE_WAREHOUSE_Y = 68;
    public static double BLUE_WAREHOUSE_ANGLE = 0;

    //red
    public static double RED_WAREHOUSE_X = 48;
    public static double RED_WAREHOUSE_Y = -68;
    public static double RED_WAREHOUSE_ANGLE = 0;


}
