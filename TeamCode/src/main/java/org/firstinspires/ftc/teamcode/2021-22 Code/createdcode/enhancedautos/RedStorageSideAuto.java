package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import static org.firstinspires.ftc.teamcode.createdcode.configs.AutoConfig.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous (group = "Red")
@Config
public class RedStorageSideAuto extends EnhancedAutoMode {
    public static double START_POS_X = -41;
    public static double START_POS_Y = -66;
    public static double START_POS_ANGLE = 90;
    public static int START_METHOD = 20;

    public static int BLOCK_DROP_POINT = -30;

    public static int SHIPPING_ELEMENT_CENTER_HEIGHT = 140;
    public static int SHIPPING_ELEMENT_POS_1_X = 36;
    public static int SHIPPING_ELEMENT_POS_2_X = 162;
    public static int SHIPPING_ELEMENT_POS_3_X = 282;

    public static double[] xCoordArr = {};
    private static double[] yCoordArr = {};
    private static double[] angleArr = {};
    private static int[] methodIdArr = {};

    public static double[] xCoordArrCase1 = {START_POS_X, BLOCK_DROP_POINT, BLOCK_DROP_POINT, -56, RED_CAROUSEL_X, RED_STORAGE_X};
    private static double[] yCoordArrCase1 = {START_POS_Y, -48, -24, -24, RED_CAROUSEL_Y, RED_STORAGE_Y};
    private static double[] angleArrCase1 = {START_POS_ANGLE, 0, 0, 0, RED_CAROUSEL_ANGLE, RED_STORAGE_ANGLE};
    private static int[] methodIdArrCase1 = {START_METHOD, 13, 22, 10, 31, 0};

    public static double[] xCoordArrCase2 = {START_POS_X, -48, -48, BLOCK_DROP_POINT, -48, RED_CAROUSEL_X, RED_STORAGE_X};
    private static double[] yCoordArrCase2 = {START_POS_Y, -48, -24, -24, -24, RED_CAROUSEL_Y, RED_STORAGE_Y};
    private static double[] angleArrCase2 = {START_POS_ANGLE, 0, 0, 0, 0, RED_CAROUSEL_ANGLE, RED_STORAGE_ANGLE};
    private static int[] methodIdArrCase2 = {START_METHOD, 12 , 0, 22, 10, 31, 0};

    public static double[] xCoordArrCase3 = {START_POS_X, -48, -48, BLOCK_DROP_POINT, -48, RED_CAROUSEL_X, RED_STORAGE_X};
    private static double[] yCoordArrCase3 = {START_POS_Y, -48, -24, -24, -24, RED_CAROUSEL_Y, RED_STORAGE_Y};
    private static double[] angleArrCase3 = {START_POS_ANGLE, 0, 0, 0, 0, RED_CAROUSEL_ANGLE, RED_STORAGE_ANGLE};
    private static int[] methodIdArrCase3 = {START_METHOD, 11, 0, 22, 10, 31, 0};

    public static double[] xCoordArrCase0 = {START_POS_X, -56, -56, BLOCK_DROP_POINT, -56, RED_CAROUSEL_X, RED_STORAGE_X};
    private static double[] yCoordArrCase0 = {START_POS_Y, -48, -24, -24, -24, RED_CAROUSEL_Y, RED_STORAGE_Y};
    private static double[] angleArrCase0 = {START_POS_ANGLE, 0, 0, 0, 0, RED_CAROUSEL_ANGLE, RED_STORAGE_ANGLE};
    private static int[] methodIdArrCase0 = {START_METHOD, 11, 0, 22, 10, 31, 0};

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode(){
        waitForStart();


        int pos = getShippingElementPos(SHIPPING_ELEMENT_CENTER_HEIGHT, SHIPPING_ELEMENT_POS_1_X, SHIPPING_ELEMENT_POS_2_X, SHIPPING_ELEMENT_POS_3_X);
        dashboardTelemetry.addData("Position Detected is", pos);
        dashboardTelemetry.update();

        if (pos == 1) {
            xCoordArr = xCoordArrCase1;
            yCoordArr = yCoordArrCase1;
            angleArr = angleArrCase1;
            methodIdArr = methodIdArrCase1;
        }
        else if (pos == 2){
            xCoordArr = xCoordArrCase2;
            yCoordArr = yCoordArrCase2;
            angleArr = angleArrCase2;
            methodIdArr = methodIdArrCase2;
        }
        else if (pos == 3) {
            xCoordArr = xCoordArrCase3;
            yCoordArr = yCoordArrCase3;
            angleArr = angleArrCase3;
            methodIdArr = methodIdArrCase3;
        }
        else{
            xCoordArr = xCoordArrCase0;
            yCoordArr = yCoordArrCase0;
            angleArr = angleArrCase0;
            methodIdArr = methodIdArrCase0;
            telemetry.addData("ERROR:", "DID NOT DETECT POSITION");
            telemetry.update();
        }


        setXCoords(xCoordArr);
        setYCoords(yCoordArr);
        setAngles(angleArr);
        setMethodIDS(methodIdArr);

        try {
            makeActionObjects();
        } catch (Exception e) {
            dashboardTelemetry.addLine(e.getMessage());
            dashboardTelemetry.update();
        }

        initThings();

        run();
    }

}