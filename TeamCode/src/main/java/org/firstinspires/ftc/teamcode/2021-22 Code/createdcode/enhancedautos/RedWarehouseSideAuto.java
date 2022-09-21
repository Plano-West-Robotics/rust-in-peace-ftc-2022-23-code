package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import static org.firstinspires.ftc.teamcode.createdcode.configs.AutoConfig.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous (group = "Red")
@Config
public class RedWarehouseSideAuto extends EnhancedAutoMode {
    public static double START_POS_X = 8;
    public static double START_POS_Y = -66;
    public static double START_POS_ANGLE = 90;
    public static int START_METHOD = 20;

    public static double BLOCK_DROP_Y = -44;

    public static int SHIPPING_ELEMENT_CENTER_HEIGHT = 142;
    public static int SHIPPING_ELEMENT_POS_1_X = 33;
    public static int SHIPPING_ELEMENT_POS_2_X = 164;
    public static int SHIPPING_ELEMENT_POS_3_X = 283;


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    public static double[] xCoordArr = {};
    private static double[] yCoordArr = {};
    private static double[] angleArr = {};
    private static int[] methodIdArr = {};

    public static double[] xCoordArrCase1 = {START_POS_X, -12, -12, -12, RED_WAREHOUSE_X};
    private static double[] yCoordArrCase1 = {START_POS_Y, -64, BLOCK_DROP_Y, -68, RED_WAREHOUSE_Y};
    private static double[] angleArrCase1 = {START_POS_ANGLE, 90, 90, 0, RED_WAREHOUSE_ANGLE};
    private static int[] methodIdArrCase1 = {START_METHOD, 13, 22, 10, 0};

    public static double[] xCoordArrCase2 = {START_POS_X, -12, -12, -12, RED_WAREHOUSE_X};
    private static double[] yCoordArrCase2 = {START_POS_Y, -64, BLOCK_DROP_Y, -68, RED_WAREHOUSE_Y};
    private static double[] angleArrCase2 = {START_POS_ANGLE, 90, 90, 0, RED_WAREHOUSE_ANGLE};
    private static int[] methodIdArrCase2 = {START_METHOD, 12, 22, 10, 0};

    public static double[] xCoordArrCase3 = {START_POS_X, -12, -12, -12, RED_WAREHOUSE_X};
    private static double[] yCoordArrCase3 = {START_POS_Y, -64, BLOCK_DROP_Y, -68, RED_WAREHOUSE_Y};
    private static double[] angleArrCase3 = {START_POS_ANGLE, 90, 90, 0, RED_WAREHOUSE_ANGLE};
    private static int[] methodIdArrCase3 = {START_METHOD, 11, 22, 10, 0};

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
            xCoordArr = xCoordArrCase3;
            yCoordArr = yCoordArrCase3;
            angleArr = angleArrCase3;
            methodIdArr = methodIdArrCase3;
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
