package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import static org.firstinspires.ftc.teamcode.createdcode.configs.AutoConfig.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@Autonomous
@Config
public class AutoBase extends EnhancedAutoMode {
    public static double START_POS_X = 0;
    public static double START_POS_Y = 0;
    public static double START_POS_ANGLE = 0;
    public static int START_METHOD = 20;

    public static int SHIPPING_ELEMENT_CENTER_HEIGHT = 0;
    public static int SHIPPING_ELEMENT_POS_1_X = 0;
    public static int SHIPPING_ELEMENT_POS_2_X = 0;
    public static int SHIPPING_ELEMENT_POS_3_X = 0;


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    public static double[] xCoordArr = {};
    private static double[] yCoordArr = {};
    private static double[] angleArr = {};
    private static int[] methodIdArr = {};

    public static double[] xCoordArrCase1 = {START_POS_X};
    private static double[] yCoordArrCase1 = {START_POS_Y};
    private static double[] angleArrCase1 = {START_POS_ANGLE};
    private static int[] methodIdArrCase1 = {START_METHOD};

    public static double[] xCoordArrCase2 = {START_POS_X};
    private static double[] yCoordArrCase2 = {START_POS_Y};
    private static double[] angleArrCase2 = {START_POS_ANGLE};
    private static int[] methodIdArrCase2 = {START_METHOD};

    public static double[] xCoordArrCase3 = {START_POS_X};
    private static double[] yCoordArrCase3 = {START_POS_Y};
    private static double[] angleArrCase3 = {START_POS_ANGLE};
    private static int[] methodIdArrCase3 = {START_METHOD};

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


        initThings();

        setXCoords(xCoordArr);
        setYCoords(yCoordArr);
        setAngles(angleArr);
        setMethodIDS(methodIdArr);

        try {
            makeActionObjects();
        } catch (Exception e) {
            dashboardTelemetry.addLine(e.getMessage());
        }


        run();
    }

}
