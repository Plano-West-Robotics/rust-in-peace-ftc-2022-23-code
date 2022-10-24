package org.firstinspires.ftc.teamcode.enhancedautos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.ActionObject;
import org.firstinspires.ftc.teamcode.driveobjs.EnhancedDriver;
import org.firstinspires.ftc.teamcode.driveobjs.ObjectDetector;

import java.util.ArrayList;
import java.util.List;

public abstract class EnhancedAutoMode extends LinearOpMode {
    private EnhancedDriver enhancedDriver;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public List<ActionObject> actionObjects =  new ArrayList<>(0);


    public void run(){
        enhancedDriver.run(actionObjects);
    }



    public void initThings(){
        enhancedDriver = new EnhancedDriver(hardwareMap);
    }


    public int getShippingElementPos(int h, int pos1, int pos2, int pos3){
        ObjectDetector detector = new ObjectDetector(hardwareMap, h, pos1, pos2, pos3);
        int pos = detector.getPos();
        detector.endStream();
        return pos;
    }

}
