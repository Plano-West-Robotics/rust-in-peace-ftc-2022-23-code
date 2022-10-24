package org.firstinspires.ftc.teamcode.driveobjs;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class ActionObject {
    private double x, y;
    private double angle;
    private int methodID;

    //angle is ending angle iirc
    public ActionObject(double x, double y, double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public ActionObject(double x, double y, double angle, int methodID){
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.methodID = methodID;
    }


    public Pose2d getPose2d(){
        return new Pose2d(x, y, Math.toRadians(angle));
    }

    public int getMethodID(){return methodID;}
}
