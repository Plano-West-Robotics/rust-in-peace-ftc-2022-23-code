package org.firstinspires.ftc.teamcode.driveobjs;

import static org.firstinspires.ftc.teamcode.configs.HardwareNames.grabServo1Name;
import static org.firstinspires.ftc.teamcode.configs.HardwareNames.grabServo2Name;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config

public class ClawDriver{
    public static double claw1Min = 0;
    public static double claw1Max = 0.55;
    public static double claw2Min = 0.35;
    public static double claw2Max = 0.55;


    private HardwareMap hardwareMap;
    private Servo clawServo1, clawServo2;


    public ClawDriver(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        init();
    }

    public void close(){
        clawServo1.setPosition(claw1Min);
        clawServo2.setPosition(claw2Max);
    }

    public void open(){
        clawServo1.setPosition(claw1Max);
        clawServo2.setPosition(claw2Min);
    }



    private void init() {
        clawServo1 = hardwareMap.get(Servo.class, grabServo1Name);
        //clawServo1.scaleRange(claw1Min, claw1Max);

        clawServo2 = hardwareMap.get(Servo.class, grabServo2Name);
        //clawServo2.scaleRange(claw2Min, claw2Max);
    }
}
