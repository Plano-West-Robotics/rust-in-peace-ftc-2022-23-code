package org.firstinspires.ftc.teamcode.troubleshooting;

import static org.firstinspires.ftc.teamcode.configs.HardwareNames.grabServo2Name;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.driveobjs.drivers.ClawDriver;

@TeleOp
public class testClawDriver extends OpMode {
    Servo testServo;
    double position;
    boolean wasPressingDpadUp = false;
    boolean wasPressingDpadDown = false;
    CRServo crServo;
    ClawDriver clawDriver;

    @Override
    public void loop() {
        if(gamepad1.x)
            clawDriver.close();
        else if(gamepad1.y)
            clawDriver.open();
    }

    @Override
    public void init(){
        testServo = hardwareMap.get(Servo.class, grabServo2Name);
        position = 0.5;

        clawDriver = new ClawDriver(hardwareMap);
    }

}