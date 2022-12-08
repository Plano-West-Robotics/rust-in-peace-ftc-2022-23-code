package org.firstinspires.ftc.teamcode.troubleshooting;

import static org.firstinspires.ftc.teamcode.configs.HardwareNames.grabServo1Name;
import static org.firstinspires.ftc.teamcode.configs.HardwareNames.grabServo2Name;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.driveobjs.ClawDriver;

@TeleOp
public class testTheServo extends OpMode {
    Servo testServo;
    double position;
    boolean wasPressingDpadUp = false;
    boolean wasPressingDpadDown = false;
    CRServo crServo;
    ClawDriver clawDriver;

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            if (!wasPressingDpadUp) {
                position+=0.05;
            }
            wasPressingDpadUp = true;
        }
        else wasPressingDpadUp = false;

        if (gamepad1.dpad_down) {
            if (!wasPressingDpadDown) {
                position-=0.05;
            }
            wasPressingDpadDown = true;
        }
        else wasPressingDpadDown = false;

        testServo.setPosition(position);
        telemetry.addData("Position", testServo.getPosition());

        if(gamepad1.a)
            testServo = hardwareMap.get(Servo.class, grabServo1Name);
        if(gamepad1.b)
            testServo = hardwareMap.get(Servo.class, grabServo2Name);

        testServo.setPosition(position);

        telemetry.addData("Position", testServo.getPosition());
    }

    @Override
    public void init(){
        testServo = hardwareMap.get(Servo.class, grabServo2Name);
        position = 0.5;

        clawDriver = new ClawDriver(hardwareMap);

    }

}