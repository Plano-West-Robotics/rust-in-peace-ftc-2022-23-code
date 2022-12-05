package org.firstinspires.ftc.teamcode.troubleshooting;

import static org.firstinspires.ftc.teamcode.configs.HardwareNames.grabServo1Name;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class testTheServo extends OpMode {
    CRServo testServo;

    @Override
    public void loop() {
        testServo.setPower(gamepad1.right_stick_y);
    }

    @Override
    public void init(){
        testServo = hardwareMap.get(CRServo.class, grabServo1Name);

    }

}