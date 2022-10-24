package org.firstinspires.ftc.teamcode.troubleshooting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class testTheMotor extends OpMode {
    DcMotor testMotor;

    @Override
    public void loop() {
        testMotor.setPower(1);
    }

    @Override
    public void init(){
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");

    }

}