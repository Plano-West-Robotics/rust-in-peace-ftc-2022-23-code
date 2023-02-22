package org.firstinspires.ftc.teamcode.troubleshooting;

import static org.firstinspires.ftc.teamcode.configs.HardwareNames.spoolMotorName;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testTheMotor extends OpMode {
    DcMotor testMotor;

    @Override
    public void loop() {
        testMotor.setPower(gamepad1.right_stick_y);
    }

    @Override
    public void init(){
        testMotor = hardwareMap.get(DcMotor.class, spoolMotorName);

    }

}