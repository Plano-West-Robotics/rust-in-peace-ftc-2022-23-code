package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class SlidePID extends LinearOpMode {
    double integralSum = 0;

    public static double p = 0, i = 0, d = 0, f = 0;

    public static int target = 500;

    private DcMotorEx slideMotor;

    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;

    @Override
    public void runOpMode() {
        slideMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            double power = PIDControl(target, slideMotor.getCurrentPosition());
            slideMotor.setPower(power);
            telemetry.addData("pos", slideMotor.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.update();
        }
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError)/timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * p) + (derivative * d) + (integralSum * i) + (reference * f);
        return output;
    }
}
