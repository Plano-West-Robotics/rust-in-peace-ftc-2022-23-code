package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.configs.HardwareNames.spoolMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlideDriver {

    private DcMotor spoolMotor;
    private HardwareMap hardwareMap;

    //TODO: tune these values
    public static double Kp = 1;
    public static double Ki = 1;
    public static double Kd = 1;
    private double integral;
    private int previous_error, targetEncoderValue = 0;
    private long lastTime = 0;
    private double outputPower = 0;


    public LinearSlideDriver(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        init();
    }

    public void setTarget(int targetEncoderValue){
        this.targetEncoderValue = targetEncoderValue;
    }

    private void PID(){
        /**
         * modeled after https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html
         */

        int error = targetEncoderValue - spoolMotor.getCurrentPosition();
        double dt = (System.currentTimeMillis() - lastTime)/(1000.0);
        integral += (error * dt);
        double derivative = (error - this.previous_error) / dt;
        outputPower = Math.min(Kp*error + Ki*integral + Kd*derivative, 1);
        previous_error = error;
    }

    private void init() {
        spoolMotor = hardwareMap.get(DcMotor.class, spoolMotorName);
        spoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spoolMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spoolMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        long lastTime = System.currentTimeMillis();
    }


    /**
     *
     * @return an array containing the target, current, and error values for telemetry
     */
    public int[] run(){
        PID();
        spoolMotor.setPower(outputPower);
        int[] outputValues = {targetEncoderValue, spoolMotor.getCurrentPosition(), previous_error};
        return outputValues;
    }


}
