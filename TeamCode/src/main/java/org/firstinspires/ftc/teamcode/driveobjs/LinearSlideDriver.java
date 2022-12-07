package org.firstinspires.ftc.teamcode.driveobjs;

import static org.firstinspires.ftc.teamcode.configs.HardwareNames.spoolMotorName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
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

    //TODO: tune these values
    public static int height1 = 0;
    public static int height2 = 100;
    public static int height3 = 200;
    public static int height4 = 300;


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

    public void setTargetHeight1(){
        targetEncoderValue = height1;
    }

    public void setTargetHeight2(){
        targetEncoderValue = height2;
    }

    public void setTargetHeight3(){
        targetEncoderValue = height3;
    }

    public void setTargetHeight4(){
        targetEncoderValue = height4;
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
