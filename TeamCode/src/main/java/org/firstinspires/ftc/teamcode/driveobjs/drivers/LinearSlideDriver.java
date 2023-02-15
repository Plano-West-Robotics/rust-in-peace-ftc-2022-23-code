package org.firstinspires.ftc.teamcode.driveobjs.drivers;

import static org.firstinspires.ftc.teamcode.configs.HardwareNames.spoolMotorName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.driveobjs.instructables.Instructable;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.Instruction;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.InstructionExecutable;

@Config
public class LinearSlideDriver implements ActionDriver, Instructable {

    private DcMotorEx spoolMotor;
    private HardwareMap hardwareMap;

    //TODO: tune these values
    public static double Kp = 0.1;
    public static double Ki = 0.1;
    public static double Kd = 0.1;

    private double integral;
    private int previous_error, targetEncoderValue = 0;
    private long lastTime = 0;
    private double pidOutput = 0;

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
        pidOutput = Kp*error + Ki*integral + Kd*derivative;
        previous_error = error;




    }

    public void init() {
        spoolMotor = hardwareMap.get(DcMotorEx.class, spoolMotorName);
        spoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //spoolMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spoolMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        long lastTime = System.currentTimeMillis();
        //spoolMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
    public int[] runWithPID(){
        PID();
        spoolMotor.setVelocity(pidOutput);
        int[] outputValues = {targetEncoderValue, spoolMotor.getCurrentPosition(), previous_error};
        return outputValues;
    }

    public int getTargetEncoderValue() {
        return targetEncoderValue;
    }
    public int getCurrentEncoderValue(){
        return spoolMotor.getCurrentPosition();
    }

    /**
     * intended for use with the ArmPosStorage class for the correct datapoints
     */
    public void setTargetPosition(int encoderValue){
        //remembers the encoder value just in case
        targetEncoderValue = encoderValue;
        spoolMotor.setTargetPosition(targetEncoderValue);
    }


    public void run(){
        spoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (spoolMotor.getTargetPosition() == spoolMotor.getCurrentPosition()){
            spoolMotor.setPower(0);
            return;
        }


        if (spoolMotor.getTargetPosition() > spoolMotor.getCurrentPosition()){
            spoolMotor.setPower(1);
        }
        else if (spoolMotor.getTargetPosition() < spoolMotor.getCurrentPosition()){
            spoolMotor.setPower(-1);
        }



    }

    public int[] getOutputValues(){
        int[] outputValues = {targetEncoderValue, spoolMotor.getCurrentPosition(), previous_error};
        return outputValues;
    }


    @Override
    public boolean isBusy(){
        return spoolMotor.isBusy();
    }


    /**
     * Creates an instruction that waits on this driver by default
     * @param triggerTag the tag that this instruction executes after
     * @param returnTag the tag that is returned when this instruction finishes executing
     * @param executable the code to execute
     * @return returns the intruction that has been created
     */
    @Override
    public Instruction makeInstruction(String triggerTag, String returnTag, InstructionExecutable executable) {
        return new Instruction(triggerTag, returnTag, executable, this);
    }


    /**
     * Creates an instruction
     * @param triggerTag the tag that this instruction executes after
     * @param executable the code to execute
     * @return
     */
    @Override
    public Instruction makeInstruction(String triggerTag, InstructionExecutable executable){
        return new Instruction(triggerTag, executable);
    }

}
