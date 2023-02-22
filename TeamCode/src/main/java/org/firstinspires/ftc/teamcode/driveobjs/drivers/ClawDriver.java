package org.firstinspires.ftc.teamcode.driveobjs.drivers;

import static org.firstinspires.ftc.teamcode.configs.HardwareNames.grabServo1Name;
import static org.firstinspires.ftc.teamcode.configs.HardwareNames.grabServo2Name;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.driveobjs.instructables.Instructable;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.Instruction;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.InstructionExecutable;

@Config

public class ClawDriver implements ActionDriver, Instructable {
    public static double claw1Min = 0;
    public static double claw1Max = 0.6;
    public static double claw2Min = 0.30;
    public static double claw2Max = 0.55;


    private double claw1Target;
    private double claw2Target;

    /**
     *
     * @return boolean determining if the claw is busy or not
     */
    @Override
    public boolean isBusy() {
        //checks if the claw is busy by seeing if either claw position is different from the target position
        return clawServo1.getPosition() != claw1Target || clawServo2.getPosition() != claw2Target;
    }

    public enum ClawState {OPEN, CLOSED}

    private HardwareMap hardwareMap;
    private Servo clawServo1, clawServo2;
    private ClawState clawState;

    public ClawDriver(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init();
    }

    public void open() {
        clawServo1.setPosition(claw1Min);
        clawServo2.setPosition(claw2Max);
        clawState = ClawState.OPEN;
    }

    public void close() {
        clawServo1.setPosition(claw1Max);
        clawServo2.setPosition(claw2Min);
        clawState = ClawState.CLOSED;
    }

    public void run() {
        switch (clawState) {
            case OPEN:
                open();
                break;
            case CLOSED:
                close();
                break;
        }
    }


    public void init() {
        clawServo1 = hardwareMap.get(Servo.class, grabServo1Name);
        //clawServo1.scaleRange(claw1Min, claw1Max);

        clawServo2 = hardwareMap.get(Servo.class, grabServo2Name);
        //clawServo2.scaleRange(claw2Min, claw2Max);

        clawState = ClawState.CLOSED;
    }

    public void setClawState(ClawState state) {
        clawState = state;
    }

    /**
     * Creates an instruction that waits on this driver by default
     * Generally not recommended to use, as IsBusy() will only be false when opening
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
     * Preferred due to claw wait usually not being significant
     * @param triggerTag the tag that this instruction executes after
     * @param executable the code to execute
     * @return
     */
    @Override
    public Instruction makeInstruction(String triggerTag, InstructionExecutable executable){
        return new Instruction(triggerTag, executable);
    }


}



