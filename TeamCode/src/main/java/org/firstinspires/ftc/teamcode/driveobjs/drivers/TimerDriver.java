package org.firstinspires.ftc.teamcode.driveobjs.drivers;

import org.firstinspires.ftc.teamcode.driveobjs.instructables.Instructable;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.Instruction;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.InstructionExecutable;

/**
 * a simple driver that returns !isBusy() after a certain amount of time
 */
public class TimerDriver implements ActionDriver, Instructable {
    long startMillis;
    long waitMillis;

    /**
     * This driver will be busy until current time is later than waitMillis + startMillis
     * This triggers on the millisecond, not after, although this precision should never be necessary
     * @param waitMillis the time to wait in millis
     */
    public TimerDriver(long waitMillis){
        this.waitMillis = waitMillis;
        init();
    }

    /**
     * sets the start time to the current time
     * please call this in the executable part of the instruction
     */
    @Override
    public void init() {
        startMillis = System.currentTimeMillis();
    }

    @Override
    public void run() {

    }

    @Override
    public boolean isBusy() {
        return System.currentTimeMillis() < startMillis+waitMillis;

    }

    /**
     * this is preferred
     * @param triggerTag the tag that this instruction executes after
     * @param returnTag the tag that is returned when this instruction finishes executing
     * @param executable the code to execute
     * @return
     */
    @Override
    public Instruction makeInstruction(String triggerTag, String returnTag, InstructionExecutable executable) {
        return new Instruction(triggerTag, returnTag, executable, this);
    }

    public static Instruction[] waitInstruction(String triggerTag, String returnTag, long time){
        TimerDriver timer = new TimerDriver(time);
        Instruction[] instructions = {
                new Instruction(triggerTag, ()->{timer.init();}),
                new Instruction(triggerTag, returnTag, ()->{}, timer)
        };
        return instructions;
    }

    /**
     * this is literally useless please do not use this
     * @param triggerTag the tag that this instruction executes after
     * @param executable the code to execute
     * @return
     */
    @Override
    public Instruction makeInstruction(String triggerTag, InstructionExecutable executable) {
        return new Instruction(triggerTag, executable);
    }
}
