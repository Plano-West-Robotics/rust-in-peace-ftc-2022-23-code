package org.firstinspires.ftc.teamcode.driveobjs.drivers;

import org.firstinspires.ftc.teamcode.driveobjs.instructables.ActionDriver;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.InstantInstruction;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.Instructable;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.Instruction;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.InstructionExecutable;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.SetInstruction;

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
    public Instruction makeInstruction(String triggerTag, String returnTag, InstructionExecutable executable, String... triggers) {
        return new SetInstruction(triggerTag, returnTag, ()->{this.init();}, this, triggers);
    }



    /**
     * @deprecated this is literally useless please do not use this
     * @param triggerTag the tag that this instruction executes after
     * @param executable the code to execute
     * @return
     */
    @Override
    public Instruction makeInstruction(String triggerTag, InstructionExecutable executable, String... triggers) {
        return new InstantInstruction(triggerTag, executable, triggers);
    }


    /**
     * public static method to make a new TimerDriver to wait
     * @param triggerTag the tag to trigger on
     * @param returnTag the tag to return
     * @param time the time to wait in milliseconds
     * @param triggers (Optional) extra tags to trigger on
     * @return
     */
    public static Instruction waitInstruction(String triggerTag, String returnTag, long time, String... triggers){
        TimerDriver timer = new TimerDriver(time);
        Instruction instruction = new SetInstruction(triggerTag, returnTag, ()->{timer.init();}, timer, triggers);
        return instruction;
    }
}
