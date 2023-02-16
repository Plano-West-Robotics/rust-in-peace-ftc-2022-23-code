package org.firstinspires.ftc.teamcode.driveobjs.drivers;

import org.firstinspires.ftc.teamcode.driveobjs.instructables.Instructable;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.Instruction;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.InstructionExecutable;

/**
 * a simple driver that returns !isBusy() after a certain amount of time
 */
public class TimerDriver implements ActionDriver, Instructable {

    @Override
    public void init() {

    }

    @Override
    public void run() {

    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public Instruction makeInstruction(String triggerTag, String returnTag, InstructionExecutable executable) {
        return null;
    }

    @Override
    public Instruction makeInstruction(String triggerTag, InstructionExecutable executable) {
        return null;
    }
}
