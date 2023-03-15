package org.firstinspires.ftc.teamcode.driveobjs.instructables;


/**
 * this is an instant instruction, which will only execute one time
 * it does not return anything upon completion
 */
public class InstantInstruction extends Instruction{
    private boolean hasExecuted;

    public InstantInstruction(String triggerTag, InstructionExecutable executable, String... extraTriggers){
        this.triggers = add2BeginningOfArray(extraTriggers, triggerTag);
        this.executable = executable;
        hasExecuted = false;
    }




    /**
     * this is going to run exactly once
     */
    @Override
    public void execute() {
        if (!hasExecuted){
            executable.execute();
            hasExecuted = true;
        }
    }
}
