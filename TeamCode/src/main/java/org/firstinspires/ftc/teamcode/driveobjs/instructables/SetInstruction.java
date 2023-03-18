package org.firstinspires.ftc.teamcode.driveobjs.instructables;

public class SetInstruction extends Instruction{
    private boolean hasExecuted;

    /**
     * Creates an instruction to be interpreted by the InstructionRunner
     * @param trigger the tag that this instruction executes after
     * @param triggers the extra tags to wait on
     * @param tag the tag that is returned when this instruction finishes executing
     * @param executable the code to execute
     * @param actionDriver tells the instruction which ActionDriver to wait on
     */
    public SetInstruction(String trigger, String tag, InstructionExecutable executable, ActionDriver actionDriver, String... triggers){
        this.triggers = add2BeginningOfArray(triggers, trigger);
        this.tag = tag;
        this.executable = executable;
        this.actionDrivers = new ActionDriver[]{actionDriver};
        hasExecuted = false;

    }

    /**
     * Creates an instruction to be interpreted by the InstructionRunner
     * @param trigger the tags that this instruction executes after
     * @param tag the tag that is returned when this instruction finishes executing
     * @param executable the code to execute
     * @param actionDrivers tells the instruction which ActionDriver to wait on
     */
    public SetInstruction(String trigger, String tag, InstructionExecutable executable, ActionDriver... actionDrivers){
        this.triggers = new String[] {trigger};
        this.tag = tag;
        this.executable = executable;
        this.actionDrivers = actionDrivers;
        hasExecuted = false;
    }

    /**
     * runs once, and then just returns after that
     */
    @Override
    void execute() {
        if (!hasExecuted){
            executable.execute();
            hasExecuted = true;
        }
    }
}
