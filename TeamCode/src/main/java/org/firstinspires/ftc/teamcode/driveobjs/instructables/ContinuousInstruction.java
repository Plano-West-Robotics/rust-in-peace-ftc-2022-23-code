package org.firstinspires.ftc.teamcode.driveobjs.instructables;

public class ContinuousInstruction extends Instruction {
    /**
     * Creates an instruction to be interpreted by the InstructionRunner
     * @param trigger the tag that this instruction executes after
     * @param triggers the extra tags to wait on
     * @param tag the tag that is returned when this instruction finishes executing
     * @param executable the code to execute
     * @param actionDriver tells the instruction which ActionDriver to wait on
     */
    public ContinuousInstruction(String trigger, String tag, InstructionExecutable executable, ActionDriver actionDriver, String... triggers){
        this.triggers = add2BeginningOfArray(triggers, trigger);
        this.tag = tag;
        this.executable = executable;
        this.actionDrivers = new ActionDriver[]{actionDriver};
    }

    /**
     * Creates an instruction to be interpreted by the InstructionRunner
     * @param trigger the tags that this instruction executes after
     * @param tag the tag that is returned when this instruction finishes executing
     * @param executable the code to execute
     * @param actionDrivers tells the instruction which ActionDriver to wait on
     */
    public ContinuousInstruction(String trigger, String tag, InstructionExecutable executable, ActionDriver... actionDrivers){
        super(trigger, tag, executable, actionDrivers);
        this.triggers = new String[] {trigger};
        this.tag = tag;
        this.executable = executable;
        this.actionDrivers = actionDrivers;
    }



    //getters
    public ActionDriver[] getActionDrivers() {
        return super.actionDrivers;
    }
    public String getTag() {
        return tag;
    }
    public String[] getTriggers() {
        return triggers;
    }
    public InstructionExecutable getExecutable(){return executable; }

    @Override
    public String toString() {
        return "Instruction{" +
                "triggers='" + triggers.toString() + '\'' +
                ", tag='" + tag + '\'' +
                '}';
    }

    /**
     * executes the executable every call
     */
    @Override
    void execute() {
        executable.execute();
    }


}


