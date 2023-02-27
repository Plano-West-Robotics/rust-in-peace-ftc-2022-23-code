package org.firstinspires.ftc.teamcode.driveobjs.instructables;

import org.firstinspires.ftc.teamcode.driveobjs.drivers.ActionDriver;

public class Instruction {
    private String[] triggers;
    private String tag;
    private  InstructionExecutable executable;
    private ActionDriver[] actionDrivers;
    private boolean hasReturn;

    /**
     * Creates an instruction to be interpreted by the InstructionRunner
     * @param triggers the tags that this instruction executes after
     * @param tag the tag that is returned when this instruction finishes executing
     * @param executable the code to execute
     * @param actionDriver tells the instruction which ActionDriver to wait on
     */
    public Instruction(String tag, InstructionExecutable executable, ActionDriver actionDriver, String... triggers){
        this.triggers = triggers;
        this.tag = tag;
        this.executable = executable;
        this.actionDrivers = actionDrivers;
        hasReturn = true;

    }



    /**
     * Creates an instruction WITHOUT A RETURN TAG, there will be no informing when this instruction finishes executing
     * @param triggers the tag that this instruction executes after
     * @param executable the code to execute
     */
    public Instruction(InstructionExecutable executable, String... triggers){
        this.triggers = triggers;
        this.executable = executable;
        hasReturn = false;
    }

    public ActionDriver[] getActionDrivers() {
        return actionDrivers;
    }

    public String getTag() {
        return tag;
    }

    public String[] getTriggers() {
        return triggers;
    }

    public boolean getHasReturn(){return hasReturn;}

    public InstructionExecutable getExecutable(){return executable; }

    @Override
    public String toString() {
        return "Instruction{" +
                "triggers='" + triggers.toString() + '\'' +
                ", tag='" + tag + '\'' +
                ", hasReturn=" + hasReturn +
                '}';
    }
}


