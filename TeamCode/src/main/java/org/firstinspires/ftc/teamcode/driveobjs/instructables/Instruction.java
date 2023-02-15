package org.firstinspires.ftc.teamcode.driveobjs.instructables;

import org.firstinspires.ftc.teamcode.driveobjs.drivers.ActionDriver;

import java.util.Objects;

public class Instruction {
    String trigger;
    ActionDriver driver;
    String tag;
    Enum state;
    Object extra;
    InstructionExecutable executable;
    ActionDriver[] actionDrivers;

    /**
     * Creates an instruction to be interpreted by the InstructionRunner
     * @param trigger the tag that this instruction executes after
     * @param tag the tag that is returned when this instruction finishes executing
     * @param executable the code to execute
     * @param actionDrivers tells the instruction WHICH ACTION DRIVERS TO WAIT TO FINISH BEFORE SENDING THE NEXT CODE (VERY IMPORTANT)
     */
    public Instruction(String trigger, String tag, InstructionExecutable executable, ActionDriver... actionDrivers){
        this.trigger = trigger;
        this.tag = tag;
        this.executable = executable;
        this.driver = driver;
        this.state = state;
        this.actionDrivers = actionDrivers;
        if(!Objects.isNull(extra)){
            this.extra = extra;
        }
    }

    /**
     * Creates an instruction WITHOUT A RETURN TAG, there will be no informing when this instruction finishes executing
     * @param trigger the tag that this instruction executes after
     * @param executable the code to execute
     */
    public Instruction(String trigger, InstructionExecutable executable){
        this.trigger = trigger;
        this.executable = executable;
    }

    public ActionDriver[] getActionDrivers() {
        return actionDrivers;
    }

    public String getTag() {
        return tag;
    }

    public String getTrigger() {
        return trigger;
    }

    public InstructionExecutable getExecutable(){return executable; }
}


