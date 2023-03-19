package org.firstinspires.ftc.teamcode.driveobjs.instructables;

import java.util.Arrays;

public abstract class Instruction {

    /**
     * these default behaviors should let contains() and containsAll() work properly
     */
    protected String[] triggers = {};
    protected String tag;
    protected InstructionExecutable executable;
    protected ActionDriver[] actionDrivers = {};



    /**
     * Quick helper function
     * @param elements array
     * @param element element
     * @param <T> type
     * @return
     */
    static <T> T[] add2BeginningOfArray(T[] elements, T element)
    {
        T[] newArray = Arrays.copyOf(elements, elements.length + 1);
        newArray[0] = element;
        System.arraycopy(elements, 0, newArray, 1, elements.length);

        return newArray;
    }


    /**
     * default constructor
     * does nothing
     */
    public Instruction(){

    }



    /**
     * @deprecated old artefacts
     * Creates an instruction to be interpreted by the InstructionRunner
     * @param trigger the tag that this instruction executes after
     * @param triggers the extra tags to wait on
     * @param tag the tag that is returned when this instruction finishes executing
     * @param executable the code to execute
     * @param actionDriver tells the instruction which ActionDriver to wait on
     */
    public Instruction(String trigger, String tag, InstructionExecutable executable, ActionDriver actionDriver, String... triggers){
        this.triggers = add2BeginningOfArray(triggers, trigger);
        this.tag = tag;
        this.executable = executable;
        this.actionDrivers[0] = actionDriver;

    }

    /**
     * @deprecated old artefacts
     * Creates an instruction to be interpreted by the InstructionRunner
     * @param trigger the tags that this instruction executes after
     * @param tag the tag that is returned when this instruction finishes executing
     * @param executable the code to execute
     * @param actionDrivers tells the instruction which ActionDriver to wait on
     */
    public Instruction(String trigger, String tag, InstructionExecutable executable, ActionDriver... actionDrivers){
        this.triggers = new String[] {trigger};
        this.tag = tag;
        this.executable = executable;
        this.actionDrivers = actionDrivers;
    }


    /**
     * @deprecated old artefacts
     * Creates an instruction WITHOUT A RETURN TAG, there will be no informing when this instruction finishes executing
     * @param trigger the tag that this instruction executes after
     * @param triggers the extra tags to wait on
     * @param executable the code to execute
     */
    public Instruction(String trigger, InstructionExecutable executable, String... triggers){
        this.triggers = add2BeginningOfArray(triggers, trigger);
        this.executable = executable;
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


    public InstructionExecutable getExecutable(){return executable; }

    @Override
    public String toString() {
        return "Instruction{" +
                "triggers='" + Arrays.asList(triggers) + '\'' +
                ", tag='" + tag + '\'' +
                '}';
    }


    /**
     * Takes in the primed triggers and checks if this instruction needs to start or not
     * @param inputTags the array of triggers to check requirements against
     * @return
     */
    public boolean checkStart(String... inputTags) {
        for (String trigger : triggers){
            boolean isContained = false;
            for (String inputTag : inputTags){
                if (!trigger.equals(inputTag))
                    isContained = true;
            }
            if(!isContained)
                return false;
        }
        return true;
    }

    /** checks if the the actionDriver is completed or not based on their state
     *
     */
    boolean checkCompletion(ActionDriver... completedActionDrivers){
        for (ActionDriver driver : actionDrivers){
            if (driver.isBusy())
                return false;
        }
        return true;
    }

    /**
     * the primary execution function
     * this will modify itself as necessary
     */
    abstract void execute();



}


