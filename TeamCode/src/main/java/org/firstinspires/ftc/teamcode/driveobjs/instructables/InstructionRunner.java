package org.firstinspires.ftc.teamcode.driveobjs.instructables;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

public class InstructionRunner {
    private Pose2d startPose;
    ActionDriver[] actionDrivers;
    Queue<String> tags = new LinkedList<String>();
    Telemetry telemetry;

    //stores the instructions
    ArrayList<Instruction> instructions = new ArrayList<Instruction>();

    //stores the instructions that need execution
    ArrayList<Instruction> needsExecution = new ArrayList<Instruction>();



    /**
     * Creates the instruction runner
     * @param startPose give the initial Pose2d
     * @param initialTag set the first tag to run
     * @param actionDrivers gives the list of drivers to run asynchronously
     */
    public InstructionRunner(HardwareMap hardwareMap, Pose2d startPose, String initialTag, Telemetry telemetry, ActionDriver... actionDrivers) {
        this.telemetry = telemetry;
        this.startPose = startPose;
        tags.add(initialTag);
        this.actionDrivers = actionDrivers;
    }


    /**
     * creates and adds and instruction
     * not recommended for use, use the drivers' specific makeInstructions instead
     * @param triggerTags the tag(s) that this instruction executes after
     * @param returnTag the tag that is returned when this instruction finishes executing
     * @param executable the code to execute
     */
    public void addContinousInstruction(String trigger, String returnTag, InstructionExecutable executable, ActionDriver actionDriver, String... triggerTags) {
        instructions.add(new ContinuousInstruction(trigger, returnTag, executable, actionDriver, triggerTags));
    }

    /**
     * adds the instruction to the list
     * preferred over the other variant
     * @param instructions the instruction to be added
     */
    public void addInstruction(Instruction... instructions){
        for(Instruction i : instructions)
            this.instructions.add(i);
    }




    /**
     * primary execution loop for InstructionRunner
     * this is intended to be looped to follow the list of instructions
     */
    public String run() {
        //creates a shallow copy of needsExecution to loop through for pruning
        ArrayList<Instruction> executionCopy = (ArrayList<Instruction>) needsExecution.clone();

        //prunes the completed instructions
        ActionDriver[] completedActionDrivers = checkActionDriverCompletion();
        for (Instruction i : executionCopy) {
            pruneInstruction(i, completedActionDrivers);
        }


        //runs findInstructions on every tag in tags
        //this gets all of the instructions needed and adds them to execution copy
        needsExecution = findInstructions(tags, instructions);


        telemetry.addData("Found Instructions: ", needsExecution.toString());



        //executes those tags
        for (Instruction i : needsExecution) {
            executeInstruction(i);
        }

        //runs the idles for the drivers
        for(ActionDriver driver : actionDrivers){
            driver.run();
        }

        return tags.toString();
    }

    /**
     * executes the instruction given by calling their execute() method
     * @param i the instruction to be run against
     */
    private void executeInstruction(Instruction i) {
        i.execute();
    }

    /**
     * removes the instruction if it is a one time execution, or if it is completed
     * @param i te instruction to be run against
     */
    private void pruneInstruction(Instruction i, ActionDriver[] completedActionDrivers){
        //checks if the completion condition has been met
        if (i.checkCompletion(completedActionDrivers)){
            //if it is met, this will remove it from the list and add its tag to tags
            needsExecution.remove(i);
            tags.add(i.getTag());
            return;
        }
    }

    /**
     * checks which actionDrivers are done and returns the list of completed ones (reduces isBusy() checks)
     * @return an arraylist of completed actiondrivers
     */
    private ActionDriver[] checkActionDriverCompletion(){
        ArrayList<ActionDriver> completedActionDrivers = new ArrayList<>();
        for(ActionDriver driver : actionDrivers){
            if (!driver.isBusy())
                completedActionDrivers.add(driver);
        }
        return completedActionDrivers.toArray(new ActionDriver[0]);
    }


    /**
     * digs out the instructions that the new tag(s) triggers
     * @param tags the tag that is being checked for as a trigger
     * @param instructions the list of instructions to check for readiness
     */
    private ArrayList<Instruction> findInstructions(Queue tags, ArrayList<Instruction> instructions) {
        ArrayList<Instruction> needsExecution = new ArrayList<>();
        for (Instruction instruction : instructions) {
            if (tags.containsAll(Arrays.asList(instruction.getTriggers())))
                needsExecution.add(instruction);
        }
        return needsExecution;
    }

    public String getTags(){
        return tags.toString();
    }
}

