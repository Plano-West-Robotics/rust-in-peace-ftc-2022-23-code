package org.firstinspires.ftc.teamcode.driveobjs.instructables;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.ActionDriver;

import java.util.ArrayList;
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
    public void addInstruction(String returnTag, InstructionExecutable executable, ActionDriver actionDriver, String... triggerTags) {
        instructions.add(new Instruction(returnTag, executable, actionDriver, triggerTags);
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
        //creates a shallow copy of needsExecution to loop through
        ArrayList<Instruction> executionCopy = (ArrayList<Instruction>) needsExecution.clone();

        for (Instruction i : executionCopy) {
            pruneInstruction(i);
        }


        //runs find instructions on every tag in tags
        while (!tags.isEmpty()){
            findInstructions(tags.poll());
        }

        telemetry.addData("Found Instructions", needsExecution.toString());



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
     * executes the instruction given
     * @param i the instruction to be run against
     */
    private void executeInstruction(Instruction i) {
        //runs the code
        i.getExecutable().execute();
    }

    /**
     * removes the instruction if it is a one time execution, or if it is completed
     * @param i te instruction to be run against
     */
    private void pruneInstruction(Instruction i){
        //checks if the Instruction has a return tag
        if (!i.getHasReturn()){
            //if it does not, removes the instruction, and exits
            needsExecution.remove(i);
            return;
        }

        //checks if the completion condition has been met
        if (checkCompletion(i.getActionDrivers())){
            //if it is met, this will remove it from the list and add its tag to tags
            needsExecution.remove(i);
            tags.add(i.getTag());
            return;
        }
    }

    /**
     * checks if all of the actionDrivers passed to it are no longer busy
     * @param actionDrivers
     * @return
     */
    private boolean checkCompletion(ActionDriver[] actionDrivers){
        for(ActionDriver driver : actionDrivers){
            if (driver.isBusy())
                return false;
        }
        return true;
    }


    /**
     * digs out the instructions that the new tag triggers
     * @param tag the tag that is being checked for as a trigger
     */
    private void findInstructions(String tag) {
        for (Instruction instruction : instructions) {
            if (instruction.getTrigger().equalsIgnoreCase(tag)) {
                needsExecution.add(instruction);
            }
        }
    }

    public String getTags(){
        return tags.toString();
    }
}

