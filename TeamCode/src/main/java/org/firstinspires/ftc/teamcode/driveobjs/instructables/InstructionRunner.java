package org.firstinspires.ftc.teamcode.driveobjs.instructables;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.driveobjs.drivers.ActionDriver;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class InstructionRunner {
    private Pose2d startPose;
    HardwareMap hardwareMap;
    ActionDriver[] actionDrivers;

    //stores the instructions
    ArrayList<Instruction> instructions = new ArrayList<Instruction>();

    //stores the instructions that need execution
    ArrayList<Instruction> needsExecution = new ArrayList<Instruction>();



    /**
     * Creates the instruction runner
     * @param hardwareMap must provide hardwareMap
     * @param startPose give the initial Pose2d
     * @param initialTag set the first tag to run
     * @param actionDrivers gives the list of drivers to run asyncrhonously
     */
    public InstructionRunner(HardwareMap hardwareMap, Pose2d startPose, String initialTag, ActionDriver... actionDrivers) {
        this.startPose = startPose;
        this.hardwareMap = hardwareMap;
        tags.add(initialTag);
        this.actionDrivers = actionDrivers;
    }


    /**
     * creates and adds and instruction
     * not recommended for use, use the drivers' specific makeInstructions instead
     * @param triggerTag the tag that this instruction executes after
     * @param returnTag the tag that is returned when this instruction finishes executing
     * @param executable the code to execute
     */
    public void addInstruction(String triggerTag, String returnTag, InstructionExecutable executable, ActionDriver... actionDrivers) {
        instructions.add(new Instruction(triggerTag, returnTag, executable, actionDrivers));
    }

    /**
     * adds the instruction to the list
     * preferred over the other variant
     * @param instruction the instruction to be added
     */
    public void addInstruction(Instruction instruction){
        instructions.add(instruction);
    }

    Queue<String> tags = new LinkedList<String>();

    /**
     * primary execution loop for InstructionRunner
     * this is intended to be looped to follow the list of instructions
     */
    public void run() {
        //runs find instructions on every tag in tags
        for (int i = 0; i < tags.size(); i++){
            findInstructions(tags.poll());
        }

        //creates a shallow copy of needsExecution to loop through
        ArrayList<Instruction> executionCopy = new ArrayList<Instruction>(needsExecution);

        //executes those tags
        for (Instruction i : executionCopy) {
            executeInstruction(i);
        }

        //runs the idles for the drivers
        for(ActionDriver driver : actionDrivers){
            driver.run();
        }

    }

    /**
     * executes the instruction given, and removes it if it is a one time execution, or if it is completed
     * @param i the instruction to be run against
     */
    private void executeInstruction(Instruction i) {
        //checks if the Instruction has a return tag
        if (!i.getHasReturn()){
            //if it does not, simply runs the code inside, removes the instruction, and exits
            i.getExecutable().execute();
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

        //if all of those checks are false, runs the code
        i.getExecutable().execute();

    }

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
            if (instruction.getTrigger().equals(tag)) {
                needsExecution.add(instruction);
            }
        }
    }
}

