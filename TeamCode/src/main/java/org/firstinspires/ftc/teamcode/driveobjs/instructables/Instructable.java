package org.firstinspires.ftc.teamcode.driveobjs.instructables;

public interface Instructable {
    /**
     * Creates an Instruction that waits
     * @param triggerTag the tag that this instruction executes after
     * @param returnTag the tag that is returned when this instruction finishes executing
     * @param executable the code to execute
     * @param triggers extra triggers to wait on
     * @return returns the intruction that has been created
     */
    Instruction makeInstruction(String triggerTag, String returnTag, InstructionExecutable executable, String... triggers);

    /**
     * Creates an instruction WITHOUT A RETURN TAG
     * @param triggerTag the tag that this instruction executes after
     * @param executable the code to execute
     * @param triggers extra triggers to wait on
     */
    Instruction makeInstruction(String triggerTag, InstructionExecutable executable, String... triggers);
}



