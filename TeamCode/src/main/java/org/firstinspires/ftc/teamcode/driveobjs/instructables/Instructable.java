package org.firstinspires.ftc.teamcode.driveobjs.instructables;

public interface Instructable {
    Instruction makeInstruction(String triggerTag, String returnTag, InstructionExecutable executable);
    Instruction makeInstruction(String triggerTag, InstructionExecutable executable);
}



