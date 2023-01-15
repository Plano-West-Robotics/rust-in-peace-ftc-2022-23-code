package org.firstinspires.ftc.teamcode.driveobjs;

public class ActionObject {
    String methodName;
    Object[] args;

    public ActionObject(String methodName, Object[] args){
        this.methodName = methodName;
        this.args = args;
    }
    public ActionObject(String methodName){
        this.methodName = methodName;
        this.args = null;
    }

    public String getMethodName() {
        return methodName;
    }

    public Object[] getArgs() {
        return args;
    }
}
