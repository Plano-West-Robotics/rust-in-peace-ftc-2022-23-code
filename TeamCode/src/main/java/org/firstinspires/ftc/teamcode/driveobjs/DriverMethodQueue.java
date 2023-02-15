package org.firstinspires.ftc.teamcode.driveobjs;

import org.firstinspires.ftc.teamcode.driveobjs.drivers.EnhancedDriver;

import java.util.ArrayList;
import java.util.List;


/**
 * THIS SHOULD REALLY IMPLEMENT ITERABLE (or extend List/Queue) but i am lazy so
 */
public class DriverMethodQueue {
    List<ActionObject> queue;

    public DriverMethodQueue(){
        queue = new ArrayList<>();
    }

    public DriverMethodQueue add(String methodName, Object... args){
        queue.add(new ActionObject(methodName, args));
        return this;
    }

    public DriverMethodQueue add(String methodName){
        queue.add(new ActionObject(methodName));
        return this;
    }



    public int length(){
        return queue.size();
    }

    public String getMethodName(int index){
        return queue.get(index).getMethodName();
    }
    public Object[] getArgs(int index){
        return queue.get(index).getArgs();
    }

    public ActionObject getActionObject(int index){
        return queue.get(index);
    }

    public void validate(EnhancedDriver enhancedDriver){

    }

}
