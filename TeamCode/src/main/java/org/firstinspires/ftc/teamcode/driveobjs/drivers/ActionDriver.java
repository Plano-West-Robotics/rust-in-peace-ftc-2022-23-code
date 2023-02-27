package org.firstinspires.ftc.teamcode.driveobjs.drivers;

public interface ActionDriver {
    void init();

    /**
     * passive action of the driver
     */
    void run();

    /**
     *
     * @return whether or not this driver is busy
     */
    boolean isBusy();
}


