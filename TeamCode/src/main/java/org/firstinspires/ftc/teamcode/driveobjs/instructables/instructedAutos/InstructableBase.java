package org.firstinspires.ftc.teamcode.driveobjs.instructables.instructedAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;

public abstract class InstructableBase extends LinearOpMode {

    Telemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
    int parkPosition = 0;

    /**
     * waits for the tag
     * this already implements !isStarted && !isStopRequested()
     * @return the tag
     */
    int waitForTag(AprilTagDetector detector){
        parkPosition = detector.getPos();
        sleep(50);

        return parkPosition;

    }
}
