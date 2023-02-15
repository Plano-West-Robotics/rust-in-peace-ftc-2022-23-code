package org.firstinspires.ftc.teamcode.driveobjs.instructables.instructedAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.ClawDriver;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.EnhancedDriver;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.LinearSlideDriver;

public abstract class InstructableBase extends LinearOpMode {
    ClawDriver claw = new ClawDriver(hardwareMap);
    EnhancedDriver driver = new EnhancedDriver(hardwareMap);
    LinearSlideDriver slide = new LinearSlideDriver(hardwareMap);

    Telemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());


    /**
     * waits for the tag
     * this already implements !isStarted && !isStopRequested()
     * @return the tag
     */
    int waitForTag(){

        int parkPosition = 0;
        AprilTagDetector detector = null;


        while (!isStarted() && !isStopRequested()) {
            detector = new AprilTagDetector(hardwareMap);
            while (!isStarted() && !isStopRequested()) {
                parkPosition = detector.getPos();
                telemetry.addLine(String.format("\nDetected tag ID=%d", parkPosition));
                telemetry.update();
                sleep(50);
            }

        }

        if (parkPosition < 1){
            long startTime = System.currentTimeMillis();
            for (int i = 0; i < 50; i++ ) {
                parkPosition = detector.getPos();
                telemetry.addLine(String.format("\nDetected tag ID=%d", parkPosition));
                telemetry.update();
                sleep(50);
            }
        }

        detector.endStream();

        if (parkPosition < 1){
            parkPosition = 1;
        }

        return parkPosition;
    }
}
