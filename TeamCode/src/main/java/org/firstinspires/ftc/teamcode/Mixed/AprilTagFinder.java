package org.firstinspires.ftc.teamcode.Mixed;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagFinder {


//THIS CODE RUNS WITH THE APRIL TAG DRIVER - DO NOT DELETE


    private AprilTagProcessor aprilTag;
    private Telemetry telemetry;

    public AprilTagFinder(AprilTagProcessor aprilTag, Telemetry telemetry) {
        this.aprilTag = aprilTag;
        this.telemetry = telemetry;
    }

    public AprilTagDetection findTag(int desiredTagId) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((desiredTagId < 0) || (detection.id == desiredTagId)) {
                    return detection;  // Return the detected tag
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        return null;  // Return null if no desired tag is found
    }
}
