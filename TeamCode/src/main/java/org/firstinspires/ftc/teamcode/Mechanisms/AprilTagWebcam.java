package org.firstinspires.ftc.teamcode.Mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "webcam"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {

        // ---------------------
        // SAFE NULL CHECK FIRST
        // ---------------------
        if (detectedId == null) {
            telemetry.addLine("No AprilTag detected.");
            return;
        }

        // If tag has metadata (FTC official tag)
        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %.1f %.1f %.1f (cm)",
                    detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %.1f %.1f %.1f (deg)",
                    detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %.1f %.1f %.1f (cm, deg, deg)",
                    detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        }

        // Unknown tag (not an FTC field tag)
        else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown Tag", detectedId.id));
            telemetry.addLine(String.format("Center %.0f %.0f (pixels)",
                    detectedId.center.x, detectedId.center.y));
        }
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
