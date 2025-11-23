package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagWebcamTest extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // Update detection list
        aprilTagWebcam.update();

        // Try to get tag ID 20
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);

        // Safe telemetry (won't crash)
        aprilTagWebcam.displayDetectionTelemetry(id20);

        telemetry.update();
    }
}
