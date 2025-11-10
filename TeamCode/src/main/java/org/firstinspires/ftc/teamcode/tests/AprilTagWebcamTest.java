package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagWebcamTest  extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();


    @Override
    public void init(){
        aprilTagWebcam.init(hardwareMap, telemetry);
    }
    @Override
    public void loop(){
        //update vision portal
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);
    }
}
