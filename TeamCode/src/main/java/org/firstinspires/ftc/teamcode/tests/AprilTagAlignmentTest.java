package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.Mechanisms.Mechanisms;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "AprilTag Auto Alignment Test", group = "Tests")
public class AprilTagAlignmentTest extends LinearOpMode {

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private Mechanisms robot = new Mechanisms();

    private static final double ALIGN_TOLERANCE_DEG = 7.0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing...");
        telemetry.update();

        robot.init(hardwareMap);
        aprilTagWebcam.init(hardwareMap, telemetry);

        telemetry.addLine("READY - Press PLAY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            aprilTagWebcam.update();
            AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(20);

            if (tag == null) {
                telemetry.addLine("No Tag Detected - Holding");
                robot.stopDrive();
                telemetry.update();
                continue;
            }

            double yaw = tag.ftcPose.yaw;

            // We want the BACK of the robot toward the tag → 180 degrees
            double targetAngle = 180;
            double error = yaw - targetAngle;

            // Normalize to [-180, 180]
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            telemetry.addData("Yaw", yaw);
            telemetry.addData("Error", error);
            telemetry.update();

            // Stop if aligned
            if (Math.abs(error) <= ALIGN_TOLERANCE_DEG) {
                robot.stopDrive();
                telemetry.addLine("✓ Back Aligned to Tag");
                telemetry.update();
                continue;
            }

            // -------- OPEN-LOOP TURN CONTROL (NEVER overshoots) ----------
            // Proportional gain for gentle turning
            double k = 0.015;
            double turnPower = k * error;

            // Clamp power to slow + safe turning
            turnPower = Math.max(-0.25, Math.min(0.25, turnPower));

            // Apply tank turn using your motor names
            robot.frontLeftMotor.setPower(turnPower);
            robot.backLeftMotor.setPower(turnPower);
            robot.frontRightMotor.setPower(-turnPower);
            robot.backRightMotor.setPower(-turnPower);

            sleep(25); // allows camera to update and prevents overshoot
        }

        robot.stopDrive();
    }
}
