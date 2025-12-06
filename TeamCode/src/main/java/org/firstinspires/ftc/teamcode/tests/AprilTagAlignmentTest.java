package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.Mechanisms.updatedMech;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "Smooth Back-Facing AprilTag", group = "Tests")
public class AprilTagAlignmentTest extends LinearOpMode {

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private updatedMech robot = new updatedMech();

    private static final double WIGGLE_DEG = 10.0;          // ±10° deadzone
    private static final double SEARCH_TURN_SPEED = 0.15;   // slow search speed
    private static final double MAX_TURN_POWER = 0.3;       // maximum turn power
    private static final double kP = 0.01;                  // proportional gain

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

            // -------------------------
            // SEARCH MODE: rotate slowly if tag is lost
            // -------------------------
            if (tag == null) {
                telemetry.addLine("Tag lost → searching...");
                telemetry.update();

                robot.frontLeftMotor.setPower(SEARCH_TURN_SPEED);
                robot.backLeftMotor.setPower(SEARCH_TURN_SPEED);
                robot.frontRightMotor.setPower(-SEARCH_TURN_SPEED);
                robot.backRightMotor.setPower(-SEARCH_TURN_SPEED);

                sleep(20);
                continue;
            }

            // -------------------------
            // TAG FOUND: compute back-facing error
            // -------------------------
            double x = tag.ftcPose.x; // left/right offset
            double y = tag.ftcPose.y; // forward/backward offset

            double angleToTagDeg = Math.toDegrees(Math.atan2(x, y));
            double backFacingError = angleToTagDeg + 180; // desired back-facing

            // Normalize to [-180, 180]
            while (backFacingError > 180) backFacingError -= 360;
            while (backFacingError < -180) backFacingError += 360;

            telemetry.addData("Tag X", x);
            telemetry.addData("Tag Y", y);
            telemetry.addData("AngleToTag", angleToTagDeg);
            telemetry.addData("BackFacingError", backFacingError);
            telemetry.update();

            // -------------------------
            // ROTATION CONTROL: smooth proportional
            // -------------------------
            double turnPower;

            if (Math.abs(backFacingError) <= WIGGLE_DEG) {
                // Inside deadzone → stop completely
                turnPower = 0;
            } else {
                // Base proportional power
                turnPower = kP * backFacingError;

                // Scale down smoothly if close to deadzone (within 2× WIGGLE_DEG)
                if (Math.abs(backFacingError) < 2 * WIGGLE_DEG) {
                    turnPower *= Math.abs(backFacingError) / (2 * WIGGLE_DEG);
                }

                // Clamp to max turn power
                turnPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnPower));
            }

            // Apply turn power
            robot.frontLeftMotor.setPower(turnPower);
            robot.backLeftMotor.setPower(turnPower);
            robot.frontRightMotor.setPower(-turnPower);
            robot.backRightMotor.setPower(-turnPower);

            // -------------------------
            // Stop and exit if aligned
            // -------------------------
            if (turnPower == 0) {
                robot.stopDrive();
                telemetry.addLine("✓ Back facing AprilTag smoothly within ±10°");
                telemetry.update();
                break; // exit while loop
            }

            sleep(20);
        }

        // Ensure motors are stopped at the end
        robot.stopDrive();
    }
}
