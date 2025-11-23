package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanisms.updatedMech;

@Autonomous(name = "UPDATED BLUE BACK AUTO", group = "Auto")
public class updatedBlueBackAuto extends LinearOpMode {

    private final updatedMech robot = new updatedMech();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing...");
        telemetry.update();

        robot.init(hardwareMap);
        resetAllMechanisms();

        telemetry.addLine("Initialization Complete. Ready to Start!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // --------------------------------------------------------------
        // 🔥 SHOOTING SEQUENCE — 2s Gecko, then 4s Gecko+Intake+Conveyor
        // --------------------------------------------------------------
        telemetry.addLine("Starting Shooting Sequence...");
        telemetry.update();

        // --- Phase 1: Gecko wheels only (2 seconds) ---
        robot.left_gecko.setPower(-0.52);
        robot.right_gecko.setPower(0.52);
        robot.intake.setPower(0);
        robot.conveyer_belt.setPower(0);
        sleep(2000);

        // --- Phase 2: Gecko + Intake + Conveyor (4 seconds) ---
        robot.intake.setPower(-0.55);          // negative intake
        robot.conveyer_belt.setPower(-1.0);
        // negative conveyor (same as right bumper)
        sleep(2000);
        robot.conveyer_belt.setPower(0);
        robot.intake.setPower(0);
        sleep(1000);
        // Gecko wheels remain spinning
        robot.conveyer_belt.setPower(-1.0);
        robot.intake.setPower(-0.55);
        sleep(2000);
        robot.conveyer_belt.setPower(0);
        robot.intake.setPower(0);
        sleep(1000);
        robot.conveyer_belt.setPower(-1.0);
        robot.intake.setPower(-0.55);
        sleep(2000);
        // Gecko wheels remain spinning

        // --- Stop all shooter components ---
        robot.left_gecko.setPower(0);
        robot.right_gecko.setPower(0);
        robot.intake.setPower(0);
        robot.conveyer_belt.setPower(0);

        telemetry.addLine("Shooting Complete!");
        telemetry.update();

        // --------------------------------------------------------------
        // STRAFE RIGHT 24 INCHES
        // --------------------------------------------------------------
        telemetry.addLine("Strafing right 24 inches...");
        telemetry.update();

        robot.strafe(
                "right",
                (int)(24 * updatedMech.TICKS_PER_INCH),
                0.5
        );

        waitUntilDriveComplete();

        telemetry.addLine("Auto complete.");
        telemetry.update();
    }

    // ------------------------------------------------------------
    // HELPER FUNCTIONS
    // ------------------------------------------------------------

    /** Wait until drive is done */
    private void waitUntilDriveComplete() {
        while (opModeIsActive() && robot.isDriveBusy()) {
            telemetry.addData("FL Encoder", robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("FR Encoder", robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("BL Encoder", robot.backLeftMotor.getCurrentPosition());
            telemetry.addData("BR Encoder", robot.backRightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
        robot.stopDrive();
        sleep(200);
    }

    /** Reset drivetrain + attachments */
    private void resetAllMechanisms() {
        robot.resetDriveEncoders();
        robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop attachments safely
        if (robot.left_gecko != null) robot.left_gecko.setPower(0);
        if (robot.right_gecko != null) robot.right_gecko.setPower(0);
        if (robot.intake != null) robot.intake.setPower(0);
        if (robot.conveyer_belt != null) robot.conveyer_belt.setPower(0);

        // Gate servo neutral
        if (robot.gateServo != null) robot.gateServo.setPosition(0.0);
    }
}
