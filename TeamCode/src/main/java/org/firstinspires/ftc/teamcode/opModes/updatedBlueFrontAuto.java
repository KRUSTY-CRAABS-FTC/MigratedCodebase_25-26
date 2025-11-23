package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanisms.updatedMech;

@Autonomous(name = "UPDATED BLUE FRONT AUTO", group = "Auto")
public class updatedBlueFrontAuto extends LinearOpMode {

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

        // ---------------------------------------------------------------------
        // 1️⃣ Move forward 4 ft (48 inches)
        // ---------------------------------------------------------------------
        telemetry.addLine("Moving Forward 4 ft...");
        telemetry.update();
        robot.strafe("forward", (int)(48 * updatedMech.TICKS_PER_INCH), 0.5);
        waitUntilDriveComplete();

        // ---------------------------------------------------------------------
        // 🔥 SHOOTING SEQUENCE — 2s Gecko, then 4s Gecko+Intake+Conveyor
        // ---------------------------------------------------------------------
        telemetry.addLine("Starting Shooting Sequence...");
        telemetry.update();

        // --- Phase 1: Gecko wheels only (2 seconds) ---
        robot.left_gecko.setPower(-0.45);
        robot.right_gecko.setPower(0.45);
        robot.intake.setPower(0);
        robot.conveyer_belt.setPower(0);
        sleep(2000);

        // --- Phase 2: Gecko + Intake + Conveyor (4 seconds) ---
        robot.intake.setPower(-0.55);          // negative intake
        robot.conveyer_belt.setPower(-1.0);    // negative conveyor (same as right bumper)
        // Gecko wheels remain spinning
        sleep(4000);

        // --- Stop all shooter components ---
        robot.left_gecko.setPower(0);
        robot.right_gecko.setPower(0);
        robot.intake.setPower(0);
        robot.conveyer_belt.setPower(0);

        telemetry.addLine("Shooting Complete!");
        telemetry.update();

        // ---------------------------------------------------------------------
        // 2️⃣ Turn right 140°
        // ---------------------------------------------------------------------
        telemetry.addLine("Turning Right 140°...");
        telemetry.update();
        robot.turn(-140, 0.5); // negative = right turn
        waitUntilDriveComplete();

        // ---------------------------------------------------------------------
        // 3️⃣ Move forward 2 ft (24 inches)
        // ---------------------------------------------------------------------
        telemetry.addLine("Moving Forward 2 ft...");
        telemetry.update();
        robot.strafe("forward", (int)(24 * updatedMech.TICKS_PER_INCH), 0.5);
        waitUntilDriveComplete();

        // ---------------------------------------------------------------------
        // 🔹 END
        // ---------------------------------------------------------------------
        robot.stopDrive();
        telemetry.addLine("✅ Auto Complete!");
        telemetry.update();
        sleep(1000);
    }

    // ------------------------------------------------------------
    // 🧩 HELPER FUNCTIONS
    // ------------------------------------------------------------

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

    private void resetAllMechanisms() {
        // Reset drivetrain
        robot.resetDriveEncoders();
        robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.stopDrive();

        // Stop attachments (for safety)
        if (robot.left_gecko != null) robot.left_gecko.setPower(0);
        if (robot.right_gecko != null) robot.right_gecko.setPower(0);
        if (robot.intake != null) robot.intake.setPower(0);
        if (robot.conveyer_belt != null) robot.conveyer_belt.setPower(0);

        // Reset gate servo
        if (robot.gateServo != null) robot.gateServo.setPosition(0.0);
    }
}
