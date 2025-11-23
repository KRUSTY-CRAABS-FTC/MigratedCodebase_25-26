package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanisms.updatedMech;

@Autonomous(name = "Updated RED FRONT AUTO", group = "Auto")
public class updatedRedFrontAuto extends LinearOpMode {

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
        // 0️⃣ Move forward 4 ft (48 inches)
        // ---------------------------------------------------------------------
        telemetry.addLine("Moving Forward 4ft...");
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
        robot.intake.setPower(-0.25);          // negative intake
        robot.conveyer_belt.setPower(-1.0);    // negative conveyor (same as right bumper)
        sleep(2000);
        robot.conveyer_belt.setPower(0);
        sleep(1000);
        // Gecko wheels remain spinning
        robot.conveyer_belt.setPower(-1.0);
        sleep(2000);
        robot.conveyer_belt.setPower(0);
        sleep(1000);
        robot.conveyer_belt.setPower(-1.0);
        sleep(2000);

        // --- Stop all shooter components ---
        robot.left_gecko.setPower(0);
        robot.right_gecko.setPower(0);
        robot.intake.setPower(0);
        robot.conveyer_belt.setPower(0);

        telemetry.addLine("Shooting Complete!");
        telemetry.update();

        // ---------------------------------------------------------------------
        // 1️⃣ Turn left 140 degrees
        // ---------------------------------------------------------------------
        telemetry.addLine("Turning Left 140°...");
        telemetry.update();
        robot.turn(140, 0.5);
        waitUntilDriveComplete();

        // ---------------------------------------------------------------------
        // 2️⃣ Move forward 2 ft (24 inches)
        // ---------------------------------------------------------------------
        telemetry.addLine("Moving Forward 2ft...");
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
        sleep(250);
    }

    private void resetAllMechanisms() {
        robot.resetDriveEncoders();
        robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.stopDrive();
    }
}
