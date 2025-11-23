package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanisms.updatedMech;

@Autonomous(name = "Updated RED BACK AUTO", group = "Auto")
public class updatedRedBackAuto extends LinearOpMode {

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
        // 🔹 SHOOTING SEQUENCE — 2s Gecko, then 4s Gecko+Intake+Conveyor
        // ---------------------------------------------------------------------
        telemetry.addLine("Starting Shooting Sequence...");
        telemetry.update();

        // --- Phase 1: Gecko wheels only (2 seconds) ---
        robot.left_gecko.setPower(-0.53);
        robot.right_gecko.setPower(0.53);
        robot.intake.setPower(0);
        robot.conveyer_belt.setPower(0);

        sleep(2000);

        // --- Phase 2: Gecko + Intake + Conveyor (4 seconds) ---
        robot.intake.setPower(-0.55);        // negative as requested
        robot.conveyer_belt.setPower(-1.0);  // negative same as right bumper
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
        // 🔹 MOVEMENT — Strafe Left 2 ft
        // ---------------------------------------------------------------------
        telemetry.addLine("Moving Left 2ft...");
        telemetry.update();

        robot.strafe("left", (int)(24 * updatedMech.TICKS_PER_INCH), 0.5);
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

    /** Wait until drive is done (encoder-based) */
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

    /** Reset encoders and drivetrain */
    private void resetAllMechanisms() {
        robot.resetDriveEncoders();
        robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.stopDrive();
    }
}
