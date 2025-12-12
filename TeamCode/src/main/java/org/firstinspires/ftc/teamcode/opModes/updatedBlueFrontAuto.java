package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanisms.updatedMech;

@Autonomous(name = "UPDATED BLUE FRONT AUTO (42in + Shooting)", group = "Auto")
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
        // 1️⃣ Move forward 42 inches
        // ---------------------------------------------------------------------
        telemetry.addLine("Moving Forward 42 inches...");
        telemetry.update();
        robot.strafe("forward", (int)(42 * updatedMech.TICKS_PER_INCH), 0.5);
        waitUntilDriveComplete();

        // ---------------------------------------------------------------------
        // 🔥 Shooting Sequence
        // ---------------------------------------------------------------------
        telemetry.addLine("Running Shooting Sequence...");
        telemetry.update();

        shootingSequence();

        telemetry.addLine("Shooting Complete!");
        telemetry.update();

        // ---------------------------------------------------------------------
        // 2️⃣ Turn LEFT 140°
        // ---------------------------------------------------------------------
        telemetry.addLine("Turning Left 140°...");
        telemetry.update();
        robot.turn(-140, 0.5);
        waitUntilDriveComplete();

        // ---------------------------------------------------------------------
        // 3️⃣ Move forward 24 inches (2 ft)
        // ---------------------------------------------------------------------
        telemetry.addLine("Moving Forward 24 inches...");
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
    // 🔥 FULL SHOOTING FUNCTION
    // ------------------------------------------------------------
    private void shootingSequence() {

        // --------------------------------------------------------
        // Phase 1: Conveyor + both geckos for 1.5 seconds
        // --------------------------------------------------------
        robot.conveyer_belt.setPower(-0.7);   // same direction as TeleOp (right bumper)
        robot.left_gecko.setPower(-0.4);
        robot.right_gecko.setPower(0.4);

        sleep(1500);

        // --------------------------------------------------------
        // Phase 2: Intake ON for 0.25 seconds
        // --------------------------------------------------------
        robot.intake.setPower(-0.6);
        sleep(100);

        // Intake OFF briefly
        robot.intake.setPower(0);
        sleep(1000);
        // --------------------------------------------------------
        // Phase 3: Intake ON again for 3 seconds
        // --------------------------------------------------------
        robot.intake.setPower(-0.75);
        sleep(250);

        robot.intake.setPower(0);
        sleep(1000);

        robot.intake.setPower(-0.55);
        sleep(2000);
        // --------------------------------------------------------
        // STOP ALL SHOOTER MECHANISMS
        // --------------------------------------------------------
        robot.conveyer_belt.setPower(0);
        robot.left_gecko.setPower(0);
        robot.right_gecko.setPower(0);
        robot.intake.setPower(0);
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

        // Reset servo
        if (robot.gateServo != null) robot.gateServo.setPosition(0.0);
    }
}
