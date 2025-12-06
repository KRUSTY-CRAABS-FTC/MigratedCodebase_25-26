package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanisms.updatedMech;

@Autonomous(name = "UPDATED Red BACK AUTO (Shooting + Left 2ft)", group = "Auto")
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
        // 🔥 Shooting Sequence (FIRST)
        // ---------------------------------------------------------------------
        telemetry.addLine("Running Shooting Sequence...");
        telemetry.update();

        shootingSequence();

        telemetry.addLine("Shooting Complete!");
        telemetry.update();

        // ---------------------------------------------------------------------
        // 2️⃣ Strafe RIGHT 2 feet (24 inches)
        // ---------------------------------------------------------------------
        telemetry.addLine("Strafing left 24 inches...");
        telemetry.update();
        robot.strafe("left", (int)(24 * updatedMech.TICKS_PER_INCH), 0.5);
        waitUntilDriveComplete();

        // ---------------------------------------------------------------------
        // 🔹 END
        // ---------------------------------------------------------------------
        robot.stopDrive();
        telemetry.addLine("✅ Blue Back Auto Complete!");
        telemetry.update();
        sleep(1000);
    }

    // ------------------------------------------------------------
    // 🔥 SHOOTING SEQUENCE (UNCHANGED)
    // ------------------------------------------------------------
    private void shootingSequence() {

        // Phase 1: Conveyor + Geckos (1.5s)
        robot.conveyer_belt.setPower(-0.7);
        robot.left_gecko.setPower(-0.53);
        robot.right_gecko.setPower(0.53);
        sleep(1500);

        // Phase 2: Intake short burst
        robot.intake.setPower(-0.6);
        sleep(100);
        robot.intake.setPower(0);
        sleep(1000);

        // Phase 3: Intake pulses
        robot.intake.setPower(-0.6);
        sleep(500);
        robot.intake.setPower(0);
        sleep(1000);

        robot.intake.setPower(-0.6);
        sleep(2000);

        // Stop everything
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
            telemetry.addData("FL", robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("FR", robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("BL", robot.backLeftMotor.getCurrentPosition());
            telemetry.addData("BR", robot.backRightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
        robot.stopDrive();
        sleep(200);
    }

    private void resetAllMechanisms() {
        robot.resetDriveEncoders();
        robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.stopDrive();

        if (robot.left_gecko != null) robot.left_gecko.setPower(0);
        if (robot.right_gecko != null) robot.right_gecko.setPower(0);
        if (robot.intake != null) robot.intake.setPower(0);
        if (robot.conveyer_belt != null) robot.conveyer_belt.setPower(0);

        if (robot.gateServo != null) robot.gateServo.setPosition(0.0);
    }
}
