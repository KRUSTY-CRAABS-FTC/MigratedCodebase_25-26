package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanisms.Mechanisms;

@Autonomous(name = "BLUE FRONT AUTO", group = "Auto")
public class blueFrontAuto extends LinearOpMode {

    private final Mechanisms robot = new Mechanisms();

    // --- Constants ---
    private static final double SERVO_STOP = 0.5;
    private static final double SERVO_FORWARD = 1.0;
    private static final double GECKO_POWER = 0.53;
    private static final double INTAKE_POWER = 0.45;

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
        // 🔹 AUTONOMOUS SEQUENCE
        // ---------------------------------------------------------------------

        // 0️⃣ Move forward 6 feet (72 inches) — added as requested
        telemetry.addLine("Moving Forward 4ft...");
        telemetry.update();
        robot.strafe("forward", (int)(48 * Mechanisms.TICKS_PER_INCH), 0.5);
        waitUntilDriveComplete();

        // 1️⃣ Shoot for 4 seconds
        telemetry.addLine("Shooting Sequence Starting...");
        telemetry.update();
        runFullShootSequence(GECKO_POWER, 0.6, 4000);

        // 3️⃣ Turn right 140 degrees
        telemetry.addLine("Turning Right 140°...");
        telemetry.update();
        // Assuming positive angle = left, use -140 for right turn
        robot.turn(-140, 0.5);
        waitUntilDriveComplete();

        // 2️⃣ Move forward 2 feet (24 inches)
        telemetry.addLine("Moving Forward 2ft...");
        telemetry.update();
        robot.strafe("forward", (int)(24 * Mechanisms.TICKS_PER_INCH), 0.5);
        waitUntilDriveComplete();


        // ---------------------------------------------------------------------
        // 🔹 END
        // ---------------------------------------------------------------------

        stopAllMechanisms();
        telemetry.addLine("✅ Auto Complete!");
        telemetry.update();
        sleep(1000);
    }

    // ------------------------------------------------------------
    // 🧩 HELPER FUNCTIONS
    // ------------------------------------------------------------

    /** Wait until drive is done (encoder based) */
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

    /** Reset encoders, motors, and servos */
    private void resetAllMechanisms() {
        robot.resetDriveEncoders();
        robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.stopDrive();

        if (robot.left_gecko != null) {
            robot.left_gecko.setPower(0);
            robot.left_gecko.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (robot.right_gecko != null) {
            robot.right_gecko.setPower(0);
            robot.right_gecko.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (robot.intake != null) {
            robot.intake.setPower(0);
            robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (robot.left_servo != null) robot.left_servo.setPosition(SERVO_STOP);
        if (robot.right_servo != null) robot.right_servo.setPosition(SERVO_STOP);
    }

    /** Safely stop all motion */
    private void stopAllMechanisms() {
        stopIntakeAndServos();
        if (robot.left_gecko != null) robot.left_gecko.setPower(0);
        if (robot.right_gecko != null) robot.right_gecko.setPower(0);
        robot.stopDrive();
    }

    // ------------------------------------------------------------
    // 🌀 INTAKE + SERVOS
    // ------------------------------------------------------------

    private void startIntakeAndServos(double intakePower) {
        if (robot.intake != null) robot.intake.setPower(-intakePower);
        setIntakeServos(SERVO_FORWARD);
    }

    private void stopIntakeAndServos() {
        if (robot.intake != null) robot.intake.setPower(0);
        setIntakeServos(SERVO_STOP);
    }

    private void setIntakeServos(double leftPosition) {
        leftPosition = Math.max(0.0, Math.min(1.0, leftPosition));
        double rightPosition = 1.0 - leftPosition;
        if (robot.left_servo != null) robot.left_servo.setPosition(leftPosition);
        if (robot.right_servo != null) robot.right_servo.setPosition(rightPosition);
    }

    // ------------------------------------------------------------
    // 🎯 SHOOTING SEQUENCE
    // ------------------------------------------------------------

    private void runFullShootSequence(double geckoPower, double intakePower, long totalMs) {
        if (robot.left_gecko == null || robot.right_gecko == null) return;

        // Spin up geckos
        robot.left_gecko.setPower(-geckoPower);
        robot.right_gecko.setPower(geckoPower);
        telemetry.addLine("🌀 Gecko Spinning Up...");
        telemetry.update();
        sleep(1500); // spin-up delay

        // Start intake and servos
        startIntakeAndServos(intakePower);
        telemetry.addLine("🎯 Feeding projectiles...");
        telemetry.update();

        long start = System.currentTimeMillis();
        boolean intakeReversed = false;

        while (opModeIsActive() && System.currentTimeMillis() - start < totalMs) {
            long elapsed = System.currentTimeMillis() - start;

            // Reverse intake after 3 seconds
            if (elapsed >= 3000 && !intakeReversed) {
                if (robot.intake != null) robot.intake.setPower(intakePower); // reverse direction
                intakeReversed = true;
                telemetry.addLine("↩️ Intake reversed...");
                telemetry.update();
            }

            idle();
        }

        // Stop all shooting actions
        stopIntakeAndServos();
        robot.left_gecko.setPower(0);
        robot.right_gecko.setPower(0);
        telemetry.addLine("✅ Shooting Complete");
        telemetry.update();
    }
}
