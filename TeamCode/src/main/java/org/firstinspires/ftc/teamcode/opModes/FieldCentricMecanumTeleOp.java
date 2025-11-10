package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * ✅ Merged Field-Centric TeleOp MAIN (SDK 8.x Safe)
 *
 * Gamepad1:
 *  - Field-centric drive (with IMU reset)
 *  - Left stick: Move (field oriented)
 *  - Right stick: Rotate
 *  - Options button: Reset IMU yaw
 *
 * Gamepad2:
 *  - Gecko wheels: A/B/Y = spin speeds
 *  - Intake: Triggers = intake + servos | Bumpers = servos only
 */
@TeleOp(name = "Merged Field-Centric TeleOp MAIN", group = "TeleOp")
public class FieldCentricMecanumTeleOp extends LinearOpMode {

    // --- Drive Motors ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- Function Motors ---
    private DcMotor geckoLeft, geckoRight, intakeMotor;

    // --- Servos ---
    private Servo intakeServoLeft, intakeServoRight;

    // --- IMU ---
    private IMU imu;

    // --- Constants ---
    private static final double SERVO_STOP = 0.5;
    private static final double SERVO_FORWARD = 1.0;
    private static final double SERVO_REVERSE = 0.0;
    private static final double MAX_INTAKE_POWER = 0.45;
    private static final double TRIGGER_DEADZONE = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        // -----------------------------
        // HARDWARE MAP
        // -----------------------------
        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");

        geckoLeft = hardwareMap.get(DcMotor.class, "left_gecko");
        geckoRight = hardwareMap.get(DcMotor.class, "right_gecko");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        intakeServoLeft = hardwareMap.get(Servo.class, "left_servo");
        intakeServoRight = hardwareMap.get(Servo.class, "right_servo");

        // -----------------------------
        // MOTOR CONFIGURATION
        // -----------------------------
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        setBrake(frontLeft, frontRight, backLeft, backRight);
        setRunWithoutEncoder(frontLeft, frontRight, backLeft, backRight);

        geckoLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        geckoRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        setBrake(geckoLeft, geckoRight, intakeMotor);
        resetAndRunWithoutEncoder(geckoLeft, geckoRight, intakeMotor);

        // -----------------------------
        // IMU SETUP
        // -----------------------------
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        telemetry.addLine("Initializing IMU...");
        telemetry.update();

        imu.initialize(parameters);    // ← Auto-calibrates
        imu.resetYaw();                // ← Always zeroed before each match

        telemetry.addLine("IMU Initialized and Reset ✅");
        telemetry.update();

        // -----------------------------
        // INITIALIZE SERVOS
        // -----------------------------
        setIntakeServos(SERVO_STOP);

        telemetry.addLine("Field-Centric + Mechanisms Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // -----------------------------
        // MAIN LOOP
        // -----------------------------
        while (opModeIsActive()) {
            // =======================================================
            // GAMEPAD 1: FIELD-CENTRIC DRIVE
            // =======================================================
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) imu.resetYaw(); // manual re-zero mid-match

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double fl = (rotY + rotX + rx) / denominator;
            double bl = (rotY - rotX + rx) / denominator;
            double fr = (rotY - rotX - rx) / denominator;
            double br = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(fl);
            backLeft.setPower(bl);
            frontRight.setPower(fr);
            backRight.setPower(br);

            // =======================================================
            // GAMEPAD 2: FUNCTION SYSTEM
            // =======================================================
            double geckoPower = 0.0;
            if (gamepad2.a) geckoPower = 0.55;
            else if (gamepad2.b) geckoPower = 0.575;
            else if (gamepad2.y) geckoPower = 0.60;

            geckoLeft.setPower(-geckoPower);
            geckoRight.setPower(-geckoPower);

            double rt = gamepad2.right_trigger;
            double lt = gamepad2.left_trigger;
            boolean rb = gamepad2.right_bumper;
            boolean lb = gamepad2.left_bumper;

            double intakePower = 0.0;
            boolean triggerActive = (rt > TRIGGER_DEADZONE) || (lt > TRIGGER_DEADZONE);

            if (triggerActive) {
                // Intake + Servos (manual)
                intakePower = Range.clip(rt - lt, -MAX_INTAKE_POWER, MAX_INTAKE_POWER);
                intakeMotor.setPower(intakePower);
                if (intakePower > 0.0) setIntakeServos(SERVO_FORWARD);
                else if (intakePower < 0.0) setIntakeServos(SERVO_REVERSE);
                else setIntakeServos(SERVO_STOP);

            } else if (rb || lb) {
                // Servos only
                intakeMotor.setPower(0.0);
                setIntakeServos(rb ? SERVO_FORWARD : SERVO_REVERSE);

            } else if (geckoPower > 0.0) {
                // Gecko spinning: servos stay stopped
                intakeMotor.setPower(0.0);
                setIntakeServos(SERVO_STOP);

            } else {
                // No input
                intakeMotor.setPower(0.0);
                setIntakeServos(SERVO_STOP);
            }

            // =======================================================
            // TELEMETRY
            // =======================================================
            telemetry.addLine("=== FIELD-CENTRIC DRIVE ===");
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(botHeading));

            telemetry.addLine("=== FUNCTION SYSTEM ===");
            telemetry.addData("Gecko Power", "%.2f", geckoPower);
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Servos", "L: %.2f | R: %.2f",
                    intakeServoLeft.getPosition(), intakeServoRight.getPosition());
            telemetry.update();
        }

        stopAll();
    }

    // -----------------------------
    // HELPERS
    // -----------------------------
    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setRunWithoutEncoder(DcMotor... motors) {
        for (DcMotor m : motors) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void resetAndRunWithoutEncoder(DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void setIntakeServos(double leftPosition) {
        leftPosition = Range.clip(leftPosition, 0.0, 1.0);
        double rightPosition = 1.0 - leftPosition;
        intakeServoLeft.setPosition(leftPosition);
        intakeServoRight.setPosition(rightPosition);
    }

    private void stopAll() {
        geckoLeft.setPower(0);
        geckoRight.setPower(0);
        intakeMotor.setPower(0);
        setIntakeServos(SERVO_STOP);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
