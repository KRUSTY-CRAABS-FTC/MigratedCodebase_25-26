package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * FunctionTest (Event-Driven) - Gecko-linked Servo Feed
 *
 * Controls (gamepad2):
 *  - Gecko Wheels:
 *      - A = low (0.62)
 *      - B = medium (0.65)
 *      - Y = high (0.69)
 *      - Servos auto-run forward when gecko at power > 0.60
 *      - Release button → stops servos + gecko
 *
 *  - Intake System (manual):
 *      - Right Trigger → Intake + Servos forward
 *      - Left Trigger  → Intake + Servos reverse
 *      - Right Bumper  → Servos forward only
 *      - Left Bumper   → Servos reverse only
 *      - Otherwise → stopped
 *
 * Notes:
 *  - Intake motor reversed so direction matches servos.
 *  - Servos use full range (0.0–1.0) for speed.
 *  - Intake capped at ±0.85.
 */
@TeleOp(name = "Function Test (Event Driven) - Gecko Auto Servo", group = "TeleOp")
public class FunctionTest extends LinearOpMode {

    // --- Motors ---
    private DcMotor geckoLeft;
    private DcMotor geckoRight;
    private DcMotor intakeMotor;

    // --- Servos ---
    private Servo intakeServoLeft;
    private Servo intakeServoRight;

    // --- Constants ---
    private static final double SERVO_STOP = 0.5;
    private static final double SERVO_FORWARD = 1.0;
    private static final double SERVO_REVERSE = 0.0;
    private static final double MAX_INTAKE_POWER = 0.45;
    private static final double TRIGGER_DEADZONE = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware mapping ---
        geckoLeft = hardwareMap.get(DcMotor.class, "left_gecko");
        geckoRight = hardwareMap.get(DcMotor.class, "right_gecko");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        intakeServoLeft = hardwareMap.get(Servo.class, "left_servo");
        intakeServoRight = hardwareMap.get(Servo.class, "right_servo");

        // --- Motor setup ---
        geckoLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        geckoRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // reversed for matching direction

        geckoLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        geckoRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Reset encoders ---
        telemetry.addLine("Resetting encoders...");
        telemetry.update();
        resetAndRunWithoutEncoder(geckoLeft);
        resetAndRunWithoutEncoder(geckoRight);
        resetAndRunWithoutEncoder(intakeMotor);
        telemetry.addLine("Encoders reset and motors ready.");
        telemetry.update();

        // --- Initialize servos ---
        setIntakeServos(SERVO_STOP);
        telemetry.addLine("Servos set to STOP");
        telemetry.addLine("Ready for start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Gecko Wheels (hold A/B/Y) ---
            double geckoPower = 0.0;

            if (gamepad2.a) geckoPower = 0.62;
            else if (gamepad2.b) geckoPower = 0.65;
            else if (gamepad2.y) geckoPower = 0.69;

            geckoLeft.setPower(-geckoPower);
            geckoRight.setPower(-geckoPower);

            boolean geckoActive = geckoPower > 0.0;

            // --- Intake / Servo logic ---
            double rt = gamepad2.right_trigger;
            double lt = gamepad2.left_trigger;
            boolean rb = gamepad2.right_bumper;
            boolean lb = gamepad2.left_bumper;

            double intakePower = 0.0;
            boolean triggerActive = (rt > TRIGGER_DEADZONE) || (lt > TRIGGER_DEADZONE);

            if (triggerActive) {
                // Triggers control intake + servos
                intakePower = Range.clip(rt - lt, -MAX_INTAKE_POWER, MAX_INTAKE_POWER);
                intakeMotor.setPower(intakePower);

                if (intakePower > 0.0) setIntakeServos(SERVO_FORWARD);
                else if (intakePower < 0.0) setIntakeServos(SERVO_REVERSE);
                else setIntakeServos(SERVO_STOP);

            } else if (rb || lb) {
                // Bumpers control servos only
                intakeMotor.setPower(0.0);
                setIntakeServos(rb ? SERVO_FORWARD : SERVO_REVERSE);

            } else if (geckoActive) {
                // Gecko spinning: auto run servos forward once geckoPower near full
                if (geckoPower >= 0.60) {
                    setIntakeServos(SERVO_FORWARD);
                } else {
                    setIntakeServos(SERVO_STOP);
                }
                intakeMotor.setPower(0.0);

            } else {
                // No input
                intakeMotor.setPower(0.0);
                setIntakeServos(SERVO_STOP);
            }

            // --- Telemetry ---
            telemetry.addData("Gecko Power", "%.2f", geckoPower);
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Servos", "L: %.2f | R: %.2f",
                    intakeServoLeft.getPosition(), intakeServoRight.getPosition());
            telemetry.addData("Encoders", "L: %d | R: %d | Intake: %d",
                    geckoLeft.getCurrentPosition(),
                    geckoRight.getCurrentPosition(),
                    intakeMotor.getCurrentPosition());
            telemetry.update();
        }

        stopAll();
    }

    private void resetAndRunWithoutEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Mirrors right servo to match physical mounting */
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
    }
}
