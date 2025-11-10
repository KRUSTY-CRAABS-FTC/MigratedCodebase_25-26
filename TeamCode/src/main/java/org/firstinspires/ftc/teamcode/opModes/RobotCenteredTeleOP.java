package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ROBOT CENTERED TELE-OP MAIN", group = "TeleOp")
public class RobotCenteredTeleOP extends LinearOpMode {

    // --- Drive Motors ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- Mechanism Motors ---
    private DcMotor geckoLeft, geckoRight, intakeMotor;

    // --- Servos ---
    private Servo intakeServoLeft, intakeServoRight;

    // --- Constants ---
    private static final double SERVO_STOP = 0.5;
    private static final double SERVO_FORWARD = 1.0;
    private static final double SERVO_REVERSE = 0.0;
    private static final double MAX_INTAKE_POWER = 0.45;
    private static final double TRIGGER_DEADZONE = 0.05;
    private static final double DRIVE_SLOW_FACTOR = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        // -------------------------------
        // HARDWARE MAP
        // -------------------------------
        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");

        geckoLeft = hardwareMap.get(DcMotor.class, "left_gecko");
        geckoRight = hardwareMap.get(DcMotor.class, "right_gecko");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        intakeServoLeft = hardwareMap.get(Servo.class, "left_servo");
        intakeServoRight = hardwareMap.get(Servo.class, "right_servo");

        // -------------------------------
        // DRIVE CONFIG
        // -------------------------------
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setBrake(frontLeft, frontRight, backLeft, backRight);
        setRunWithoutEncoder(frontLeft, frontRight, backLeft, backRight);

        // -------------------------------
        // FUNCTION CONFIG
        // -------------------------------
        geckoLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        geckoRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setBrake(geckoLeft, geckoRight, intakeMotor);
        resetAndRunWithoutEncoder(geckoLeft, geckoRight, intakeMotor);

        setIntakeServos(SERVO_STOP);

        telemetry.addLine("All systems initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ---------------------------------------
            // GAMEPAD 1: DRIVE (Robot-centric)
            // ---------------------------------------
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double fl = drive + strafe + rotate;
            double fr = drive - strafe - rotate;
            double bl = drive - strafe + rotate;
            double br = drive + strafe - rotate;

            double max = Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            fl *= DRIVE_SLOW_FACTOR;
            fr *= DRIVE_SLOW_FACTOR;
            bl *= DRIVE_SLOW_FACTOR;
            br *= DRIVE_SLOW_FACTOR;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // ---------------------------------------
            // GAMEPAD 2: FUNCTION SYSTEM
            // ---------------------------------------
            double geckoPower = 0.0;
            if (gamepad2.a) geckoPower = 0.62;
            else if (gamepad2.b) geckoPower = 0.65;
            else if (gamepad2.y) geckoPower = 0.69;

            boolean geckoActive = geckoPower > 0.0;

            if (geckoActive) {
                // --- Unified Gecko + Intake + Servos ---
                geckoLeft.setPower(-geckoPower);
                geckoRight.setPower(-geckoPower);
                intakeMotor.setPower(MAX_INTAKE_POWER);
                setIntakeServos(SERVO_FORWARD);

            } else {
                // --- Normal Manual Intake Control ---
                geckoLeft.setPower(0.0);
                geckoRight.setPower(0.0);

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

                } else {
                    // No input
                    intakeMotor.setPower(0.0);
                    setIntakeServos(SERVO_STOP);
                }
            }

            // ---------------------------------------
            // TELEMETRY
            // ---------------------------------------
            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("FL", "%.2f", fl);
            telemetry.addData("FR", "%.2f", fr);
            telemetry.addData("BL", "%.2f", bl);
            telemetry.addData("BR", "%.2f", br);

            telemetry.addLine("=== FUNCTION ===");
            telemetry.addData("Gecko Power", "%.2f", geckoPower);
            telemetry.addData("Intake Power", "%.2f", intakeMotor.getPower());
            telemetry.addData("Servos", "L: %.2f | R: %.2f",
                    intakeServoLeft.getPosition(), intakeServoRight.getPosition());
            telemetry.update();
        }

        stopAll();
    }

    // --- Helper Methods ---
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
