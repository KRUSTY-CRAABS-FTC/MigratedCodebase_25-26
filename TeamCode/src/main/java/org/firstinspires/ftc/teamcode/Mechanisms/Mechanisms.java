package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Mechanisms {

    // --- Drive Motors (GoBilda 5203) ---
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // --- Gecko Motors (REV UltraPlanetary) ---
    public DcMotor left_gecko, right_gecko;

    // --- Intake Motor (REV UltraPlanetary) ---
    public DcMotor intake;

    // --- Servos (GoBilda) ---
    public Servo left_servo, right_servo;

    // --- Constants (tune these for your robot) ---
    private static final double TICKS_PER_REV = 537.7; // GoBilda 5203 motor
    private static final double WHEEL_DIAMETER_INCHES = 3.78; // adjust to your wheels
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;
    public static final double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;

    // --- Constructor ---
    public Mechanisms() {}

    // --- Initialization ---
    public void init(HardwareMap hwMap) {
        // Drive
        frontLeftMotor = hwMap.get(DcMotor.class, "left_front_drive");
        frontRightMotor = hwMap.get(DcMotor.class, "right_front_drive");
        backLeftMotor = hwMap.get(DcMotor.class, "left_back_drive");
        backRightMotor = hwMap.get(DcMotor.class, "right_back_drive");

        // Attachments
        left_gecko = hwMap.get(DcMotor.class, "left_gecko");
        right_gecko = hwMap.get(DcMotor.class, "right_gecko");
        intake = hwMap.get(DcMotor.class, "intake");

        // Servos
        left_servo = hwMap.get(Servo.class, "left_servo");
        right_servo = hwMap.get(Servo.class, "right_servo");

        // Motor directions (adjust if needed)
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Zero power behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset and enable encoders for auto control
        resetDriveEncoders();
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // --- Encoder Utility Methods ---
    public void resetDriveEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setDriveRunMode(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        backRightMotor.setMode(mode);
    }

    // --- Drive To Position ---
    public void driveToPosition(int ticks, double power) {
        frontLeftMotor.setTargetPosition(ticks);
        frontRightMotor.setTargetPosition(ticks);
        backLeftMotor.setTargetPosition(ticks);
        backRightMotor.setTargetPosition(ticks);

        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    // --- Strafe (left, right, forward, backward) ---
    public void strafe(String direction, int ticks, double power) {
        resetDriveEncoders();

        switch (direction.toLowerCase()) {
            case "left":
                frontLeftMotor.setTargetPosition(-ticks);
                frontRightMotor.setTargetPosition(ticks);
                backLeftMotor.setTargetPosition(ticks);
                backRightMotor.setTargetPosition(-ticks);
                break;

            case "right":
                frontLeftMotor.setTargetPosition(ticks);
                frontRightMotor.setTargetPosition(-ticks);
                backLeftMotor.setTargetPosition(-ticks);
                backRightMotor.setTargetPosition(ticks);
                break;

            case "forward":
                frontLeftMotor.setTargetPosition(ticks);
                frontRightMotor.setTargetPosition(ticks);
                backLeftMotor.setTargetPosition(ticks);
                backRightMotor.setTargetPosition(ticks);
                break;

            case "backward":
                frontLeftMotor.setTargetPosition(-ticks);
                frontRightMotor.setTargetPosition(-ticks);
                backLeftMotor.setTargetPosition(-ticks);
                backRightMotor.setTargetPosition(-ticks);
                break;

            default:
                // Invalid input, stop robot
                stopDrive();
                return;
        }

        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    // --- Turn using encoders (approximate) ---
    public void turn(double degrees, double power) {
        // Tune this constant for your robot — how many ticks = 360° turn
        double ticksPerDegree = 10.0; // start with 10, adjust by testing

        int turnTicks = (int) (ticksPerDegree * degrees);

        resetDriveEncoders();

        // Left side goes backward, right side goes forward
        frontLeftMotor.setTargetPosition(-turnTicks);
        backLeftMotor.setTargetPosition(-turnTicks);
        frontRightMotor.setTargetPosition(turnTicks);
        backRightMotor.setTargetPosition(turnTicks);

        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    // --- Status Helpers ---
    public boolean isDriveBusy() {
        return frontLeftMotor.isBusy() || frontRightMotor.isBusy()
                || backLeftMotor.isBusy() || backRightMotor.isBusy();
    }

    public void stopDrive() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
