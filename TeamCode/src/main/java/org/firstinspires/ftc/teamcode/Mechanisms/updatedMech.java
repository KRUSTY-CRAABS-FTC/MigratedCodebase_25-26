package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class updatedMech {

    // --- Drive Motors (GoBilda 5203) ---
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // --- Gecko Motors (REV UltraPlanetary) ---
    public DcMotor left_gecko, right_gecko;

    // --- Intake Motor ---
    public DcMotor intake;

    // --- Conveyor Belt Motor ---
    public DcMotor conveyer_belt;

    // --- Gate Servo ---
    public Servo gateServo;

    // --- Constants ---
    private static final double TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;
    public static final double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;

    public updatedMech() {}

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

        // Conveyor
        conveyer_belt = hwMap.get(DcMotor.class, "conveyer_belt");
        conveyer_belt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyer_belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        conveyer_belt.setDirection(DcMotor.Direction.FORWARD);

        // Servo
        gateServo = hwMap.get(Servo.class, "gate_servo");
        gateServo.setPosition(0.0);

        // Drivetrain directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Zero power behaviors
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // FULL reset of drive encoders
        resetDriveEncoders();
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

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
                stopDrive();
                return;
        }

        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    public void turn(double degrees, double power) {
        double ticksPerDegree = 10.0;
        int turnTicks = (int) (ticksPerDegree * degrees);

        resetDriveEncoders();

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
