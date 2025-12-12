package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class testingVelocityMech {

    // ================= DRIVE MOTORS (VELOCITY) =================
    public DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // ================= MECHANISMS =================
    public DcMotorEx left_gecko, right_gecko;
    public DcMotorEx intake;
    public DcMotorEx conveyer_belt;

    // ================= SERVO =================
    public Servo gateServo;

    // ================= CONSTANTS =================
    public static final double TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_INCHES = 3.78;
    public static final double WHEEL_CIRCUMFERENCE =
            Math.PI * WHEEL_DIAMETER_INCHES;
    public static final double TICKS_PER_INCH =
            TICKS_PER_REV / WHEEL_CIRCUMFERENCE;

    // Max GoBilda 5203 (312 RPM)
    public static final double MAX_TPS =
            (312.0 * TICKS_PER_REV) / 60.0;

    public testingVelocityMech() {}

    // ================= INIT =================
    public void init(HardwareMap hwMap) {

        // -------- Drive --------
        frontLeftMotor  = hwMap.get(DcMotorEx.class, "left_front_drive");
        frontRightMotor = hwMap.get(DcMotorEx.class, "right_front_drive");
        backLeftMotor   = hwMap.get(DcMotorEx.class, "left_back_drive");
        backRightMotor  = hwMap.get(DcMotorEx.class, "right_back_drive");

        // -------- Mechanisms --------
        left_gecko     = hwMap.get(DcMotorEx.class, "left_gecko");
        right_gecko    = hwMap.get(DcMotorEx.class, "right_gecko");
        intake         = hwMap.get(DcMotorEx.class, "intake");
        conveyer_belt  = hwMap.get(DcMotorEx.class, "conveyer_belt");

        conveyer_belt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyer_belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // -------- Servo --------
        gateServo = hwMap.get(Servo.class, "gate_servo");
        gateServo.setPosition(0.0);

        // -------- Directions --------
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // -------- Brake --------
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetDriveEncoders();
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ================= DRIVE VELOCITY =================
    public void setDriveVelocity(double fl, double fr, double bl, double br) {
        frontLeftMotor.setVelocity(fl);
        frontRightMotor.setVelocity(fr);
        backLeftMotor.setVelocity(bl);
        backRightMotor.setVelocity(br);
    }

    public void stopDrive() {
        setDriveVelocity(0, 0, 0, 0);
    }

    // ================= ENCODERS =================
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

    // ================= AUTO: DRIVE =================
    public void driveToPosition(int ticks, double velocityScale) {

        frontLeftMotor.setTargetPosition(ticks);
        frontRightMotor.setTargetPosition(ticks);
        backLeftMotor.setTargetPosition(ticks);
        backRightMotor.setTargetPosition(ticks);

        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        double vel = velocityScale * MAX_TPS;

        setDriveVelocity(vel, vel, vel, vel);
    }

    // ================= AUTO: STRAFE =================
    public void strafe(String direction, int ticks, double velocityScale) {

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

        double vel = velocityScale * MAX_TPS;
        setDriveVelocity(vel, vel, vel, vel);
    }

    // ================= AUTO: TURN =================
    public void turn(double degrees, double velocityScale) {

        double ticksPerDegree = 10.0;
        int turnTicks = (int) (ticksPerDegree * degrees);

        resetDriveEncoders();

        frontLeftMotor.setTargetPosition(-turnTicks);
        backLeftMotor.setTargetPosition(-turnTicks);
        frontRightMotor.setTargetPosition(turnTicks);
        backRightMotor.setTargetPosition(turnTicks);

        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        double vel = velocityScale * MAX_TPS;
        setDriveVelocity(-vel, vel, -vel, vel);
    }

    // ================= STATE =================
    public boolean isDriveBusy() {
        return frontLeftMotor.isBusy()
                || frontRightMotor.isBusy()
                || backLeftMotor.isBusy()
                || backRightMotor.isBusy();
    }
}
