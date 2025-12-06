package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class velocityBasedMech {

    // MUST be DcMotorEx for velocity control
    public DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DcMotorEx left_gecko, right_gecko;
    public DcMotorEx intake;
    public DcMotorEx conveyer_belt;

    public Servo gateServo;

    public static final double TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_INCHES = 3.78;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;
    public static final double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;

    public velocityBasedMech() {}

    public void init(HardwareMap hwMap) {

        // ----------------------
        // DRIVE MOTORS (DcMotorEx)
        // ----------------------
        frontLeftMotor  = hwMap.get(DcMotorEx.class, "left_front_drive");
        frontRightMotor = hwMap.get(DcMotorEx.class, "right_front_drive");
        backLeftMotor   = hwMap.get(DcMotorEx.class, "left_back_drive");
        backRightMotor  = hwMap.get(DcMotorEx.class, "right_back_drive");

        // ----------------------
        // MECHANISMS (DcMotorEx)
        // ----------------------
        left_gecko   = hwMap.get(DcMotorEx.class, "left_gecko");
        right_gecko  = hwMap.get(DcMotorEx.class, "right_gecko");
        intake       = hwMap.get(DcMotorEx.class, "intake");
        conveyer_belt = hwMap.get(DcMotorEx.class, "conveyer_belt");

        conveyer_belt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyer_belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        conveyer_belt.setDirection(DcMotor.Direction.FORWARD);

        // Servo
        gateServo = hwMap.get(Servo.class, "gate_servo");
        gateServo.setPosition(0.0);

        // Directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    // ------- Now you can use setVelocity everywhere ---------

    public void setDriveVelocity(double fl, double fr, double bl, double br) {
        frontLeftMotor.setVelocity(fl);
        frontRightMotor.setVelocity(fr);
        backLeftMotor.setVelocity(bl);
        backRightMotor.setVelocity(br);
    }

    public void stopDrive() {
        frontLeftMotor.setVelocity(0);
        frontRightMotor.setVelocity(0);
        backLeftMotor.setVelocity(0);
        backRightMotor.setVelocity(0);
    }
}
