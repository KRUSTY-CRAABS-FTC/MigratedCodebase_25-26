package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.velocityBasedMech;

@TeleOp(name = "UPDATED FIELD CENTRIC TELE OP (VELOCITY)", group = "TeleOp")
public class velocityBasedFieldCentric extends LinearOpMode {

    private velocityBasedMech robot = new velocityBasedMech();

    // ======================================================
    // ===============  TUNABLE VELOCITY CONSTANTS ==========
    // ======================================================

    // Max ticks/sec for GoBilda 5203 (312RPM)
    public static final double MAX_TPS = (312.0 * 537.7) / 60.0;

    // -------- Drive Speed Scale --------
    public static final double DRIVE_SPEED = 1.00;   // <---- MAIN DRIVE VELOCITY SCALE

    // -------- Gecko Shooter Speeds --------
    public static final double GECKO_A_SPEED = 0.44;   // gamepad2 A
    public static final double GECKO_B_SPEED = 0.46;   // gamepad2 B
    public static final double GECKO_Y_SPEED = 0.55;   // gamepad2 Y
    public static final double GECKO_REV_SPEED = -0.50; // gamepad2 X

    // -------- Intake --------
    public static final double INTAKE_MAX_SPEED = 0.55;
    private static final double TRIGGER_DEADZONE = 0.05;

    // -------- Conveyor --------
    public static final double CONVEYOR_SPEED = 0.30;

    // ======================================================
    private IMU imu;
    private boolean gateOpen = false;
    private boolean lastLeftStickPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.gateServo.setPosition(0.0);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        ));
        imu.resetYaw();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ---------------- GATE TOGGLE ----------------
            boolean leftStickPressed = gamepad2.left_stick_button;
            if (leftStickPressed && !lastLeftStickPressed) {
                gateOpen = !gateOpen;
                robot.gateServo.setPosition(gateOpen ? 0.06 : 0.0);
            }
            lastLeftStickPressed = leftStickPressed;

            // ---------------- FIELD CENTRIC DRIVE ----------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            if (gamepad1.options) imu.resetYaw();
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double fl = (rotY + rotX + rx) / denominator * DRIVE_SPEED;
            double bl = (rotY - rotX + rx) / denominator * DRIVE_SPEED;
            double fr = (rotY - rotX - rx) / denominator * DRIVE_SPEED;
            double br = (rotY + rotX - rx) / denominator * DRIVE_SPEED;

            robot.frontLeftMotor.setVelocity(fl * MAX_TPS);
            robot.backLeftMotor.setVelocity(bl * MAX_TPS);
            robot.frontRightMotor.setVelocity(fr * MAX_TPS);
            robot.backRightMotor.setVelocity(br * MAX_TPS);

            // ---------------- GECKO (Velocity) ----------------
            double geckoPower = 0;

            if (gamepad2.a) geckoPower = GECKO_A_SPEED;
            else if (gamepad2.b) geckoPower = GECKO_B_SPEED;
            else if (gamepad2.y) geckoPower = GECKO_Y_SPEED;
            else if (gamepad2.x) geckoPower = GECKO_REV_SPEED;

            double geckoVelocity = geckoPower * MAX_TPS;

            robot.left_gecko.setVelocity(-geckoVelocity);
            robot.right_gecko.setVelocity(geckoVelocity);

            // ---------------- INTAKE (Velocity) ----------------
            double rt = gamepad2.right_trigger;
            double lt = gamepad2.left_trigger;

            double intakePower = 0;
            if (rt > TRIGGER_DEADZONE || lt > TRIGGER_DEADZONE)
                intakePower = Range.clip(rt - lt, -INTAKE_MAX_SPEED, INTAKE_MAX_SPEED);

            robot.intake.setVelocity(-intakePower * MAX_TPS);

            // ---------------- CONVEYOR (Velocity) ----------------
            double conveyorPower = 0;

            if (gamepad2.right_bumper) conveyorPower = -CONVEYOR_SPEED;
            else if (gamepad2.left_bumper) conveyorPower = CONVEYOR_SPEED;
            else if (intakePower != 0) conveyorPower = -CONVEYOR_SPEED;

            robot.conveyer_belt.setVelocity(conveyorPower * MAX_TPS);

            // ---------------- TELEMETRY ----------------
            telemetry.addLine("=== VELOCITY CONTROL ===");
            telemetry.addData("Drive Speed Scale", DRIVE_SPEED);
            telemetry.addData("Gecko Vel", geckoVelocity);
            telemetry.addData("Intake Vel", intakePower * MAX_TPS);
            telemetry.addData("Conveyor Vel", conveyorPower * MAX_TPS);
            telemetry.update();
        }

        stopAllMotors();
    }

    private void stopAllMotors() {
        robot.frontLeftMotor.setVelocity(0);
        robot.frontRightMotor.setVelocity(0);
        robot.backLeftMotor.setVelocity(0);
        robot.backRightMotor.setVelocity(0);
        robot.left_gecko.setVelocity(0);
        robot.right_gecko.setVelocity(0);
        robot.intake.setVelocity(0);
        robot.conveyer_belt.setVelocity(0);
        robot.gateServo.setPosition(0);
    }
}
