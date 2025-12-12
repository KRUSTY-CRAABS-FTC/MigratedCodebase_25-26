package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.testingVelocityMech;

@TeleOp(name = "TESTING VELOCITY DRIVETRAIN", group = "TeleOp")
public class testingVelocityDrive extends LinearOpMode {

    private testingVelocityMech robot = new testingVelocityMech();

    // ================= VELOCITY CONSTANTS =================
    public static final double MAX_TPS =
            (312.0 * 537.7) / 60.0;   // GoBilda 5203
    public static final double DRIVE_SPEED = 1.0;

    // ================= MECHANISMS =================
    private static final double MAX_INTAKE_POWER = 0.50;
    private static final double TRIGGER_DEADZONE = 0.05;

    private IMU imu;

    // Gecko toggle
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private double geckoPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

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

            // ================= FIELD CENTRIC DRIVE (VELOCITY) =================
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            if (gamepad1.options) imu.resetYaw();

            double botHeading =
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denom = Math.max(
                    Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1
            );

            double fl = (rotY + rotX + rx) / denom * DRIVE_SPEED;
            double bl = (rotY - rotX + rx) / denom * DRIVE_SPEED;
            double fr = (rotY - rotX - rx) / denom * DRIVE_SPEED;
            double br = (rotY + rotX - rx) / denom * DRIVE_SPEED;

            robot.frontLeftMotor.setVelocity(fl * MAX_TPS);
            robot.backLeftMotor.setVelocity(bl * MAX_TPS);
            robot.frontRightMotor.setVelocity(fr * MAX_TPS);
            robot.backRightMotor.setVelocity(br * MAX_TPS);

            // ================= GECKOS (POWER) =================
            if (gamepad2.a && !lastA) geckoPower = 0.42;
            if (gamepad2.b && !lastB) geckoPower = 0.44;
            if (gamepad2.y && !lastY) geckoPower = 0.53;
            if (gamepad2.x && !lastX) geckoPower = -0.50;
            if (gamepad2.right_stick_button) geckoPower = 0;

            lastA = gamepad2.a;
            lastB = gamepad2.b;
            lastX = gamepad2.x;
            lastY = gamepad2.y;

            robot.left_gecko.setPower(-geckoPower);
            robot.right_gecko.setPower(geckoPower);

            // ================= INTAKE (POWER) =================
            double rt = gamepad2.right_trigger;
            double lt = gamepad2.left_trigger;

            double intakePower = 0;
            if (rt > TRIGGER_DEADZONE || lt > TRIGGER_DEADZONE)
                intakePower = Range.clip(
                        rt - lt, -MAX_INTAKE_POWER, MAX_INTAKE_POWER
                );

            robot.intake.setPower(-intakePower);

            // ================= CONVEYOR (POWER) =================
            double conveyorPower = 0;
            if (gamepad2.right_bumper) conveyorPower = -0.7;
            else if (gamepad2.left_bumper) conveyorPower = 0.7;

            robot.conveyer_belt.setPower(conveyorPower);

            // ================= TELEMETRY =================
            telemetry.addLine("=== VELOCITY DRIVE ONLY ===");
            telemetry.addData("Heading (deg)", "%.1f",
                    Math.toDegrees(botHeading));
            telemetry.addData("FL / FR TPS", "%.0f / %.0f",
                    fl * MAX_TPS, fr * MAX_TPS);
            telemetry.addData("BL / BR TPS", "%.0f / %.0f",
                    bl * MAX_TPS, br * MAX_TPS);

            telemetry.update();
        }

        stopAllMotors();
    }

    private void stopAllMotors() {
        robot.frontLeftMotor.setVelocity(0);
        robot.frontRightMotor.setVelocity(0);
        robot.backLeftMotor.setVelocity(0);
        robot.backRightMotor.setVelocity(0);

        robot.left_gecko.setPower(0);
        robot.right_gecko.setPower(0);
        robot.intake.setPower(0);
        robot.conveyer_belt.setPower(0);
    }
}
