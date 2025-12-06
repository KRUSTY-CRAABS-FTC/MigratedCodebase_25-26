package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.updatedMech;

@TeleOp(name = "UPDATED FIELD CENTRIC TELE OP", group = "TeleOp")
public class UpdatedFieldCentricMecanumTeleOp extends LinearOpMode {

    private updatedMech robot = new updatedMech();

    private static final double MAX_INTAKE_POWER = 0.50;   // intake max = 0.5
    private static final double TRIGGER_DEADZONE = 0.05;

    private IMU imu;

    // For gecko toggle system
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private double geckoPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

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

            double fl = (rotY + rotX + rx) / denominator;
            double bl = (rotY - rotX + rx) / denominator;
            double fr = (rotY - rotX - rx) / denominator;
            double br = (rotY + rotX - rx) / denominator;

            robot.frontLeftMotor.setPower(fl);
            robot.backLeftMotor.setPower(bl);
            robot.frontRightMotor.setPower(fr);
            robot.backRightMotor.setPower(br);

            // ---------------- GECKOS (TOGGLE BUTTONS) ----------------
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

            // ---------------- INTAKE (max 0.5) ----------------
            double rt = gamepad2.right_trigger;
            double lt = gamepad2.left_trigger;

            double intakePower = 0.0;
            boolean triggerActive = (rt > TRIGGER_DEADZONE) || (lt > TRIGGER_DEADZONE);

            if (triggerActive) {
                intakePower = Range.clip(rt - lt, -MAX_INTAKE_POWER, MAX_INTAKE_POWER);
                robot.intake.setPower(-intakePower);
            } else {
                robot.intake.setPower(0.0);
            }

            // ---------------- CONVEYOR (LB + RB, capped at 0.5) ----------------
            double conveyorPower = 0;

            if (gamepad2.right_bumper) {
                conveyorPower = -0.7;   // forward
            } else if (gamepad2.left_bumper) {
                conveyorPower = 0.7;    // reverse
            } else {
                conveyorPower = 0;
            }

            robot.conveyer_belt.setPower(conveyorPower);

            // ---------------- TELEMETRY ----------------
            telemetry.addLine("=== FIELD-CENTRIC DRIVE ===");
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(botHeading));

            telemetry.addLine("=== MECHANISMS ===");
            telemetry.addData("Gecko Power", "%.2f", geckoPower);
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Conveyor Power", "%.2f", conveyorPower);

            telemetry.update();
        }

        stopAllMotors();
    }

    private void stopAllMotors() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        robot.left_gecko.setPower(0);
        robot.right_gecko.setPower(0);
        robot.intake.setPower(0);

        robot.conveyer_belt.setPower(0);
    }
}
