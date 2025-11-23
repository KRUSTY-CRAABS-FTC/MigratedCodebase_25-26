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

    private static final double MAX_INTAKE_POWER = 0.55;
    private static final double TRIGGER_DEADZONE = 0.05;

    private IMU imu;

    private boolean gateOpen = false;
    private boolean lastLeftStickPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.gateServo.setPosition(0.0);
        gateOpen = false;

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

            boolean leftStickPressed = gamepad2.left_stick_button;
            if (leftStickPressed && !lastLeftStickPressed) {
                gateOpen = !gateOpen;
                robot.gateServo.setPosition(gateOpen ? 1.0 : 0.0);
            }
            lastLeftStickPressed = leftStickPressed;

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

            double geckoPower = 0.0;

            if (gamepad2.a) geckoPower = 0.44;    // normal forward
            else if (gamepad2.b) geckoPower = 0.46; // normal forward
            else if (gamepad2.y) geckoPower = 0.55; // normal forward
            else if (gamepad2.x) geckoPower = -0.50; // X button: backward spin

            robot.left_gecko.setPower(-geckoPower);
            robot.right_gecko.setPower(geckoPower);

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

            // AUTO CONVEYOR + MANUAL OVERRIDE, SAME DIRECTION AS RIGHT BUMPER
            boolean intakeActive = triggerActive;
            double conveyorPower;

            if (gamepad2.right_bumper) {
                conveyorPower = -1; // main direction
            } else if (gamepad2.left_bumper) {
                conveyorPower = 1;  // reverse
            } else if (intakeActive) {
                conveyorPower = -0.5; // auto same direction as right bumper
            } else {
                conveyorPower = 0;
            }

            robot.conveyer_belt.setPower(conveyorPower);

            telemetry.addLine("=== FIELD-CENTRIC DRIVE ===");
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(botHeading));

            telemetry.addLine("=== MECHANISMS ===");
            telemetry.addData("Gecko Power", "%.2f", geckoPower);
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Conveyor Power", "%.2f", conveyorPower);

            telemetry.addLine("=== GATE SERVO ===");
            telemetry.addData("Gate State", gateOpen ? "OPEN (1.0)" : "CLOSED (0.0)");
            telemetry.addData("Gate Servo Position", robot.gateServo.getPosition());

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
        robot.gateServo.setPosition(0);
    }
}
