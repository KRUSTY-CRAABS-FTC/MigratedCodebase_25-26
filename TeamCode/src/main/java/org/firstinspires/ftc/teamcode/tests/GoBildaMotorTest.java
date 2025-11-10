package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "GoBilda Motor Test", group = "Test")
public class GoBildaMotorTest extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBack = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Reverse one side if necessary (typically right side)
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders and set mode
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("✅ GoBilda Motor Test Ready");
        telemetry.addLine("Press ▶ to start and use gamepad buttons to test:");
        telemetry.addLine("A = Left Front");
        telemetry.addLine("B = Right Front");
        telemetry.addLine("X = Left Back");
        telemetry.addLine("Y = Right Back");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Default all motors to stop
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            // Control one motor at a time
            if (gamepad1.a) leftFront.setPower(1);
            if (gamepad1.b) rightFront.setPower(1);
            if (gamepad1.x) leftBack.setPower(1);
            if (gamepad1.y) rightBack.setPower(1);

            // Telemetry data
            telemetry.addData("Left Front", "Power: %.2f | Pos: %d", leftFront.getPower(), leftFront.getCurrentPosition());
            telemetry.addData("Left Back", "Power: %.2f | Pos: %d", leftBack.getPower(), leftBack.getCurrentPosition());
            telemetry.addData("Right Front", "Power: %.2f | Pos: %d", rightFront.getPower(), rightFront.getCurrentPosition());
            telemetry.addData("Right Back", "Power: %.2f | Pos: %d", rightBack.getPower(), rightBack.getCurrentPosition());
            telemetry.addLine("Press A/B/X/Y to test motors individually");
            telemetry.update();
        }
    }
}
