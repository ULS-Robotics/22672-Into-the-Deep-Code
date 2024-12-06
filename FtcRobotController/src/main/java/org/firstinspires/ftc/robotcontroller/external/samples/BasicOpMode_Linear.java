package org.firstinspires.ftc.robotcontroller.external.samples;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="OpModeCode2024", group="Linear OpMode")

public class BasicOpMode_Linear extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private DcMotor rightShoulder;      //all arms

    private DcMotor rightElbow;

    private DcMotor leftShoulder;

    private DcMotor leftElbow;

    private CRServo clawEat;

    private Servo clawWrist;
    private int reverse_multiplier = -1;
    private boolean canSwitch = true;

    private int arm_position_index = 0;

    private boolean FirstClawTurn = true;

    @Override
    public void runOpMode() {

        // declare servos
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        clawEat = hardwareMap.get(CRServo.class, "claw_eat");
        clawWrist = hardwareMap.get(Servo.class, "claw_wrist");
        rightShoulder = hardwareMap.get(DcMotor.class, "right_shoulder");
        rightElbow = hardwareMap.get(DcMotor.class, "right_elbow");
        leftShoulder = hardwareMap.get(DcMotor.class, "left_shoulder");
        leftElbow = hardwareMap.get(DcMotor.class, "left_elbow");

        // set directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        rightElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightShoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightElbow.setDirection(DcMotorSimple.Direction.FORWARD);
        leftShoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        leftElbow.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        waitForStart();
        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFront_pwr = axial - lateral + yaw;
            double rightFront_pwr = axial + lateral - yaw;
            double leftBack_pwr = axial + lateral + yaw;
            double rightBack_pwr = axial - lateral - yaw;

            if ((gamepad1.a) && (canSwitch)) {
                canSwitch = false;
                reverse_multiplier *= -1;
            } else if (!gamepad1.a) {
                canSwitch = true;
            }

            if (gamepad2.y){
                leftShoulder.setTargetPosition(1273);
                rightShoulder.setTargetPosition(-1294);
                leftShoulder.setPower(0.5);
                rightShoulder.setPower(-0.5);
                leftShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }else if (gamepad2.b){
                leftShoulder.setTargetPosition(300);
                rightShoulder.setTargetPosition(-323);
                leftShoulder.setPower(0.5);
                rightShoulder.setPower(-0.5);
                leftShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }else if (gamepad2.a){
                leftShoulder.setTargetPosition(0);
                rightShoulder.setTargetPosition(0);
                leftShoulder.setPower(0.5);
                rightShoulder.setPower(-0.5);
                leftShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.right_trigger < 0) {
                leftElbow.setPower(gamepad2.right_trigger);
                rightElbow.setPower(-gamepad2.right_trigger);
            }else if (gamepad2.right_trigger > 0){
                leftElbow.setPower(gamepad2.right_trigger);
                rightElbow.setPower(-gamepad2.right_trigger);
            }

            if (gamepad2.left_bumper) {
                clawWrist.setPosition(0.8);
            } else if (gamepad2.right_bumper) {
                clawWrist.setPosition(-0.5);
            }

            if (gamepad2.dpad_down){
                clawEat.setPower(- 1);
            }else if (gamepad2.dpad_up){
                clawEat.setPower(1);
            }

            leftFront.setPower(reverse_multiplier * leftFront_pwr);
            rightFront.setPower(reverse_multiplier * rightFront_pwr);
            leftBack.setPower(reverse_multiplier * leftBack_pwr);
            rightBack.setPower(reverse_multiplier * rightBack_pwr);

            telemetry.addData("Left Shoulder Power: ", leftShoulder.getPower());
            telemetry.addData("Right Shoulder Power: ", rightShoulder.getPower());
            telemetry.addData("Left Elbow Power: ", leftElbow.getPower());
            telemetry.addData("Right Elbow Power: ", rightElbow.getPower());

            telemetry.addData("", "\n");
            telemetry.addData("Left Shoulder Position", leftShoulder.getCurrentPosition());
            telemetry.addData("Right Shoulder Position", rightShoulder.getCurrentPosition());
            telemetry.addData("Left Elbow Position", leftElbow.getCurrentPosition());
            telemetry.addData("Right Elbow Position", rightElbow.getCurrentPosition());
            telemetry.addData("", "\n");

            telemetry.addData("gamepad2.right_stick_y: ", gamepad2.right_stick_y);
            telemetry.addData("gamepad2.left_stick_y: ", gamepad2.left_stick_y);
            telemetry.addData("gamepad2.left_stick_y: ", gamepad2.left_stick_y);
            telemetry.addData("gamepad2.right_stick_y: ", gamepad2.right_stick_y);
            telemetry.addData("", "\n");

            telemetry.addData("Front Left Power", leftFront_pwr * reverse_multiplier);
            telemetry.addData("Front Right Power", rightFront_pwr* reverse_multiplier);
            telemetry.addData("Back Left Power", leftBack_pwr * reverse_multiplier);
            telemetry.addData("Back Right Power", rightBack_pwr * reverse_multiplier);

            telemetry.update();
        }
    }
}