package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
    private int ReverseMultiplier = -1;
    private boolean canSwitch = true;

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

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set directions Questionable for now....
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        //end questionable code....

        rightElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightShoulder.setDirection(DcMotor.Direction.FORWARD);
        leftShoulder.setDirection(DcMotor.Direction.FORWARD);

        leftElbow.setDirection(DcMotor.Direction.FORWARD);
        rightElbow.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        waitForStart();
        while (opModeIsActive()) {
            double power = (gamepad1.left_trigger > gamepad1.right_trigger) ?
                    gamepad1.left_trigger : -gamepad1.right_trigger;

            if ((gamepad1.a) && (canSwitch)) {
                canSwitch = false;
                ReverseMultiplier *= -1;
            } else if (!gamepad1.a) {
                canSwitch = true;
            }

            if (gamepad2.y) {
                leftShoulder.setTargetPosition(900);
                rightShoulder.setTargetPosition(-900);
                leftShoulder.setPower(0.5);
                rightShoulder.setPower(-0.5);
                leftShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad2.b) {
                leftShoulder.setTargetPosition(250);
                rightShoulder.setTargetPosition(-250);
                leftShoulder.setPower(0.5);
                rightShoulder.setPower(-0.5);
                leftShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad2.a) {
                leftShoulder.setTargetPosition(47);
                rightShoulder.setTargetPosition(-5);
                leftShoulder.setPower(0.5);
                rightShoulder.setPower(-0.5);
                leftShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.left_trigger > 0) {
                leftElbow.setPower(-gamepad2.left_trigger);
                rightElbow.setPower(-gamepad2.left_trigger);
            } else if (gamepad2.right_trigger > 0) {
                leftElbow.setPower(gamepad2.right_trigger);
                rightElbow.setPower(gamepad2.right_trigger);
            }

            if (gamepad2.left_bumper) {
                clawWrist.setPosition(0.1);
            } else if (gamepad2.right_bumper) {
                clawWrist.setPosition(0.72);
            }

            if (gamepad2.x){
                leftShoulder.setTargetPosition(975);
                rightShoulder.setTargetPosition(-1017);
                leftShoulder.setPower(1);
                rightShoulder.setPower(-1);
                leftShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (gamepad2.ps){
                    leftElbow.setTargetPosition(0);
                    rightShoulder.setTargetPosition(0);
                    leftShoulder.setPower(1);
                    rightShoulder.setPower(-1);
                    leftShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

            }

            if (gamepad2.dpad_down) {
                clawEat.setPower(-1);
            } else if (gamepad2.dpad_up) {
                clawEat.setPower(1);
            }
            Move_USE_ANALOG_STICK(power, gamepad1.left_stick_x, gamepad1.right_stick_x);
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

            telemetry.addData("Front Left POS", leftFront.getCurrentPosition());
            telemetry.addData("Front Right POS", rightFront.getCurrentPosition());
            telemetry.addData("Back Left POS", leftBack.getCurrentPosition());
            telemetry.addData("Back Right POS", rightBack.getCurrentPosition());
            telemetry.update();
        }
    }

    public void Move_USE_ANALOG_STICK(double axial, double lateral, double yaw) {
        /*
         *       This function uses emulated joystick inputs in
         * the range of [-1.0, 1.0] to move the robot
         * i.e.: axial is the y coordinate for the left stick on the control pad
         *       lateral is the x coordinate for the left stick on the control pad
         *       yaw is the x coordinate for the right stick on the control pad
         * */
        double leftFront_pwr = axial - lateral + yaw;
        double rightFront_pwr = axial + lateral - yaw;
        double leftBack_pwr = axial + lateral + yaw;
        double rightBack_pwr = axial - lateral - yaw;
        leftFront.setPower(ReverseMultiplier * leftFront_pwr/2);
        rightFront.setPower(ReverseMultiplier * rightFront_pwr/2);
        leftBack.setPower(ReverseMultiplier * leftBack_pwr/2);
        rightBack.setPower(ReverseMultiplier * rightBack_pwr/2);
    }
}
