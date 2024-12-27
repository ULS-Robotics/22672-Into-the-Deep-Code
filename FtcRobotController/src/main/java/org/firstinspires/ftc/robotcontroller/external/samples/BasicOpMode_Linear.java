package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;
import java.util.ArrayList;
import java.util.List;

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

    private int RuntimeCounter = 0;

    private List<Double> RegisteredPower = new ArrayList<>();

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

        rightShoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightElbow.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        leftElbow.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        waitForStart();
        while (opModeIsActive()) {

            double leftInputPower = gamepad1.left_trigger;
            double rightInputPower = gamepad1.right_trigger;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            MoveBase_ANALOG(lateral, yaw, reverse_multiplier);
            if (RuntimeCounter > 3) {
                MoveBase(RegisteredPower, reverse_multiplier);
                RegisteredPower.clear();
                RuntimeCounter = 0;
            }else{
                double power = (leftInputPower > rightInputPower) ? leftInputPower : -rightInputPower;
                RegisteredPower.add(power);
            }

            if ((gamepad1.a) && (canSwitch)) {
                canSwitch = false;
                reverse_multiplier *= -1;
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
                leftShoulder.setTargetPosition(242);
                rightShoulder.setTargetPosition(-249);
                leftShoulder.setPower(0.5);
                rightShoulder.setPower(-0.5);
                leftShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad2.a) {
                leftShoulder.setTargetPosition(0);
                rightShoulder.setTargetPosition(0);
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
                clawWrist.setPosition(1);
            } else if (gamepad2.right_bumper) {
                clawWrist.setPosition(-0.1);
            }

            if (gamepad2.dpad_down) {
                clawEat.setPower(-1);
            } else if (gamepad2.dpad_up) {
                clawEat.setPower(1);
            }

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
            RuntimeCounter += 1;
        }
    }

    public void MoveBase(List<Double> PowerList, int ReverseMultiplier) {
        /*This function uses an analog input from the controller and uses a mathematical function
         * to smoothen the input and maybe amplify it a little but we can always change it though*/
        double POWER = ReverseMultiplier * MovingAverage(PowerList);
        leftFront.setPower(POWER);
        rightFront.setPower(POWER);
        leftBack.setPower(POWER);
        rightBack.setPower(POWER);
    }

    public void MoveBase_ANALOG(double lateral, double yaw, int ReverseMultiplier) {
        /*
         *       This function uses emulated joystick inputs in
         * the range of [-1.0, 1.0] to move the robot
         * i.e.: axial is the y coordinate for the left stick on the control pad
         *       lateral is the x coordinate for the left stick on the control pad
         *       yaw is the x coordinate for the right stick on the control pad
         * */
        double leftFront_pwr = -lateral + yaw;
        double rightFront_pwr = lateral - yaw;
        double leftBack_pwr = lateral + yaw;
        double rightBack_pwr = -lateral - yaw;
        leftFront.setPower(ReverseMultiplier * leftFront_pwr);
        rightFront.setPower(ReverseMultiplier * rightFront_pwr);
        leftBack.setPower(ReverseMultiplier * leftBack_pwr);
        rightBack.setPower(ReverseMultiplier * rightBack_pwr);
    }


    public double MovingAverage(List<Double> PowerArray) {
        double sum = 0;
        int length = PowerArray.size();
        for (int i = 0 ; i < length ; i++){
            sum += PowerArray.get(i);
        }
        return sum / length;
    }
}