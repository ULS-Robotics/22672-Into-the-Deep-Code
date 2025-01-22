package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Objects;

@TeleOp(name="AUTONOMOUS *RED*", group="Linear OpMode")

public class Autonomous_Red extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private DcMotor rightShoulder;

    private DcMotor rightElbow;

    private DcMotor leftShoulder;

    private DcMotor leftElbow;

    private CRServo clawEat;

    private Servo clawWrist;
    @Override
    public void runOpMode() {

        // declare motors and servos
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
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)

        waitForStart();

        while (opModeIsActive()) {
            MoveBase_USE_POSITION(1042,1089, 1072, 1048);
            Halt();
            MoveBase_USE_POSITION(-990, 1179,-1136, 855);
            Halt();
            MoveBase_USE_POSITION(1478, 1548, 1541, 1427);
            Halt();
            MoveBase_USE_POSITION(1335, -936, 961, -1159);
            Halt();
            MoveBase_USE_POSITION(2554, 2534, 2503, 2513);
            Halt();
            MoveBase_USE_POSITION(1165, -886, 870, -982);
            Halt();
            MoveShoulder_USE_POSITION("M");
            Halt();
            //MoveBase_USE_POSITION(662, 555, 426, 377);
            //Halt();
            break;
        }
    }
    // packaged functions to move things
    public void MoveElbow_USE_POSITION(int LEFT_Elbow_Position, int RIGHT_Elbow_Position){
        /*This function uses ENCODERS to move arms to a given POSITION
         * i.e.: Set a given location measured through experiment and
         *       put them into this function
         * */
        leftElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftElbow.setTargetPosition(LEFT_Elbow_Position);
        rightElbow.setTargetPosition(RIGHT_Elbow_Position);

        leftElbow.setPower(0.5);
        rightElbow.setPower(-0.5);

        leftElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void MoveShoulder_USE_POSITION(String Position_Code){


        /*This will move the arm to a given position
         *               *WE SHALL USE THIS MORE OFTEN*
         * Position Code: H, M, R
         *   H = high
         *   M = medium (horizontal)
         *   R = reset to 0
         *
         * IMPORTANT NOTE: THIS FUNCTION WILL ONLY MOVE THE SHOULDER BUT **NOT**
         *                 THE ELBOW
         * */

        if (Objects.equals(Position_Code, "H")){
            leftShoulder.setTargetPosition(1273);
            rightShoulder.setTargetPosition(-1294);
            leftShoulder.setPower(0.5);
            rightShoulder.setPower(-0.5);
            leftShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }else if (Objects.equals(Position_Code, "M")){
            leftShoulder.setTargetPosition(300);
            rightShoulder.setTargetPosition(-323);
            leftShoulder.setPower(0.5);
            rightShoulder.setPower(-0.5);
            leftShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }else if (Objects.equals(Position_Code, "R")){
            leftShoulder.setTargetPosition(0);
            rightShoulder.setTargetPosition(0);
            leftShoulder.setPower(0.5);
            rightShoulder.setPower(-0.5);
            leftShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void MoveBase_USE_POSITION(int ... positions){
        /*It takes a list input:
         * INDEX     |       Value
         * [0]               Left Front Position
         * [1]               Right Front Position
         * [2]               Left Back Position
         * [3]               Right Back Position
         * */

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(positions[0]);
        rightFront.setTargetPosition(positions[1]);
        leftBack.setTargetPosition(positions[2]);
        rightBack.setTargetPosition(positions[3]);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
    }

    public void UseClaw(double turn_position, boolean spit){

        /*This function utilizes the claw by specifying the turning location for the
         * claw wrist and uses a boolean expression to manage whether or not to spit the block */

        clawWrist.setPosition(turn_position);
        if (spit){
            clawEat.setPower(1);
        }else{
            clawEat.setPower(-1);
        }
    }

    public void Halt(){
        while ((leftFront.isBusy()) | (rightBack.isBusy()) | (leftBack.isBusy()) | (rightFront.isBusy())
        | (leftShoulder.isBusy()) | (rightShoulder.isBusy()) | (leftElbow.isBusy()) | (rightElbow.isBusy())){

        }
    }

}