package org.firstinspires.ftc.robotcontroller.external.samples;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="OpModeCode2024", group="Linear OpMode")

public class Linear_Auto extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    //private DcMotor leftArm;
    //private DcMotor rightArm;

    private Servo clawEat;

    private Servo clawWrist;
    private AprilTagProcessor aprilTag;
    private int reverse_multiplier = -1;
    private final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;
    private boolean canSwitch = true;

    @Override
    public void runOpMode() {

        // declare servos
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        //leftArm = hardwareMap.get(DcMotor.class, "left_arm");
        //rightArm = hardwareMap.get(DcMotor.class, "right_arm");
        clawEat = hardwareMap.get(Servo.class, "claw_eat");
        clawWrist = hardwareMap.get(Servo.class, "servo_wrist");

        // set directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        waitForStart();
        initAprilTag();
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

            /* things on the to do list
            if (gamepad1.right_bumper){
                clawWrist.setPosition(0.5);
                clawWrist.setPower(-0.5);
            }*/

            leftFront.setPower(reverse_multiplier * leftFront_pwr);
            rightFront.setPower(reverse_multiplier * rightFront_pwr);
            leftBack.setPower(reverse_multiplier * leftBack_pwr);
            rightBack.setPower(reverse_multiplier * rightBack_pwr);
            AprilTagDetections();
            telemetry.update();

        }
    }
    public void initAprilTag(){
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Eyes"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    @SuppressLint("DefaultLocale")
    public void AprilTagDetections(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                //telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }

}