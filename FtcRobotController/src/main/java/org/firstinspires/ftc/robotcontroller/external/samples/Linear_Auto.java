package org.firstinspires.ftc.robotcontroller.external.samples;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Autonomous1", group="Concept")

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
            /*
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFront_pwr = axial - lateral + yaw;
            double rightFront_pwr = axial + lateral - yaw;
            double leftBack_pwr = axial + lateral + yaw;
            double rightBack_pwr = axial - lateral - yaw;


            leftFront.setPower(reverse_multiplier * leftFront_pwr);
            rightFront.setPower(reverse_multiplier * rightFront_pwr);
            leftBack.setPower(reverse_multiplier * leftBack_pwr);
            rightBack.setPower(reverse_multiplier * rightBack_pwr);
            AprilTagDetections();
            telemetry.update();
            */

            AprilTagDetections();
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
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }

}