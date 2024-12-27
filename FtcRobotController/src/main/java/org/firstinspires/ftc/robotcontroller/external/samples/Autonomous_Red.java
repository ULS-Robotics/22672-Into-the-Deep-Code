package org.firstinspires.ftc.robotcontroller.external.samples;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

@TeleOp(name="AUTONOMOUS *Red*", group="Linear OpMode")

public class Autonomous_Red extends LinearOpMode {
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

    private static final boolean USE_WEBCAM = true;

    private VisionPortal visionPortal;

    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;

    private int DesiredAprilTag_ID = 0;

    private double DesiredDistance_INCH = 12;

    private int RunTimeCounter = 0;

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
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)

        waitForStart();

        initAprilTag();
        //first extend the arms to load the specimen and then move towards the center structure
        // then enter the main loop

        while (opModeIsActive()) {

            /*
            How this code should be done in autonomous
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            */

            if (RunTimeCounter == 1) {
                sleep(100);

                MoveArms_POSITION("H"); //TEMP DATA

                MoveBase(0.5, 0, 0, -1); //AGAIN, TEMP DATA
            }

            List<Object> AprilTagResults = AprilTagReturn();

            double yaw_value = Double.parseDouble(AprilTagResults.get(5).toString());
            double distance_value = Double.parseDouble(AprilTagResults.get(3).toString());

            int AprilTag_ID_Localized = Integer.parseInt(AprilTagResults.get(0).toString());


        }
        RunTimeCounter += 1;
    }
   // packaged functions to move things
    public void MoveElbow_ANALOG(int Elbow_position){
        /*This function uses ENCODERS to move arms to a given POSITION
        * i.e.: Set a given location measured through experiment and
        *       put them into this function
        * */
        leftElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftElbow.setTargetPosition(Elbow_position);
        rightElbow.setTargetPosition(- Elbow_position);

        leftElbow.setPower(0.5);
        rightShoulder.setPower(-0.5);

        leftElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void MoveArms_POSITION(String Position_Code){


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

    public void MoveBase(double axial,
                         double lateral,
                         double yaw,
                         int ReverseMultiplier){
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
        leftFront.setPower(ReverseMultiplier * leftFront_pwr);
        rightFront.setPower(ReverseMultiplier * rightFront_pwr);
        leftBack.setPower(ReverseMultiplier * leftBack_pwr);
        rightBack.setPower(ReverseMultiplier * rightBack_pwr);
    }

    public void MoveBase_TO_POSITION(List<Integer> Position){
        /*It takes a list input:
        * INDEX     |       Value
        * [0]               Left Front Position
        * [1]               Right Front Position
        * [2]               Left Back Position
        * [3]               Right Back Position
        * */

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    //APRIL TAGS!

    private void initAprilTag() {

        /*Initialize the april tag processor, if this function works, do not touch or change it. */

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Eyes"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    @SuppressLint("DefaultLocale")
    private List<Object> AprilTagReturn() {

        /*This function will gather the data from the recognized April Tags in a list
        *
        *       <NOTE> The way this function is currently implemented is not the most ideal way to
        *              do so. By which I mean that it objectified the List output, this is known to
        *              cause:
        *                   + Extremely insufficient memory usage
        *                   + Potential danger of a memory leak
        *                   + Redundant conversion between data types, when used, is required
        *                   + Potentially slowing down the processing speed of the robot
        *              But with the given time, it is the best method I can come up with. So in
        *              the future, if anyone is more trained with Java (anyone is better
        *              compared to me, frankly)
        *
        * */

        List<Object> AprilTagResults = new ArrayList<>();

        int AprilTage_ID = 0;
        String AprilTage_Name = null;
        double x_position = 0;
        double y_position = 0;
        double z_position = 0;
        double yaw_position = 0;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

            if (detection.metadata != null) {

                /*INDEX 0*/ AprilTage_ID = detection.id; // int
                /*INDEX 1*/ AprilTage_Name = detection.metadata.name; //string
                /*INDEX 2*/ x_position = detection.robotPose.getPosition().x; //double
                /*INDEX 3*/ y_position = detection.robotPose.getPosition().y; //double
                /*INDEX 4*/ z_position = detection.robotPose.getPosition().z; //double
                /*INDEX 5*/ yaw_position = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES); //double

            }
        }
        AprilTagResults.add(AprilTage_ID);
        AprilTagResults.add(AprilTage_Name);
        AprilTagResults.add(x_position);
        AprilTagResults.add(y_position);
        AprilTagResults.add(z_position);
        AprilTagResults.add(yaw_position);
        // AprilTagResults = [AprilTag_ID, AprilTage_Name, x_position, y_position, z_position]
        return AprilTagResults;
    }
}

