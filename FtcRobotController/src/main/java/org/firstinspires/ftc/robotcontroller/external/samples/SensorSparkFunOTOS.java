/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Objects;

/*
 * This OpMode illustrates how to use the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS)
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_otos".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.sparkfun.com/products/24904
 */
@TeleOp(name = "Sensor: SparkFun OTOS", group = "Sensor")
@Disabled
public class SensorSparkFunOTOS extends LinearOpMode {
    // Create an instance of the sensor
    SparkFunOTOS myOtos;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

        // Wait for the start button to be pressed
        waitForStart();

        // Loop until the OpMode ends
        while (opModeIsActive()) {
            // Get the latest position, which includes the x and y coordinates, plus the
            // heading angle
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            // Reset the tracking if the user requests it
            if (gamepad1.y) {
                myOtos.resetTracking();
            }

            // Re-calibrate the IMU if the user requests it
            if (gamepad1.x) {
                myOtos.calibrateImu();
            }

            // Inform user of available controls
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();

            // Log the position to the telemetry
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);

            // Update the telemetry on the driver station
            telemetry.update();
        }
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    @TeleOp(name="AUTONOMOUS *RED*", group="Linear OpMode")

    public static class Autonomous_Red extends LinearOpMode {
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
}
