/*
 * Copyright (c) 2024 DigitalChickenLabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

/*
 * This OpMode illustrates how to use the DigitalChickenLabs OctoQuad Quadrature Encoder & Pulse Width Interface Module
 *
 * The OctoQuad has 8 input channels that can used to read either Relative Quadrature Encoders or Pulse-Width Absolute Encoder inputs.
 * Relative Quadrature encoders are found on most FTC motors, and some stand-alone position sensors like the REV Thru-Bore encoder.
 * Pulse-Width encoders are less common. The REV Thru-Bore encoder can provide its absolute position via a variable pulse width,
 * as can several sonar rangefinders such as the MaxBotix MB1000 series.
 *
 * This basic sample shows how an OctoQuad can be used to read the position three Odometry pods fitted
 * with REV Thru-Bore encoders.  For a more advanced example showing additional OctoQuad capabilities, see the SensorOctoQuadAdv sample.
 *
 * This OpMode assumes that the OctoQuad is attached to an I2C interface named "octoquad" in the robot configuration.
 *
 * The code assumes the first three OctoQuad inputs are connected as follows
 * - Chan 0: for measuring forward motion on the left side of the robot.
 * - Chan 1: for measuring forward motion on the right side of the robot.
 * - Chan 2: for measuring Lateral (strafing) motion.
 *
 * The encoder values may be reset to zero by pressing the X (left most) button on Gamepad 1.
 *
 * This sample does not show how to interpret these readings, just how to obtain and display them.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.tindie.com/products/35114/
 */
@TeleOp(name = "OctoQuad Basic", group="OctoQuad")
@Disabled
public class SensorOctoQuad extends LinearOpMode {

    // Identify which encoder OctoQuad inputs are connected to each odometry pod.
    private final int ODO_LEFT  = 0; // Facing forward direction on left side of robot (Axial motion)
    private final int ODO_RIGHT = 1; // Facing forward direction on right side or robot (Axial motion)
    private final int ODO_PERP  = 2; // Facing perpendicular direction at the center of the robot (Lateral motion)

    // Declare the OctoQuad object and members to store encoder positions and velocities
    private OctoQuad    octoquad;

    private int         posLeft;
    private int         posRight;
    private int         posPerp;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        // Connect to OctoQuad by referring to its name in the Robot Configuration.
        octoquad = hardwareMap.get(OctoQuad.class, "octoquad");

        // Read the Firmware Revision number from the OctoQuad and display it as telemetry.
        telemetry.addData("OctoQuad Firmware Version ", octoquad.getFirmwareVersion());

        // Reverse the count-direction of any encoder that is not what you require.
        // e.g. if you push the robot forward and the left encoder counts down, then reverse it so it counts up.
        octoquad.setSingleEncoderDirection(ODO_LEFT,  OctoQuad.EncoderDirection.REVERSE);
        octoquad.setSingleEncoderDirection(ODO_RIGHT, OctoQuad.EncoderDirection.FORWARD);
        octoquad.setSingleEncoderDirection(ODO_PERP,  OctoQuad.EncoderDirection.FORWARD);

        // Any changes that are made should be saved in FLASH just in case there is a sensor power glitch.
        octoquad.saveParametersToFlash();

        telemetry.addLine("\nPress START to read encoder values");
        telemetry.update();

        waitForStart();

        // Configure the telemetry for optimal display of data.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        // Set all the encoder inputs to zero.
        octoquad.resetAllPositions();

        // Loop while displaying the odometry pod positions.
        while (opModeIsActive()) {
            telemetry.addData(">", "Press X to Reset Encoders\n");

            // Check for X button to reset encoders.
            if (gamepad1.x) {
                // Reset the position of all encoders to zero.
                octoquad.resetAllPositions();
            }

            // Read all the encoder data.  Load into local members.
            readOdometryPods();

            // Display the values.
            telemetry.addData("Left ", "%8d counts", posLeft);
            telemetry.addData("Right", "%8d counts", posRight);
            telemetry.addData("Perp ", "%8d counts", posPerp);
            telemetry.update();
        }
    }

    private void readOdometryPods() {
        // For best performance, we should only perform ONE transaction with the OctoQuad each cycle.
        // Since this example only needs to read positions from a few channels, we could use either
        //   readPositionRange(idxFirst, idxLast) to get a select number of sequential channels
        // or
        //   readAllPositions() to get all 8 encoder readings
        //
        // Since both calls take almost the same amount of time, and the actual channels may not end up
        // being sequential, we will read all of the encoder positions, and then pick out the ones we need.
        int[] positions = octoquad.readAllPositions();
        posLeft  = positions[ODO_LEFT];
        posRight = positions[ODO_RIGHT];
        posPerp  = positions[ODO_PERP];
    }

    @TeleOp(name="AUTONOMOUS *BLUE*", group="Linear OpMode")

    public static class Autonomous_Blue extends LinearOpMode {
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

            MoveBase_USE_POSITION(1042, 1089, 1072, 1048);
            Halt();
            MoveBase_USE_POSITION(990, -1179, 1136, -855);
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
                    | (leftShoulder.isBusy()) | (rightShoulder.isBusy()) | (leftElbow.isBusy())
                    | (rightElbow.isBusy())){

            }
        }

    }
}
