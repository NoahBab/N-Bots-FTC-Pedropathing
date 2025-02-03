/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Set;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="First_try")
//@Disabled
public class NewOpModetherightone extends LinearOpMode {
    IMU imu;
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightExtend = null;
    private DcMotor rightTilt = null;
    private DcMotor leftExtend = null;
    private DcMotor leftTilt = null;
    //private CRServo Intake;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    //private Servo gateservo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;
    double leftExtendPower = 0.0;
    double rightExtendPower = 0.0;
    //double rightTiltPower = 0.0;
    //double leftTiltPower = 0.0;
    double  maxHozExtend = 33;

    @Override
    public void runOpMode() {
        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        //Intake = hardwareMap.get(CRServo.class, "intake");
        //gateservo = hardwareMap.get(Servo.class, "gate");
        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         *
         * If you are using a REV 9-Axis IMU, you can use the Rev9AxisImuOrientationOnRobot class instead of the
         * RevHubOrientationOnRobot class, which has an I2cPortFacingDirection instead of a UsbFacingDirection.
         */

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.



        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        //rightTilt = hardwareMap.get(DcMotor.class, "righttilt");
        //leftTilt = hardwareMap.get(DcMotor.class, "lefttilt");
        //leftExtend = hardwareMap.get(DcMotor.class, "leftextend");
        //rightExtend = hardwareMap.get(DcMotor.class, "rightextend");
        // ####################################################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions and names from configuation.            !!!!!
        // ####################################################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");

        waitForStart();
        runtime.reset();
        double setAngle = 0.0;
        //check positive or negative values on this
        double ProConstant = 0.1;
        //leftTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftTilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double startAngle =  90;
        //double startTilt = leftTilt.getCurrentPosition();
        double currentPos;
        double currentAngle;
        double currentExtend;
        double currentExtendInches;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //currentPos = leftTilt.getCurrentPosition();
            //currentExtend = leftExtend.getCurrentPosition();
            double max;
            double botheading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double stick_y = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double stick_x =  gamepad1.left_stick_x;
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Angle", botheading);
            telemetry.update();
            if(gamepad1.a){
                imu.resetYaw();
                //leftTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //leftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //proportional algorithm for angle correction
            //double error = setAngle-botheading;
            //double yaw = error*ProConstant;

            //yaw += gamepad1.right_stick_x;


            double x_rotated = stick_x * Math.cos(-botheading) - stick_y * Math.sin(-botheading);
            double y_rotated = stick_x * Math.sin(-botheading) + stick_y * Math.cos(-botheading);
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower  = x_rotated + y_rotated + yaw;
            double rightFrontPower = x_rotated - y_rotated + yaw;
            double leftBackPower   = x_rotated - y_rotated - yaw;
            double rightBackPower  = x_rotated + y_rotated - yaw;


            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            if(gamepad1.right_bumper){
                leftFrontPower /= 2;
                rightBackPower /=2;
                leftBackPower /= 2;
                rightFrontPower /= 2;
            }
            if(gamepad1.left_bumper){
                leftFrontPower /= 4;
                rightBackPower /=4;
                leftBackPower /= 4;
                rightFrontPower /= 4;
            }
            /*
            if(gamepad1.x){
                Intake.setPower(-1);
            }
            else if (gamepad1.b) {
                Intake.setPower(1);

            }else{
                Intake.setPower(0);



            }
*/

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

/*
            if (gamepad2.right_stick_y > 0) {
                rightTilt.setPower(-1);

            }else if (gamepad2.right_stick_y < 0){
                rightTilt.setPower(1);

            }else{
                rightTilt.setPower(0);
            }
            currentAngle = 90-((currentPos * 360) / 537.7) / 28;
            if(currentAngle>=100){
                leftTilt.setPower(.1);
            }else {
                if (gamepad2.left_stick_y > 0) {
                    leftTilt.setPower(-1 * gamepad2.left_stick_y);

                } else if (gamepad2.left_stick_y < 0) {
                    leftTilt.setPower(-1 * gamepad2.left_stick_y);

                } else {
                    leftTilt.setPower(0);
                }
            }
            //double targetPos =  1/(1.483 * 3.141596) * 537.7 * inches;
            currentExtendInches = (currentExtend*1.483*3.1415)/537.7;
            telemetry.addData("current angle", currentAngle);
            telemetry.addData("currentExtendInches", currentExtendInches);
            telemetry.addData("max extention",maxHozExtend/Math.cos(Math.toRadians(currentAngle)));
            double temp = maxHozExtend/Math.cos(Math.toRadians(currentAngle));
            if(currentExtendInches+13 > temp && currentAngle<60){
                leftExtend.setPower(-0.5);
            }else{
                if (gamepad2.left_bumper) {
                    leftExtend.setPower(-1);

                }else if (gamepad2.left_trigger > 0){
                    //((temp*537.7*28)/360);
                    leftExtend.setPower(1);

                    leftExtend.setPower(1);

                }else{
                    leftExtend.setPower(0);
                }
            }
            if(gamepad2.a){
                leftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftExtend.setTargetPosition(1000);
                leftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.right_stick_y > 0) {
                rightTilt.setPower(-1 * gamepad2.right_stick_y);

            } else if (gamepad2.left_stick_y < 0) {
                rightTilt.setPower(-1 * gamepad2.right_stick_y);

            } else {
                rightTilt.setPower(0);
*/

        }
    }}
