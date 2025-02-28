package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Yellow auto", group = "Examples")
public class yellowauto extends OpMode {
    private DcMotor rightExtend = null;
    private DcMotor Tilt = null;
    private DcMotor leftExtend = null;


    private CRServo Intake;
    private Servo directionalLeft;
    private Servo directionalRight;
    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8.5, 109, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose pickup1Pose = new Pose(-33.5, -45.5, Math.toRadians(0));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(-26, -45.5, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose scorePose = new Pose(-16, -9, Math.toRadians(90));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain dumpBlock1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        dumpBlock1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 108.500, Point.CARTESIAN),
                                new Point(17.257, 117.348, Point.CARTESIAN),
                                new Point(13.806, 126.551, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(13.806, 126.551, Point.CARTESIAN),
                                new Point(18.983, 120.799, Point.CARTESIAN),
                                new Point(29.912, 119.840, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();
        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(29.912, 119.840, Point.CARTESIAN),
                                new Point(18.983, 120.607, Point.CARTESIAN),
                                new Point(13.806, 126.743, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                follower.setMaxPower(.5);
                follower.followPath(dumpBlock1,false);
                leftExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftExtend.setTargetPosition(3600);
                rightExtend.setTargetPosition(3600);
                leftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftExtend.setPower(.75);
                rightExtend.setPower(-0.75);
                setPathState(1);
                break;

            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    Tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Tilt.setTargetPosition(-200);
                    Tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Tilt.setPower(.5);
                    directionalLeft.setPosition(.1);
                    directionalRight.setPosition(.1);
                    setPathState(2);
                }
                break;
            case 2:
               if(pathTimer.getElapsedTimeSeconds()>3){
                    Intake.setPower(-1);
                    setPathState(3);
               }
               break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    Intake.setPower(0);
                    directionalRight.setPosition(.85);
                    directionalLeft.setPosition(.85);
                    Tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Tilt.setTargetPosition(0);
                    Tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Tilt.setPower(.5);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    leftExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftExtend.setTargetPosition(0);
                    rightExtend.setTargetPosition(0);
                    leftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftExtend.setPower(1);
                    rightExtend.setPower(-1);
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    Tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Tilt.setTargetPosition(3400);
                    Tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Tilt.setPower(.5);
                    setPathState(6);
                }
                break;

            case 6:
                follower.setMaxPower(.75);
                follower.followPath(grabPickup2, true);
                Intake.setPower(1);
                setPathState(10000);
                break;
            case 10000:
                if(pathTimer.getElapsedTimeSeconds()>3){

                    setPathState(7);
                }
                break;
            case 7:
                follower.setMaxPower(.5);
                follower.followPath(grabPickup3, true);
                Tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Tilt.setTargetPosition(-200);
                Tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Tilt.setPower(.5);
                setPathState(8);
                break;
            case 8:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    Intake.setPower(0);
                    leftExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftExtend.setTargetPosition(3600);
                    rightExtend.setTargetPosition(3600);
                    leftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftExtend.setPower(.75);
                    rightExtend.setPower(-0.75);
                    setPathState(9);
                }
                break;
            case 9:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    directionalLeft.setPosition(.1);
                    directionalRight.setPosition(.1);
                    setPathState(10);
                }
                break;
            case 10:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    Intake.setPower(-1);
                    setPathState(11);
                }
                break;
            case 11:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    Intake.setPower(0);
                    directionalRight.setPosition(.85);
                    directionalLeft.setPosition(.85);
                }

                /*
            case 2:

                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;*/
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
                // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setMaxPower(.5);
        follower.setStartingPose(startPose);
        buildPaths();


        //define motors
        Tilt = hardwareMap.get(DcMotor.class, "tilt");
        Tilt.setDirection(DcMotorSimple.Direction.REVERSE);
        leftExtend = hardwareMap.get(DcMotor.class, "leftextend");
        rightExtend = hardwareMap.get(DcMotor.class, "rightextend");
        rightExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake = hardwareMap.get(CRServo.class, "intake");
        directionalLeft = hardwareMap.get(Servo.class, "directionalLeft");
        directionalRight = hardwareMap.get(Servo.class, "directionalRight");
        directionalRight.setDirection(Servo.Direction.REVERSE);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

