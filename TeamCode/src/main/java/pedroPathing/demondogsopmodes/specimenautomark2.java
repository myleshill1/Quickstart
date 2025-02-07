package pedroPathing.demondogsopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/202
 */

@Autonomous(name = "specimenautomark2", group = "Examples")
public class specimenautomark2 extends OpMode {

    DcMotor FL = hardwareMap.dcMotor.get("FL"); //
    DcMotor BL = hardwareMap.dcMotor.get("BL"); //
    DcMotor FR = hardwareMap.dcMotor.get("FR"); //
    DcMotor BR = hardwareMap.dcMotor.get("BR"); //

    DcMotor pivot = hardwareMap.get(DcMotorEx.class, "pivot"); //

    DcMotor Rslides = hardwareMap.dcMotor.get("Rslides"); //
    DcMotor Lslides = hardwareMap.dcMotor.get("Lslides"); //

    Servo claw = hardwareMap.servo.get("claw"); //
    Servo rotateClaw = hardwareMap.servo.get("rotateServo"); //
    Servo clawPivot = hardwareMap.servo.get("clawPivot"); //

    Servo RArm = hardwareMap.servo.get("RArm"); //
    Servo LArm = hardwareMap.servo.get("LArm"); //


    boolean score = false;
    boolean grab = false;

    boolean setupforgrabspecimengood = false;



    public void setupforgrabSpecimen() {
        claw.setPosition(0);

        RArm.setPosition(0.1);
        LArm.setPosition(0.1);

        //pivotTarget = 0;

        Rslides.setTargetPosition(0);
        Lslides.setTargetPosition(0);
        Rslides.setPower(1);
        Lslides.setPower(1);
        Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void setupforscoreSpecimen() {


            claw.setPosition(1);

            actionTimer.resetTimer();
            grab = true;
        Rslides.setTargetPosition(100);
        Lslides.setTargetPosition(100);
        Rslides.setPower(1);
        Lslides.setPower(1);
        Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (actionTimer.getElapsedTimeSeconds() >= 0.5 && grab == true){

            RArm.setPosition(0.97);
            LArm.setPosition(0.97);

            grab = false;

        }

    }

    public void scoreSpecimen() {


            Rslides.setTargetPosition(750);
            Lslides.setTargetPosition(750);
            Rslides.setPower(1);
            Lslides.setPower(1);
            Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            actionTimer.resetTimer();
            score = true;



        if(actionTimer.getElapsedTimeSeconds() >= 0.8 && score == true){

            claw.setPosition(0);

        }

        if(actionTimer.getElapsedTimeSeconds() >= 1.5 && score == true){

            Rslides.setTargetPosition(0);
            Lslides.setTargetPosition(0);
            Rslides.setPower(1);
            Lslides.setPower(1);
            Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            score = false;

        }

    }



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
    private final Pose startPose = new Pose(8.0625, 66.5, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePreloadPose = new Pose(41.5, 66.5, Math.toRadians(0));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pushspec1Pose = new Pose(68.82, 23.86, Math.toRadians(180));

    private final Pose endpushspec1Pose = new Pose(37, 121, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pushspec2Pose = new Pose(43, 130, Math.toRadians(0));
    private final Pose endpushspec2Pose = new Pose(43, 130, Math.toRadians(0));
    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pushspec3Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose endpushspec3Pose = new Pose(49, 135, Math.toRadians(0));
    private final Pose scorecyclePose = new Pose(49, 135, Math.toRadians(0));

    private final Pose pickupcyclePose = new Pose(49, 135, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    private final Pose pushspec1controlPose = new Pose(13.67, 50.837, Math.toRadians(180));

    private final Pose pushspec2controlPose = new Pose(61.87, 30.33, Math.toRadians(180));

    private final Pose pushspec3controlPose = new Pose(62.11, 18.22, Math.toRadians(180));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park;
    private PathChain scorePreload, pushspec1, endpushspec1, pushspec2, endpushspec2, pushspec3, endpushspec3, scorecycle, pickupcycle;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */


        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading())
                .build();

        pushspec1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreloadPose), /* Control Point */ new Point(pushspec1controlPose), new Point(pushspec1Pose)))
                .setLinearHeadingInterpolation(scorePreloadPose.getHeading(), pushspec1Pose.getHeading())
                .build();


        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        endpushspec1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushspec1Pose), new Point(endpushspec1Pose)))
                .setLinearHeadingInterpolation(pushspec1Pose.getHeading(), endpushspec1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushspec2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(endpushspec1Pose), /* Control Point */ new Point(pushspec2controlPose), new Point(pushspec2Pose)))
                .setLinearHeadingInterpolation(endpushspec1Pose.getHeading(), pushspec2Pose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        endpushspec2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushspec2Pose), new Point(endpushspec2Pose)))
                .setLinearHeadingInterpolation(pushspec2Pose.getHeading(), endpushspec2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushspec3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(endpushspec2Pose), /* Control Point */ new Point(pushspec3controlPose), new Point(pushspec3Pose)))
                .setLinearHeadingInterpolation(endpushspec2Pose.getHeading(), pushspec3Pose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        endpushspec3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushspec3Pose), new Point(endpushspec3Pose)))
                .setLinearHeadingInterpolation(pushspec3Pose.getHeading(), endpushspec3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorecycle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(endpushspec3Pose), new Point(scorecyclePose)))
                .setLinearHeadingInterpolation(endpushspec3Pose.getHeading(), scorecyclePose.getHeading())
                .build();

        pickupcycle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorecyclePose), new Point(pickupcyclePose)))
                .setLinearHeadingInterpolation(scorecyclePose.getHeading(), pickupcyclePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
      //  park = new Path(new BezierCurve(new Point(scorecyclePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
      //  park.setLinearHeadingInterpolation(scorecyclePose.getHeading(), parkPose.getHeading());
        // ONLY USE IF SCORING IS FINISHED
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pushspec1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(endpushspec1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pushspec2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(endpushspec2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pushspec3,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {

                    setupforgrabSpecimen(); //set up for specimen grab from wall

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(endpushspec3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* pickup specimen one */
                    setupforscoreSpecimen();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(scorecycle,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* score specimen one */
                    scoreSpecimen();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */

                    follower.followPath(pickupcycle,true);
                    setupforgrabspecimengood = true;
                    setPathState(9);
                }
                break;
            case 9:
                if (setupforgrabspecimengood == true) {
                    setupforgrabSpecimen();
                }
                setupforgrabspecimengood = false;
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* pickup specimen two */
                    setupforscoreSpecimen();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(scorecycle,true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* score specimen two */
                    scoreSpecimen();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(pickupcycle,true);
                    setupforgrabspecimengood = true;
                    setPathState(11);
                }
                break;
            case 11:
                if (setupforgrabspecimengood == true) {
                    setupforgrabSpecimen();
                }
                setupforgrabspecimengood = false;
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* pickup specimen 3 */
                    setupforscoreSpecimen();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(scorecycle,true);
                    setPathState(12);
                }
                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score specimen 3 */
                    scoreSpecimen();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(pickupcycle,true);
                    setPathState(13);
                }
                break;
            case 13:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
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
        actionTimer = new Timer();
        actionTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();


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
