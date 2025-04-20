package pedroPathing.opmodes;

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
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import pedroPathing.opmodes.AutoSubsystems.Claw;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;


import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.opmodes.AutoSubsystems.Larm;
import pedroPathing.opmodes.AutoSubsystems.Omni;
import pedroPathing.opmodes.AutoSubsystems.Rarm;
import pedroPathing.opmodes.AutoSubsystems.Slides;
import pedroPathing.opmodes.AutoSubsystems.Wrist;

@Autonomous(name = "specimenautomark2", group = "Examples")
public class fivespecauto extends PedroOpMode {

    public fivespecauto() {
        super(
                Claw.INSTANCE,
                Wrist.INSTANCE,
                Larm.INSTANCE,
                Rarm.INSTANCE,
                Slides.INSTANCE,
                Omni.INSTANCE

        );
    }
    RobotConstants robot = new RobotConstants();

    double scoringspecimenx = 41.5;
    double grabspecimenx = 10;
    double grabspecimeny = 28.5;

    private final Pose startPose = new Pose(8.0625, 66.5, Math.toRadians(-90));
    private final Pose scorePreloadPose = new Pose(scoringspecimenx, 66.5, Math.toRadians(0));
    private final Pose pushspec1Pose = new Pose(68.82, 23.86, Math.toRadians(0));
    private final Pose endpushspec1Pose = new Pose(13, 23.86, Math.toRadians(0));
    private final Pose pushspec2Pose = new Pose(68.82, 12, Math.toRadians(0));
    private final Pose endpushspec2Pose = new Pose(13, 12, Math.toRadians(0));

    private final Pose pushspec3Pose = new Pose(68.82, 6, Math.toRadians(0));
    private final Pose endpushspec3Pose = new Pose(grabspecimenx, 6, Math.toRadians(0));
    private final Pose scorespec2Pose = new Pose(scoringspecimenx, 73, Math.toRadians(0));
    private final Pose scorespec3Pose = new Pose(scoringspecimenx, 71.5, Math.toRadians(0));
    private final Pose scorespec4Pose = new Pose(scoringspecimenx, 70, Math.toRadians(0));
    private final Pose scorespec5Pose = new Pose(scoringspecimenx, 68.5, Math.toRadians(0));
    private final Pose pickupcyclePose = new Pose(grabspecimenx, grabspecimeny, Math.toRadians(0));
    private final Pose parkPose = new Pose(10, 20, Math.toRadians(0));
    private final Pose pushspec1controlPose = new Pose(1.91, 53.35, Math.toRadians(0));

    private final Pose pushspec2controlPose = new Pose(61.87, 30.33, Math.toRadians(0));

    private final Pose pushspec3controlPose = new Pose(62.11, 18.22, Math.toRadians(0));


    private PathChain park;
    private PathChain scorePreload;
    private PathChain pushspec1;
    private PathChain endpushspec1;
    private PathChain pushspec2;
    private PathChain endpushspec2;
    private PathChain pushspec3;
    private PathChain endpushspec3;
    private PathChain scorespec2;
    private PathChain pickupspec3;

    private PathChain scorespec3;
    private PathChain pickupspec4;
    private PathChain scorespec4;
    private PathChain pickupspec5;

    private PathChain scorespec5;




    /**
     * Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts.
     **/
    public void buildPaths() {
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
        scorespec2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(endpushspec3Pose), new Point(scorespec2Pose)))
                .setLinearHeadingInterpolation(endpushspec3Pose.getHeading(), scorespec2Pose.getHeading())
                .build();

        pickupspec3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorespec2Pose), new Point(pickupcyclePose)))
                .setLinearHeadingInterpolation(scorespec2Pose.getHeading(), pickupcyclePose.getHeading())
                .build();
        scorespec3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupcyclePose), new Point(scorespec3Pose)))
                .setLinearHeadingInterpolation(pickupcyclePose.getHeading(), scorespec3Pose.getHeading())
                .build();
        pickupspec4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorespec3Pose), new Point(pickupcyclePose)))
                .setLinearHeadingInterpolation(scorespec3Pose.getHeading(), pickupcyclePose.getHeading())
                .build();
        scorespec4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupcyclePose), new Point(scorespec4Pose)))
                .setLinearHeadingInterpolation(pickupcyclePose.getHeading(), scorespec4Pose.getHeading())
                .build();
        pickupspec5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorespec4Pose), new Point(pickupcyclePose)))
                .setLinearHeadingInterpolation(scorespec4Pose.getHeading(), pickupcyclePose.getHeading())
                .build();
        scorespec5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupcyclePose), new Point(scorespec5Pose)))
                .setLinearHeadingInterpolation(pickupcyclePose.getHeading(), scorespec5Pose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorespec5Pose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorespec5Pose.getHeading(), parkPose.getHeading())
                .build();
    }

    public Command setupforgrabSpecimen() {

        return new SequentialGroup(
          new ParallelGroup(
                  Claw.INSTANCE.open().thenWait(0.2),
                  Wrist.INSTANCE.grabSpecimen(),
                  Larm.INSTANCE.grabSpecimen(),
                  Rarm.INSTANCE.grabSpecimen(),
                  Omni.INSTANCE.reversed(),
                  Slides.INSTANCE.toZero()
          )
        );

    }

    public Command setupforscoreSpecimen() {

        return new SequentialGroup(
                new ParallelGroup(
                        Claw.INSTANCE.close(),
                        Wrist.INSTANCE.setupScorespecimen(),
                        Larm.INSTANCE.setupScorespecimen(),
                        Rarm.INSTANCE.setupScorespecimen(),
                        Slides.INSTANCE.toSpecimenscore()
                )
        );
    }
    public Command scoreSpecimen(){
        return new SequentialGroup(
                new ParallelGroup(
                        Wrist.INSTANCE.scorespecimen(),
                        Larm.INSTANCE.scorespecimen(),
                        Rarm.INSTANCE.scorespecimen()
                ).thenWait(1),
                Claw.INSTANCE.open()
        );

    }

    public Command preloadSpecimen(){
        return new SequentialGroup(
                new FollowPath(scorePreload).thenWait(1),
                setupforscoreSpecimen().thenWait(1),
                scoreSpecimen()
        );
    }

    public Command endAutoservopos() {
        return new SequentialGroup(
                new ParallelGroup(
                        Wrist.INSTANCE.endofauto(),
                        Larm.INSTANCE.endofauto(),
                        Rarm.INSTANCE.endofauto(),
                        Omni.INSTANCE.normal()

                )
        );
    }
    public Command runAuto(){
        return new SequentialGroup(
                preloadSpecimen(),
                new ParallelGroup(
                        setupforgrabSpecimen(),
                        new FollowPath(pushspec1)
                ),
                new FollowPath(endpushspec1),
                new FollowPath(pushspec2),
                new FollowPath(endpushspec2),
                new FollowPath(pushspec3),
                new FollowPath(endpushspec3),
                //start cycling here
                new ParallelGroup(
                        setupforscoreSpecimen().thenWait(0.5),
                        new FollowPath(scorespec2)
                ).thenWait(2),
                scoreSpecimen(),
                //scores spec 2, now go back
                new ParallelGroup(
                        new FollowPath(pickupspec3),
                        setupforgrabSpecimen()
                ),
                new ParallelGroup(
                        setupforscoreSpecimen().thenWait(0.5),
                        new FollowPath(scorespec3)
                ).thenWait(2),
                scoreSpecimen(),
                //scores spec 3, now go back
                new ParallelGroup(
                        new FollowPath(pickupspec4),
                        setupforgrabSpecimen()
                ),
                new ParallelGroup(
                        setupforscoreSpecimen().thenWait(0.5),
                        new FollowPath(scorespec4)
                ).thenWait(2),
                scoreSpecimen(),
                //scores spec 4, now go back
                new ParallelGroup(
                        new FollowPath(pickupspec5),
                        setupforgrabSpecimen()
                ),
                new ParallelGroup(
                        setupforscoreSpecimen().thenWait(0.5),
                        new FollowPath(scorespec5)
                ).thenWait(2),
                scoreSpecimen(),
                new FollowPath(park),
                endAutoservopos()
        );
    };
    @Override
    public void onInit() {



        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap,FConstants.class,LConstants.class);
        follower.setStartingPose(startPose);
        Claw.INSTANCE.close();
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        runAuto().invoke();

    }

}





