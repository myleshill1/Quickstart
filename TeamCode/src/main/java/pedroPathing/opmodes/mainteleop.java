package pedroPathing.opmodes;



import android.widget.Switch;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import pedroPathing.opmodes.RobotConstants;




@TeleOp
public class mainteleop extends LinearOpMode {

    RobotConstants robot = new RobotConstants();



    public enum STATE {

        NEUTRAL,
        MIDDLE,
        MIDDLEGRAB,
        SPECI,
        SAMPLE,


    }



//    PID_Slides pidSlides = new PID_Slides();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.SlideController = new PIDController(RobotConstants.sp, RobotConstants.si, RobotConstants.sd);
        robot.PivotController = new PIDController(RobotConstants.p, RobotConstants.i, RobotConstants.d);

        robot.FL = hardwareMap.dcMotor.get("FL_Rodo"); //
        robot.BL = hardwareMap.dcMotor.get("BL_Podo"); //
        robot.FR = hardwareMap.dcMotor.get("FR_Lodo"); //
        robot.BR = hardwareMap.dcMotor.get("BR"); //

        robot.pivot = hardwareMap.get(DcMotorEx.class, "pivot"); //

        robot.Rslides = hardwareMap.get(DcMotorEx.class, "RSlides");
        robot.Lslides = hardwareMap.get(DcMotorEx.class, "LSlides");

        robot.Claw = hardwareMap.servo.get("Claw"); //
        robot.Omni = hardwareMap.servo.get("Omni"); //
        robot.Wrist = hardwareMap.servo.get("Wrist"); //

        robot.RArm = hardwareMap.servo.get("Rarm"); //
        robot.LArm = hardwareMap.servo.get("Larm"); //

        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.slidesTarget = 0;

        double rotation = 0.93;

        robot.Rslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Lslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Rslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Rslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Lslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //We need to find out which slide/pivot should be reversed

        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.Rslides.setDirection(DcMotorSimple.Direction.REVERSE);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        ElapsedTime TIMER = new ElapsedTime();


        TIMER.startTime();
        boolean MIDDLE = false;
        boolean NEUTRAL = true;
        boolean SAMPLED = false;
        boolean GRAB = false;

        STATE state = STATE.NEUTRAL;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            robot.Omni.setPosition(rotation);

            if (gamepad2.left_stick_x > 0.5) {

                rotation += 0.02;

            }

            if (-gamepad2.left_stick_x > 0.5) {

                rotation -= 0.02;

            }

            if (rotation != robot.Omni.getPosition()) {

                robot.Omni.setPosition(rotation);

            }

            if (rotation >= 1 && rotation > 0) {

                rotation = 1;

            } else if (rotation <= 0) {

                rotation = 0;

            }

            //---------omni math----------

            robot.SlideController.setPID(robot.sp, robot.si, robot.sd);

            int RslidePos = robot.Rslides.getCurrentPosition();
            int LslidePos = robot.Lslides.getCurrentPosition();


            double Rpid = robot.SlideController.calculate(RslidePos, robot.slidesTarget);
            double Lpid = robot.SlideController.calculate(LslidePos, robot.slidesTarget);

            double Rff = Math.cos(Math.toRadians(robot.slidesTarget / robot.sticks_in_degree)) * robot.sf;
            double Lff = Math.cos(Math.toRadians(robot.slidesTarget / robot.sticks_in_degree)) * robot.sf;

            double Rpower = Rpid + Rff;
            double LPower = Lpid + Lff;


            robot.Rslides.setPower(Rpower);
            robot.Lslides.setPower(LPower);


            telemetry.addData("RslidePos: ", RslidePos);
            telemetry.addData("LslidePos: ", LslidePos);
            telemetry.addData("Target: ", robot.slidesTarget);
            //==============================

            robot.PivotController.setPID(robot.p, robot.i, robot.d);

            int pivotPos = robot.pivot.getCurrentPosition();


            double pid = robot.PivotController.calculate(pivotPos, robot.pivotTarget);

            double ff = Math.cos(Math.toRadians(robot.pivotTarget / robot.ticks_in_degree)) * robot.f;

            double power = pid + ff;


            robot.pivot.setPower(power);

            //-------------slide pid stuff--------------


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.FL.setPower(frontLeftPower);
            robot.BL.setPower(backLeftPower);
            robot.FR.setPower(frontRightPower);
            robot.BR.setPower(backRightPower);

            telemetry.addData("Rslide", robot.Rslides.getCurrentPosition());
            telemetry.addData("Lslide", robot.Lslides.getCurrentPosition());

            telemetry.update();


            //Slide isbusy after zero for like 0.5s -> give the motors zero output


            switch (state) {

                case NEUTRAL:

                    if (NEUTRAL) {

                        rotation = 0.93;

                        robot.slidesTarget = 0;

                        robot.LArm.setPosition(0.35);
                        robot.RArm.setPosition(0.35);

                        robot.Wrist.setPosition(0.4);
                        robot.Claw.setPosition(0.24);

                    }

                    if (robot.Lslides.getCurrentPosition() < 200 && robot.Rslides.getCurrentPosition() < 200 && NEUTRAL) {

                        robot.pivotTarget = -1500;

                        NEUTRAL = false;
                    }

                    if (gamepad2.b) {
                        state = STATE.MIDDLE;
                        TIMER.reset();
                        MIDDLE = true;
                    }

                    if (gamepad2.x) {
                        state = STATE.SAMPLE;
                        TIMER.reset();
                        SAMPLED = true;
                    }

                    if (gamepad2.y) {
                        state = STATE.SPECI;
                        TIMER.reset();
                    }

                    break;
                case MIDDLE:

                    if(TIMER.seconds() >= 0.15 & robot.pivot.getCurrentPosition() < -1350){

                        robot.slidesTarget = 600;

                    }

                    if (TIMER.seconds() >= 0.8 && MIDDLE) {

                        robot.LArm.setPosition(0.37);
                        robot.RArm.setPosition(0.37);

                        robot.Wrist.setPosition(0.4);
                        robot.Claw.setPosition(0.05);

                        MIDDLE = false;
                        state = STATE.MIDDLEGRAB;
                    }

                    break;

                case MIDDLEGRAB:

                    if(gamepad2.right_bumper){

                        robot.LArm.setPosition(0.265);
                        robot.RArm.setPosition(0.265);
                        robot.Wrist.setPosition(0.5);


                        TIMER.reset();
                        GRAB = true;

                    }

                    if(TIMER.seconds() > 0.2 && GRAB){
                        robot.Claw.setPosition(0.236);

                        GRAB = false;
                    }

                    if(gamepad2.left_bumper){

                        robot.LArm.setPosition(0.37);
                        robot.RArm.setPosition(0.37);

                        robot.Wrist.setPosition(0.5);
                        robot.Claw.setPosition(0.05);

                    }

                    if(gamepad2.a){
                        state = STATE.NEUTRAL;
                        NEUTRAL = true;
                    }


                    break;

                case SAMPLE:

                    if(TIMER.seconds() > 0.15 && SAMPLED){

                        robot.pivotTarget = 1510;


                    }

                    if (TIMER.seconds() > 0.65 && SAMPLED && robot.pivot.getCurrentPosition() > -100 ) {

                        rotation = 0.37;

                        robot.slidesTarget = 1350;

                    }

                    if (TIMER.seconds() > 0.85 && robot.Rslides.getCurrentPosition() > 750 && robot.Lslides.getCurrentPosition() > 750 && SAMPLED){

                        robot.LArm.setPosition(0.3);
                        robot.RArm.setPosition(0.3);

                        robot.Wrist.setPosition(0.2);

                        SAMPLED = false;

                    }

                    if (gamepad2.right_trigger > 0.75){

                        robot.Claw.setPosition(0.05);


                    }

                    if(gamepad2.dpad_left){

                        state = STATE.NEUTRAL;

                        NEUTRAL = true;

                    }


                    break;





            }

        }
    }
}



