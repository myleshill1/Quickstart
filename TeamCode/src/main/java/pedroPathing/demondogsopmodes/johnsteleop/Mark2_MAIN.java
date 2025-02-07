package pedroPathing.demondogsopmodes.johnsteleop;

import android.bluetooth.le.ScanSettings;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;

import java.sql.Time;

@TeleOp
public class Mark2_MAIN extends LinearOpMode {

    private PIDController controller;

    public static double p = 0.01, i = 0.0001, d = 0.0002;

    public static double f = 0.017;

    public static int target = 0;

    private final double ticks_in_degree = 1425.1 / 180;
    private DcMotorEx pivot;




    @Override
    public void runOpMode() throws InterruptedException {

        /*================================= Control Hub / Expansion Hub =================================*/

        DcMotor FL = hardwareMap.dcMotor.get("FL"); //
        DcMotor BL = hardwareMap.dcMotor.get("BL"); //
        DcMotor FR = hardwareMap.dcMotor.get("FR"); //
        DcMotor BR = hardwareMap.dcMotor.get("BR"); //

        pivot = hardwareMap.get(DcMotorEx.class, "pivot"); //

        controller = new PIDController(p, i, d);

        DcMotor Rslides = hardwareMap.dcMotor.get("Rslides"); //
        DcMotor Lslides = hardwareMap.dcMotor.get("Lslides"); //

        Servo claw = hardwareMap.servo.get("claw"); //
        Servo rotateClaw = hardwareMap.servo.get("rotateServo"); //
        Servo clawPivot = hardwareMap.servo.get("clawPivot"); //

        Servo RArm = hardwareMap.servo.get("RArm"); //
        Servo LArm = hardwareMap.servo.get("LArm"); //

        /*================================= Variables =================================*/

        ElapsedTime timer = new ElapsedTime();

        timer.startTime();

        int change_Mode = 1;

        double rotation = 0.52;

        boolean middleGrab = false;
        boolean goMiddle = false;

        boolean goBucket = false;
        boolean bucketScore = false;

        boolean prepare = false;
        boolean setup = false;
        boolean scoreSpeci = false;


        boolean zero = false;
        boolean zerotwo = false;

        /*================================= Init Motors/Servos =================================*/

        Lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RArm.setPosition(0.9);
        LArm.setPosition(0.9);

        clawPivot.setPosition(0.2);



        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Rslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //We need to find out which slide/pivot should be reversed

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        Lslides.setDirection(DcMotorSimple.Direction.REVERSE);
        LArm.setDirection(Servo.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        /*================================= Drive =================================*/

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x - rx) / denominator;
            double backLeftPower = (y - x - rx) / denominator;
            double frontRightPower = (y - x + rx) / denominator;
            double backRightPower = (y + x + rx) / denominator;

            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);
            /*================================= Switching Modes =================================*/

            /* sample */ if (gamepad1.left_bumper){

                change_Mode = 1;

            }

            /* Specimen */ if (gamepad1.right_bumper){

                change_Mode = 2;

            }

            /*================================= Omni Claw Parameters/Emergency buttons =================================*/

            rotateClaw.setPosition(rotation);

            controller.setPID(p, i, d);

            int pivotPos = pivot.getCurrentPosition();


            double pid = controller.calculate(pivotPos, target);

            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;


            pivot.setPower(power);

            if (gamepad2.left_stick_x > 0.5){

                rotation += 0.01;

            }

            if (-gamepad2.left_stick_x > 0.5){

                rotation -= 0.01;

            }

            if (rotation != rotateClaw.getPosition()){

                rotateClaw.setPosition(rotation);

            }

            if (rotation >= 1 && rotation > 0){

                rotation = 1;

            }

            else if (rotation <= 0){

                rotation = 0;

            }

            //EMERGENCY / MANUAL BUTTONS

            //open
            if(gamepad2.dpad_right){

                claw.setPosition(0);

            }

            //close
            if (gamepad2.dpad_left){

                claw.setPosition(1);

            }


            switch (change_Mode) {

                case 1: /* Sample */

                    /*================================= Going into middle =================================*/

                    if (gamepad2.x) {

                        claw.setPosition(0);

                        target = -710;

                        timer.reset();
                        goMiddle = true;

                    }

                    if (timer.seconds() >= 0.7 && goMiddle == true) {

                        Rslides.setTargetPosition(1000);
                        Lslides.setTargetPosition(1000);
                        Rslides.setPower(1);
                        Lslides.setPower(1);
                        Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    }

                    if (timer.seconds() >= 1 && goMiddle == true) {

                        RArm.setPosition(0.32);
                        LArm.setPosition(0.32);
                        clawPivot.setPosition(0.9);

                        goMiddle = false;

                    }

                    /*================================= Grabbing from middle =================================*/

                    if (gamepad2.b) {

                        RArm.setPosition(0.22);
                        LArm.setPosition(0.22);

                        timer.reset();
                        middleGrab = true;

                    }

                    if (timer.seconds() >= 0.4 && middleGrab == true) {

                        claw.setPosition(1);

                    }

                    if (timer.seconds() >= 1 && middleGrab == true) {

                        RArm.setPosition(0.7);
                        LArm.setPosition(0.7);

                        clawPivot.setPosition(0.3);

                        middleGrab = false;
                        zero = true;

                    }

                    if (timer.seconds() >= 1.4 && zero == true) {


                        target = 0;

                        Rslides.setTargetPosition(0);
                        Lslides.setTargetPosition(0);
                        Rslides.setPower(1);
                        Lslides.setPower(1);
                        Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        zero = false;

                    }

                    /*================================= Going up to High Basket =================================*/

                    if (gamepad2.y) {


                        target = 0;

                        RArm.setPosition(0.5);
                        LArm.setPosition(0.5);


                        timer.reset();
                        goBucket = true;

                    }

                    if (timer.seconds() >= 0.2 && goBucket == true) {

                        clawPivot.setPosition(0.4);

                        Rslides.setTargetPosition(1600); // find the max our slides go
                        Lslides.setTargetPosition(1600); // find the max our slides go
                        Rslides.setPower(1);
                        Lslides.setPower(1);
                        Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        goBucket = false;
                    }



                    /*================================= Scored in High Basket =================================*/

                    if (gamepad2.a) {

                        claw.setPosition(0);

                        timer.reset();
                        bucketScore = true;


                    }

                    if (timer.seconds() >= 0.7 && bucketScore == true) {

                        RArm.setPosition(0.4);
                        LArm.setPosition(0.4);


                        bucketScore = false;

                        zerotwo = true;

                    }

                    if (timer.seconds() >= 1.2 && zerotwo == true) {

                        Rslides.setTargetPosition(0);
                        Lslides.setTargetPosition(0);
                        Rslides.setPower(1);
                        Lslides.setPower(1);
                        Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    }

                    if (timer.seconds() >= 1.4 && Rslides.getCurrentPosition() <= 100 && Lslides.getCurrentPosition() <= 100 && zerotwo == true) {

                        target = 0;

                        zerotwo = false;

                    }

                    break;

                case 2: /* Specimen */

                    /*================================= Going into middle =================================*/

                    if (gamepad2.x) {

                        claw.setPosition(0);

                        pivot.setTargetPosition(815);
                        pivot.setPower(1);
                        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        timer.reset();
                        goMiddle = true;

                    }

                    if (timer.seconds() >= 1 && goMiddle == true) {

                        Rslides.setTargetPosition(1000);
                        Lslides.setTargetPosition(1000);
                        Rslides.setPower(1);
                        Lslides.setPower(1);
                        Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    }

                    if (timer.seconds() >= 1.2 && goMiddle == true) {

                        RArm.setPosition(0.32);
                        LArm.setPosition(0.32);
                        clawPivot.setPosition(0.48);

                        goMiddle = false;

                    }

                    /*================================= Grabbing from middle =================================*/

                    if (gamepad2.b) {

                        RArm.setPosition(0.22);
                        LArm.setPosition(0.22);

                        timer.reset();
                        middleGrab = true;

                    }

                    if (timer.seconds() >= 0.7 && middleGrab == true) {

                        claw.setPosition(1);

                    }

                    if (timer.seconds() >= 1.5 && middleGrab == true) {

                        RArm.setPosition(0.7);
                        LArm.setPosition(0.7);

                        clawPivot.setPosition(0.2);

                        middleGrab = false;
                        zero = true;

                    }

                    if (timer.seconds() >= 1.7 && zero == true) {


                        pivot.setTargetPosition(0);
                        pivot.setPower(1);
                        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        Rslides.setTargetPosition(0);
                        Lslides.setTargetPosition(0);
                        Rslides.setPower(1);
                        Lslides.setPower(1);
                        Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        zero = false;

                    }

                    /*================================= Set Up to grab Specimen =================================*/

                    if (gamepad2.right_bumper) {

                        claw.setPosition(0);

                        pivot.setTargetPosition(0);
                        pivot.setPower(1);
                        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        Rslides.setTargetPosition(0);
                        Lslides.setTargetPosition(0);
                        Rslides.setPower(1);
                        Lslides.setPower(1);
                        Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        RArm.setPosition(0.86);
                        LArm.setPosition(0.86);

                        clawPivot.setPosition(0.6);

                    }

                    /*================================= grab Specimen / set up =================================*/

                    if (gamepad2.y) {

                        claw.setPosition(1);

                        timer.reset();
                        setup = true;

                    }

                    if (timer.seconds() >= 0.5 && setup == true){

                        RArm.setPosition(0.05);
                        LArm.setPosition(0.05);

                        clawPivot.setPosition(0.7);

                        setup = false;
                    }


                    /*================================= Score Specimen =================================*/


                    break;

            }



            telemetry.addData("pivot", pivot.getCurrentPosition());
            telemetry.addData("Lslides", Lslides.getCurrentPosition());
            telemetry.addData("Rslides", Rslides.getCurrentPosition());
            telemetry.addData("Claw Rotation" , rotateClaw.getPosition());
            telemetry.addData("Claw Pivot", clawPivot.getPosition());
            telemetry.addData("Rotation value" , rotation);
            telemetry.addData("RArm" , RArm.getPosition());
            telemetry.addData("LArm" , LArm.getPosition());

            if (change_Mode == 1){

                telemetry.addData("State : SAMPLE" , change_Mode);

            }

            else if (change_Mode == 2){

                telemetry.addData("State : SPECIMEN" , change_Mode);

            }


            updateTelemetry(telemetry);


        }
    }
}
