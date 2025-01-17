package pedroPathing.demondogsopmodes.johnsteleop;

import android.graphics.Color;
import android.preference.PreferenceActivity;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp
public class Mark_2_TeleOp extends LinearOpMode {

//    private DcMotorEx slides;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FL = hardwareMap.dcMotor.get("FL_rightodo_1");
        DcMotor BL = hardwareMap.dcMotor.get("BL");
        DcMotor FR = hardwareMap.dcMotor.get("FR_leftodo_0");
        DcMotor BR = hardwareMap.dcMotor.get("BR_strafeodo_2");


        Servo claw = hardwareMap.servo.get("claw");
        Servo rotateClaw = hardwareMap.servo.get("rotateServo");
        Servo clawPivot = hardwareMap.servo.get("clawPivot");

        Servo RArm = hardwareMap.servo.get("WHATEVER WE CALL IT");
        Servo LArm = hardwareMap.servo.get("WHATEVER WE CALL IT");


        DcMotor pivot = hardwareMap.dcMotor.get("Pivot");
        DcMotor Rslides = hardwareMap.dcMotor.get("Rslides");
        DcMotor Lslides = hardwareMap.dcMotor.get("Lslides");

        /* ====================================================================== Variables ====================================================================== */

        ElapsedTime timer = new ElapsedTime();

        timer.startTime();

        double rotation = 0.52;


        // Zero action Boolean
        // Might make this a button

        boolean zero;
        boolean zeroTwo; // In case we need to zero from bucket differently


        // first action boolean (In the order we do it)

        boolean goMiddle = false;

        // second action boolean

        boolean grabMiddle;

        // third action boolean

        boolean goHighBasket;

        // Fourth action boolean

        boolean scoredHighBasket;


        /* ====================================================================== Init Motors & Servos ====================================================================== */

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawPivot.setPosition(0.35);

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Rslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //We need to find out what on the robot should be reversed

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            /* ====================================================================== Drive ====================================================================== */

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

            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);


            /* ====================================================================== Rotating Claw Parameters / Manual stuff ====================================================================== */

            rotateClaw.setPosition(rotation);

            if (rotation != rotateClaw.getPosition()){

                rotateClaw.setPosition(rotation);

            }

            if (rotation >= 1 && rotation > 0){

                rotation = 1;

            }

            else if (rotation <= 0){

                rotation = 0;

            }

            if (gamepad2.left_stick_x > 0.5){

                rotation += 0.02;

            }

            if (-gamepad2.left_stick_x > 0.5){

                rotation -= 0.02;

            }

            //EMERGENCY MANUAL CLAW

            //open
            if(gamepad2.dpad_left){

                claw.setPosition(0.4);

            }

            //close
            if (gamepad2.dpad_right){

                claw.setPosition(0.9);

            }


            /* ====================================================================== Go to middle ====================================================================== */

            if (gamepad2.a){

                // Pivot has to go first
                // then slides
                // then arm & claw pivot

                claw.setPosition(0.4);

                pivot.setTargetPosition( /* find pos */ );

                pivot.setPower(1);

                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                goMiddle = true;

                timer.reset();


            }

            if (timer.seconds() >= 1 && goMiddle == true){

                Rslides.setTargetPosition(/* find pos */);
                Lslides.setTargetPosition(/* find pos */);

                Rslides.setPower(1);
                Lslides.setPower(1);

                Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            }

            if (timer.seconds() >= 1.5 && goMiddle == true{

                // Find a mid point for the Arm so arm hovers over samples

                RArm.setPosition(/* find pos */);
                LArm.setPosition(/* find pos */);

                clawPivot.setPosition(/* find pos */);

                rotateClaw.setPosition(/* find pos */);

                goMiddle = false;

            }

            /* ====================================================================== Grab from middle ====================================================================== */

            if (gamepad2.b){

                //Make the Arm go fully on the sample

                RArm.setPosition();
                LArm.setPosition();

                grabMiddle = true;

                timer.reset();


            }

            if (timer.seconds() >= 0.6 && grabMiddle == true){

                claw.setPosition(0.9);

            }

            if (timer.seconds() >= 1.5 && grabMiddle == true){

                RArm.setPosition(/* find pos */);
                LArm.setPosition(/* find pos */;

                grabMiddle = false;

                zero = true;
            }

            if (timer.seconds() >= 2 && zero == true){


                Rslides.setTargetPosition(0);
                Lslides.setTargetPosition(0);

                Rslides.setPower(1);
                Lslides.setPower(1);

                Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                zero = false;


            }

            /* ====================================================================== Going to High Basket  ====================================================================== */

            if (gamepad2.y){

                // pivot and slides go up

                pivot.setTargetPosition(0); // pivot is up on init (slides vertical) meaning this is the 0

                pivot.setPower(1);

                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                goHighBasket = true;

                timer.reset();

            }

            if (timer.seconds() >= 1 && goHighBasket == true){

                Rslides.setTargetPosition(/* find pos */);
                Lslides.setTargetPosition(/* find pos */);

                Rslides.setPower(1);
                Lslides.setPower(1);

                Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                RArm.setPosition(/* find pos */);
                LArm.setPosition(/* find pos */);

                clawPivot.setPosition(/* find pos */);

                goHighBasket = false;
            }

            /* ====================================================================== Scored on High Basket  ====================================================================== */

            if (gamepad2.x){

                claw.setPosition(/* find pos */);

                timer.reset();

                scoredHighBasket = true;

            }

            if (timer.seconds() >= 0.5 && scoredHighBasket == true){


                RArm.setPosition(/* find pos */);
                LArm.setPosition(/* find pos */);

                scoredHighBasket = false;

                zeroTwo = true;
            }

            if (timer.seconds() >= 1 && zeroTwo == true){

                Rslides.setTargetPosition(/* find pos */);
                Lslides.setTargetPosition(/* find pos */);

                Rslides.setPower(1);
                Lslides.setPower(1);

                Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                zeroTwo = false;


            }




            /* ====================================================================== Telemetry  ====================================================================== */



            telemetry.addData("pivot", pivot.getCurrentPosition());
            telemetry.addData("Lslides", Lslides.getCurrentPosition());
            telemetry.addData("Rslides", Rslides.getCurrentPosition());
            telemetry.addData("Claw Pivot", clawPivot.getPosition());
            telemetry.addData("Rotate claw" , rotateClaw.getPosition());
            telemetry.addData("Rotation value" , rotation);
            telemetry.addData("claw" , claw.getPosition());
            telemetry.addData("Right Arm" , RArm.getPosition());
            telemetry.addData("Left Arm" , LArm.getPosition());


            updateTelemetry(telemetry);


        }
    }
}

