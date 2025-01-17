package pedroPathing.demondogsopmodes.johnsteleop;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp
public class Scrim_TeleOP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FL = hardwareMap.dcMotor.get("FL_rightodo_1");
        DcMotor BL = hardwareMap.dcMotor.get("BL");
        DcMotor FR = hardwareMap.dcMotor.get("FR_leftodo_0");
        DcMotor BR = hardwareMap.dcMotor.get("BR_strafeodo_2");

//        int CurrentColor;
        Servo claw = hardwareMap.servo.get("claw");
        Servo rotateClaw = hardwareMap.servo.get("rotateServo");
        Servo clawPivot = hardwareMap.servo.get("clawPivot");

//        Servo rotateServo = hardwareMap.servo.get("rotateServo");
//        Servo clawPivot = hardwareMap.servo.get("clawPivot");

//        ColorSensor colorsens = hardwareMap.get(ColorSensor.class, "colorsens");

        DcMotor Lpivot = hardwareMap.dcMotor.get("Lpivot");
        DcMotor Rpivot = hardwareMap.dcMotor.get("Rpivot");
        DcMotor Rslides = hardwareMap.dcMotor.get("Rslides");
        DcMotor Lslides = hardwareMap.dcMotor.get("Lslides");

        ElapsedTime timer = new ElapsedTime();

        timer.startTime();

        double rotation = 0.52;

        boolean middleGrab = false;
        boolean goMiddle = false;
        boolean goBucket = false;
        boolean bucketScore = false;

//        boolean specimen = false;

        boolean slidesIn = false;


        boolean zero = false;
        boolean zerotwo = false;


        Lpivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rpivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawPivot.setPosition(0.35);

        Rpivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lpivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Rslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //We need to find out which slide/pivot should be reversed

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        Rslides.setDirection(DcMotorSimple.Direction.REVERSE);
        Rpivot.setDirection(DcMotorSimple.Direction.REVERSE);

//       colorsens.enableLed(true);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

//            CurrentColor = Color.rgb(colorsens.red(), colorsens.green(), colorsens.blue());
//
//            if (JavaUtil.colorToSaturation(CurrentColor) >= 0 && JavaUtil.colorToHue(CurrentColor) < 270) {
//                gamepad1.rumble(1000);
//            }
//
//            else {
//                gamepad1.stopRumble();
//            }
//            if (JavaUtil.colorToValue(CurrentColor) >= 0.05 && JavaUtil.colorToSaturation(CurrentColor) >= 0.2 && JavaUtil.colorToHue(CurrentColor) > 30 && JavaUtil.colorToHue(CurrentColor) < 75)
//            {
//                gamepad1.rumble(1000);
//            }
//
//            else {
//                gamepad1.stopRumble();
//            }
//            if (JavaUtil.colorToSaturation(CurrentColor) >= 0.2 && JavaUtil.colorToHue(CurrentColor) >= 300 && JavaUtil.colorToHue(CurrentColor) <= 355 || JavaUtil.colorToHue(CurrentColor) >= 0 && JavaUtil.colorToHue(CurrentColor) < 75) {
//                gamepad1.rumble(1000);
//            }
//
//            else {
//                gamepad1.stopRumble();
//            }
//            if (JavaUtil.colorToValue(CurrentColor) < 25 && JavaUtil.colorToValue(CurrentColor) > 35 && JavaUtil.colorToSaturation(CurrentColor) <= 0.4 && JavaUtil.colorToHue(CurrentColor) >= 0 && JavaUtil.colorToHue(CurrentColor) <= 355){
//                gamepad1.stopRumble();
//            }

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

            //EMERGENCY MANUAL CLAW

            //open
            if(gamepad2.dpad_left){

                claw.setPosition(0.4);

            }

            //close
            if (gamepad2.dpad_right){

                claw.setPosition(0.9);

            }

            if(gamepad2.dpad_down){

                clawPivot.setPosition(0);

            }



            if (gamepad2.left_stick_x > 0.5){

                rotation += 0.02;


            }

            if (-gamepad2.left_stick_x > 0.5){

                rotation -= 0.02;


            }



            //going to middle


            if (gamepad2.a) {

                //pivot down or up

                claw.setPosition(0.4);
                clawPivot.setPosition(0.35);

                Lpivot.setTargetPosition(-325);
                Rpivot.setTargetPosition(-325);

                Lpivot.setPower(1);
                Rpivot.setPower(1);

                Lpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                timer.reset();

                goMiddle = true;

            }

            if (timer.seconds() >= 0.3 && goMiddle == true) {

                //slides out


                clawPivot.setPosition(0.9);


                Lslides.setTargetPosition(1500);
                Rslides.setTargetPosition(1500);

                Lslides.setPower(1);
                Rslides.setPower(1);

                Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                goMiddle = false;

            }

            // actually grabbing from middle

            if (gamepad2.y) {

                //pivot down to sample

                Lpivot.setTargetPosition(-180);
                Rpivot.setTargetPosition(-180);

                Lpivot.setPower(1);
                Rpivot.setPower(1);

                Lpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                timer.reset();

                middleGrab = true;

            }

            if (timer.seconds() >= 0.5 && middleGrab == true) {

                claw.setPosition(0.9);

                middleGrab = false;

                zero = true;


            }

            // setting everthing to zero

            if (timer.seconds() >= 1.5 && zero == true ) {

                clawPivot.setPosition(0.27);

                Lslides.setTargetPosition(0);
                Rslides.setTargetPosition(0);

                Lslides.setPower(-1);
                Rslides.setPower(-1);

                Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Lpivot.setTargetPosition(-350);
                Rpivot.setTargetPosition(-350);

                Lpivot.setPower(-1);
                Rpivot.setPower(-1);

                Lpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                zero = false;

            }

            //bucket score

            if (gamepad2.b) {

                clawPivot.setPosition(0.35);

                Lpivot.setTargetPosition(-1620);
                Rpivot.setTargetPosition(-1620);

                Lpivot.setPower(1);
                Rpivot.setPower(1);

                Lpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                timer.reset();

                goBucket = true;


            }

            if (timer.seconds() >= 1 && goBucket == true) {

                Lslides.setTargetPosition(2300);
                Rslides.setTargetPosition(2300);

                Lslides.setPower(1);
                Rslides.setPower(1);

                Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                goBucket = false;

            }

            if(gamepad2.x){

                clawPivot.setPosition(0);

                bucketScore = true;

                timer.reset();


            }

            if (timer.seconds() >= 0.15 && bucketScore == true){

                claw.setPosition(0.4);

                bucketScore = false;

                slidesIn = true;



            }

            if(timer.seconds() >= 0.75 && slidesIn == true){

                clawPivot.setPosition(0.5);

                Rslides.setTargetPosition(0);
                Lslides.setTargetPosition(0);

                Rslides.setPower(-1);
                Lslides.setPower(-1);

                Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesIn = false;

                zerotwo = true;

            }

            // setting everthing to zero

            if (timer.seconds() >= 1 && zerotwo == true && Rslides.getCurrentPosition() <= 100 && Lslides.getCurrentPosition() <= 100) {

                claw.setPosition(0.1);


                Lpivot.setTargetPosition(0);
                Rpivot.setTargetPosition(0);

                Lpivot.setPower(-0.7);
                Rpivot.setPower(-0.7);

                Lpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rpivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                zerotwo = false;

            }



            telemetry.addData("Lpivot", Lpivot.getCurrentPosition());
            telemetry.addData("Rpivot", Rpivot.getCurrentPosition());
            telemetry.addData("Lslides", Lslides.getCurrentPosition());
            telemetry.addData("Rslides", Rslides.getCurrentPosition());
            telemetry.addData("Claw Rotation" , rotateClaw.getPosition());
            telemetry.addData("Claw Pivot", clawPivot.getPosition());
            telemetry.addData("Rotate claw" , rotateClaw.getPosition());
            telemetry.addData("Rotation value" , rotation);
            telemetry.addData("claw" , claw.getPosition());


            updateTelemetry(telemetry);


        }
    }
}
