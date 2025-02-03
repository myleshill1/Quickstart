package pedroPathing.demondogsopmodes.johnsteleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PID_M2TeleOP extends LinearOpMode {

    private PIDController Pivotcontroller;

    public static double pp = 0.022, pi = 0.18, pd = 0.00075;

    public static double pf = 0.073;

    public static int pivotTarget = 0;

    private final double Pivot_ticks_in_degree = 1425.1 / 180;
    private DcMotorEx pivot;


    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor FL = hardwareMap.dcMotor.get("FL"); //
        DcMotor BL = hardwareMap.dcMotor.get("BL"); //
        DcMotor FR = hardwareMap.dcMotor.get("FR"); //
        DcMotor BR = hardwareMap.dcMotor.get("BR"); //

        pivot = hardwareMap.get(DcMotorEx.class, "pivot"); //

        DcMotor Rslides = hardwareMap.dcMotor.get("Rslides"); //
        DcMotor Lslides = hardwareMap.dcMotor.get("Lslides"); //

        Servo claw = hardwareMap.servo.get("claw"); //
        Servo rotateClaw = hardwareMap.servo.get("rotateServo"); //
        Servo clawPivot = hardwareMap.servo.get("clawPivot"); //

        Servo RArm = hardwareMap.servo.get("RArm"); //
        Servo LArm = hardwareMap.servo.get("LArm"); //

        /*================================= Variables =================================*/

        Pivotcontroller = new PIDController(pp, pi, pd);

        ElapsedTime timer = new ElapsedTime();

        timer.startTime();

        double rotation = 0.52;

        boolean middleGrab = false;
        boolean goMiddle = false;
        boolean setup = false;
        boolean grab = false;
        boolean score = false;


        boolean zero = false;
        boolean zerotwo = false;


        /*================================= Clawpivot's CONSTANT POS =================================*/

        clawPivot.setPosition(0.5);

        /*================================= Init Motors/Servos =================================*/

        Lslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Rslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //We need to find out which slide/pivot should be reversed

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        Lslides.setDirection(DcMotorSimple.Direction.REVERSE);
        LArm.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;



        while (opModeIsActive()) {

            /*================================= PID MATH =================================*/

            Pivotcontroller.setPID(pp, pi, pd);

            int pivotPos = pivot.getCurrentPosition();


            double pid = Pivotcontroller.calculate(pivotPos, pivotTarget);

            double ff = Math.cos(Math.toRadians(pivotTarget / Pivot_ticks_in_degree)) * pf;

            double power = pid + ff;



            pivot.setPower(power);

            /*================================= Drive =================================*/

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

            FL.setPower(frontLeftPower * 0.8);
            BL.setPower(backLeftPower * 0.8);
            FR.setPower(frontRightPower * 0.8);
            BR.setPower(backRightPower * 0.8);



            /*================================= Omni Claw Parameters/Emergency buttons =================================*/

            rotateClaw.setPosition(rotation);

            if (gamepad2.left_stick_x > 0.5){

                rotation += 0.02;

            }

            if (-gamepad2.left_stick_x > 0.5){

                rotation -= 0.02;

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
            if(gamepad2.dpad_left){

                claw.setPosition(0.4);

            }

            //close
            if (gamepad2.dpad_right){

                claw.setPosition(0.9);

            }



            /*================================= Going into middle =================================*/

            if (gamepad2.a) {

                claw.setPosition(0);

                pivotTarget = -670;

                timer.reset();
                goMiddle = true;

            }

            if (timer.seconds() >= 1.5 && goMiddle == true) {

                Rslides.setTargetPosition(1000);
                Lslides.setTargetPosition(1000);
                Rslides.setPower(1);
                Lslides.setPower(1);
                Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if (timer.seconds() >= 1.7 && goMiddle == true) {

                RArm.setPosition(0.82);
                LArm.setPosition(0.82);
                clawPivot.setPosition(0.5);

                goMiddle = false;

            }

            /*================================= Grabbing from middle =================================*/

            if (gamepad2.b) {

                RArm.setPosition(0.86);
                LArm.setPosition(0.86);



                timer.reset();
                middleGrab = true;

            }

            if (timer.seconds() >= 0.7 && middleGrab == true){

                claw.setPosition(1);

            }

            if (timer.seconds() >= 1.5 && middleGrab == true){

                RArm.setPosition(0);
                LArm.setPosition(0);

                clawPivot.setPosition(0.2);

                middleGrab = false;
                zero = true;

            }

            if (timer.seconds() >= 1.2 && zero == true){

                pivotTarget = 0;

                Rslides.setTargetPosition(0);
                Lslides.setTargetPosition(0);
                Rslides.setPower(1);
                Lslides.setPower(1);
                Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                zero = false;

            }

            /*================================= Set up to grab specimen =================================*/

            if (gamepad2.right_bumper) {

                claw.setPosition(0);

                RArm.setPosition(0.1);
                LArm.setPosition(0.1);

                pivot.setTargetPosition(0);
                pivot.setPower(1);
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Rslides.setTargetPosition(0);
                Lslides.setTargetPosition(0);
                Rslides.setPower(1);
                Lslides.setPower(1);
                Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            /*================================= Set up to score =================================*/

            if (gamepad2.y) {

                claw.setPosition(1);

                timer.reset();
                grab = true;

            }

            if (timer.seconds() >= 0.5 && grab == true){

                RArm.setPosition(0.97);
                LArm.setPosition(0.97);

                grab = false;

            }


            /*================================= Set up to score =================================*/

            if (gamepad2.x) {

                Rslides.setTargetPosition(1100);
                Lslides.setTargetPosition(1100);
                Rslides.setPower(1);
                Lslides.setPower(1);
                Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                timer.reset();
                score = true;

            }

            if(timer.seconds() >= 1 && score == true){

                claw.setPosition(0);

            }

            if(timer.seconds() >= 1.2 && score == true){

                Rslides.setTargetPosition(0);
                Lslides.setTargetPosition(0);
                Rslides.setPower(1);
                Lslides.setPower(1);
                Rslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                score = false;

            }


            telemetry.addData("pivot", pivot.getCurrentPosition());
            telemetry.addData("Lslides", Lslides.getCurrentPosition());
            telemetry.addData("Rslides", Rslides.getCurrentPosition());
            telemetry.addData("Claw Rotation" , rotateClaw.getPosition());
            telemetry.addData("Claw Pivot", clawPivot.getPosition());
            telemetry.addData("Rotation value" , rotation);


            updateTelemetry(telemetry);


        }
    }
}