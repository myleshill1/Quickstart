package pedroPathing.demondogsopmodes.johnsteleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class NewServoTEST extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.servo.get("claw"); //
        Servo rotateClaw = hardwareMap.servo.get("rotateServo"); //
        Servo clawPivot = hardwareMap.servo.get("clawPivot"); //


        Servo LArm = hardwareMap.servo.get("LArm"); //

        Servo RArm = hardwareMap.servo.get("RArm");

        LArm.setDirection(Servo.Direction.REVERSE);

        double rotation = 0;
        double arm = 1;
        double pivot = 0;



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            rotateClaw.setPosition(rotation);

            clawPivot.setPosition(pivot);

            RArm.setPosition(arm);
            LArm.setPosition(arm);


            if (rotation != rotateClaw.getPosition()){


                rotateClaw.setPosition(rotation);



            }

            if (rotation >= 1 && rotation > 0){

                rotation = 1;

            }

            else if (rotation <= 0){

                rotation = 0;

            }

            if (arm !=  RArm.getPosition() && arm != LArm.getPosition() ){


                RArm.setPosition(arm);
                LArm.setPosition(arm);


            }

            if (arm >= 1 && arm > 0){

                arm = 1;

            }

            else if (arm <= 0){

                arm = 0;

            }

            if (pivot != clawPivot.getPosition()){


                clawPivot.setPosition(pivot);



            }

            if (pivot >= 1 && pivot > 0){

                pivot = 1;

            }

            else if (pivot <= 0){

                pivot = 0;

            }


            //EMERGENCY MANUAL CLAW

            //open
            if(gamepad2.dpad_left){

                claw.setPosition(0);

            }

            //close
            if (gamepad2.dpad_right){

                claw.setPosition(1);

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

           

            if (gamepad2.right_trigger >= 0.5){

              arm -= 0.02;


            }

            if (gamepad2.left_trigger >= 0.5){

                arm += 0.02;


            }

            if (gamepad2.dpad_down){

                pivot -= 0.02;



            }

            if (gamepad2.dpad_up){

                pivot += 0.02;



            }





            telemetry.addData("Claw Rotation" , rotateClaw.getPosition());
            telemetry.addData("Claw Pivot", clawPivot.getPosition());
            telemetry.addData("Rotate claw" , rotateClaw.getPosition());
            telemetry.addData("Rotation value" , rotation);
            telemetry.addData("claw" , claw.getPosition());
            telemetry.addData("RArm" , RArm.getPosition());
            telemetry.addData("LArm" , LArm.getPosition());

            updateTelemetry(telemetry);


        }
    }
}
