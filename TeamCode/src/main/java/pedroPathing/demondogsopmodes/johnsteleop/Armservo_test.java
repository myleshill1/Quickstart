package pedroPathing.demondogsopmodes.johnsteleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Armservo_test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {



        Servo LArm = hardwareMap.servo.get("LArm"); //

        Servo RArm = hardwareMap.servo.get("RArm");

        LArm.setDirection(Servo.Direction.REVERSE);


        double arm = 0;




        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            RArm.setPosition(arm);
            LArm.setPosition(arm);

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


            if (gamepad2.right_trigger >= 0.5){

               arm -= 0.02;


            }

            if (gamepad2.left_trigger >= 0.5){

                arm += 0.02;


            }



            telemetry.addData("RArm" , RArm.getPosition());
            telemetry.addData("LArm" , LArm.getPosition());

            updateTelemetry(telemetry);


        }
    }
}
