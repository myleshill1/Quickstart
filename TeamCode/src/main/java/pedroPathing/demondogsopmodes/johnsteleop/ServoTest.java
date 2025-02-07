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
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        int CurrentColor;

        ColorSensor colorsens = hardwareMap.get(ColorSensor.class, "colorsens");

        colorsens.enableLed(true);



        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {


            CurrentColor = Color.rgb(colorsens.red(), colorsens.green(), colorsens.blue());

            //if (JavaUtil.colorToSaturation(CurrentColor) >= 0.5 && JavaUtil.colorToHue(CurrentColor) > 220 && JavaUtil.colorToHue(CurrentColor) < 260) {
               // gamepad1.rumble(1000);
           // }

           // else
           // {
            //    gamepad1.stopRumble();
            //}
            //if (JavaUtil.colorToValue(CurrentColor) >= 0.05 && JavaUtil.colorToSaturation(CurrentColor) >= 0.2 && JavaUtil.colorToHue(CurrentColor) > 30 && JavaUtil.colorToHue(CurrentColor) < 75)
            //{
            //    gamepad1.rumble(1000);
           // }

           // else
           // {
             //  gamepad1.stopRumble();
           // }
            if (JavaUtil.colorToSaturation(CurrentColor) >= 0.2 && JavaUtil.colorToHue(CurrentColor) >= 300 && JavaUtil.colorToHue(CurrentColor) <= 355) {

                gamepad1.rumble(1000);
            }
            else
            {
               gamepad1.stopRumble();
            }

            //if (JavaUtil.colorToValue(CurrentColor) < 25 && JavaUtil.colorToValue(CurrentColor) > 35 && JavaUtil.colorToSaturation(CurrentColor) <= 0.4 && JavaUtil.colorToHue(CurrentColor) >= 0 && JavaUtil.colorToHue(CurrentColor) <= 355){
             //   gamepad1.stopRumble();
           }












        }
    }
//}
