package pedroPathing.demondogsopmodes.johnsteleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config


@TeleOp
public class PID_Slides extends OpMode {

    private PIDController SlideController;

    public static double sp = 0, si = 0, sd = 0;

    public static double sf = 0;

    public static int slidesTarget = 0;

    private final double ticks_in_degree = 	103.8 / 180;
    private DcMotorEx Rslide;
    private DcMotorEx Lslide;


    @Override
    public void init() {

        SlideController = new PIDController(sp, si, sd);
        telemetry = new MultipleTelemetry( telemetry , FtcDashboard.getInstance().getTelemetry());

        Rslide = hardwareMap.get(DcMotorEx.class, "Rslides");
        Lslide = hardwareMap.get(DcMotorEx.class, "Lslides");

        Lslide.setDirection(DcMotorSimple.Direction.REVERSE);



    }


    @Override
    public void loop() {

        SlideController.setPID(sp, si, sd);

        int RslidePos = Rslide.getCurrentPosition();
        int LslidePos = Lslide.getCurrentPosition();


        double Rpid = SlideController.calculate(RslidePos, slidesTarget);
        double Lpid = SlideController.calculate(LslidePos, slidesTarget);

        double Rff = Math.cos(Math.toRadians(slidesTarget / ticks_in_degree)) * sf;
        double Lff = Math.cos(Math.toRadians(slidesTarget / ticks_in_degree)) * sf;

        double Rpower = Rpid + Rff;
        double LPower = Lpid + Lff;



        Rslide.setPower(Rpower);
        Lslide.setPower(LPower);


        telemetry.addData("RslidePos: ", RslidePos);
        telemetry.addData("LslidePos: ", LslidePos);
        telemetry.addData("Target: ", slidesTarget);

        telemetry.update();

    }




}