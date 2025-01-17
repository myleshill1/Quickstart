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
public class PIDF_Pivot extends OpMode {

    private PIDController controller;

    public static double p = 0, i = 0, d = 0;

    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 5281.1 / 180;
    private DcMotorEx Lpivot;

    private DcMotorEx Rpivot;

    @Override
    public void init() {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry( telemetry , FtcDashboard.getInstance().getTelemetry());

        Lpivot = hardwareMap.get(DcMotorEx.class, "Lpivot");

        Rpivot = hardwareMap.get(DcMotorEx.class, "Rpivot");

        Rpivot.setDirection(DcMotorSimple.Direction.REVERSE);



//        Rpivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Lpivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }


    @Override
    public void loop() {

        controller.setPID(p, i, d);

        int LpivotPos = Lpivot.getCurrentPosition();
        int RpivotPos = Rpivot.getCurrentPosition();

        double pid = controller.calculate(LpivotPos, target);

        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;



        Lpivot.setPower(power);
        Rpivot.setPower(power);

        telemetry.addData("LpivotPos: ", LpivotPos);
        telemetry.addData("Target: ", target);

        telemetry.addData("RpivotPos: ", RpivotPos);
        telemetry.addData("Target: ", target);

        telemetry.update();

    }




}
