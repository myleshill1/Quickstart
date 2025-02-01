package pedroPathing.demondogsopmodes.johnsteleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config


@TeleOp
public class PID_RESET extends OpMode {

    private PIDController controller;

    public static double p = 0, i = 0, d = 0;

    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 1425.1 / 180;
    private DcMotorEx pivot;


    @Override
    public void init() {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");


        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }


    @Override
    public void loop() {

        controller.setPID(p, i, d);

        int pivotPos = pivot.getCurrentPosition();


        double pid = controller.calculate(pivotPos, target);

        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;


        pivot.setPower(power);


        telemetry.addData("pivotPos: ", pivotPos);
        telemetry.addData("Target: ", target);

        telemetry.update();

    }

}

