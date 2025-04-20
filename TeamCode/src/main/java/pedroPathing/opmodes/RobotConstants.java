package pedroPathing.opmodes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotConstants {
    public DcMotor FL, FR, BL, BR;
    public Servo Claw, Omni, Wrist, LArm, RArm;
    public PIDController SlideController;

    public static double sp = 0.035, si = 0.001, sd = 0.0000001;

    public static double sf = 0;

    public static int slidesTarget = 0;

    public final double sticks_in_degree = 103.8 / 180;

    public DcMotorEx Rslides, Lslides;

    //==================

    public PIDController PivotController;

    public static double p = 0.02, i = 0.01, d = 0.0003;

    public static double f = 0;

    public static int pivotTarget = 0;

    public final double ticks_in_degree = 2786.2 / 180;
    public DcMotorEx pivot;
}
