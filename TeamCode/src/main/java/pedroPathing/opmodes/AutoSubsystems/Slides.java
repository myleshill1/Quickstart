package pedroPathing.opmodes.AutoSubsystems;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.ArmFeedforward;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import pedroPathing.opmodes.AutoSubsystems.PIDControllerWrapper;
public class Slides extends Subsystem {
    // BOILERPLATE
    public static final Slides INSTANCE = new Slides();
    private Slides() { }

    // USER CODE
    public MotorGroup motor;
    public MotorEx Lmotor, Rmotor;
    public static double sp = 0.035, si = 0.001, sd = 0.0000001, tolerance = 3;
    public PIDControllerWrapper controller = new PIDControllerWrapper(new PIDController(sp,si,sd),tolerance);

    public String Rslides = "RSlides";
    public String Lslides = "LSlides";

    public Command toZero() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                0.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM

    }

    public Command toSpecimenscore() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                500.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM

    }

    @Override
    public void initialize() {
        Lmotor =  new MotorEx(Lslides);
        Rmotor = new MotorEx(Rslides);

        motor = new MotorGroup(
                Lmotor, Rmotor.reverse()
        );
        Lmotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rmotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lmotor.resetEncoder();
        Rmotor.resetEncoder();
    }
}
