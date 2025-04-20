package pedroPathing.opmodes.AutoSubsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.rowanmcalpin.nextftc.core.control.controllers.Controller;

public class PIDControllerWrapper implements Controller {

    private final PIDController ftcLibController;
    private double setPointTolerance;

    public PIDControllerWrapper(PIDController ftcLibController, double setPointTolerance) {
        this.ftcLibController = ftcLibController;
        this.setPointTolerance = setPointTolerance;
    }

    @Override
    public double getTarget() {
        return ftcLibController.getSetPoint();
    }

    @Override
    public void setTarget(double value) {
        ftcLibController.setSetPoint(value);
    }

    @Override
    public double calculate(double reference) {
        return ftcLibController.calculate(reference);
    }

    @Override
    public void reset() {
        ftcLibController.reset();
    }

    @Override
    public double getSetPointTolerance() {
        return setPointTolerance;
    }

    @Override
    public void setSetPointTolerance(double tolerance) {
        this.setPointTolerance = tolerance;
    }

    @Override
    public boolean atTarget(double reference) {
        return Math.abs(reference - getTarget()) < setPointTolerance;
    }
}
