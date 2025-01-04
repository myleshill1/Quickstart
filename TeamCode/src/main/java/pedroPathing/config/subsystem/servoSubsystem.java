package pedroPathing.config.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import pedroPathing.config.RobotConstants;


public class servoSubsystem {
    private Servo wrist, grab, omni, pivotarmL, pivotarmR;

    public servoSubsystem(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wristServo");
        grab = hardwareMap.get(Servo.class, "clawServo");
        omni = hardwareMap.get(Servo.class, "omniServo");
        pivotarmL = hardwareMap.get(Servo.class, "pivotarmLservo");
        //NOTE: one of these might need to be reversed!
        //pivotarmR.setDirection(Servo.Direction.REVERSE);
        pivotarmR = hardwareMap.get(Servo.class, "pivotarmRservo");

    }


    /*----------------------OPEN AND CLOSE CLAW-------------------*/
    public void closeClaw() {
        grab.setPosition(RobotConstants.closedClaw);
    }

    public void openClaw() {
        grab.setPosition(RobotConstants.openClaw);
    }

    /*---------------------------ROTATE CLAW---------------------*/

    public void RotateClawto0() {
        omni.setPosition(RobotConstants.normalClawRotate);
    }

    public void RotateClawto180() {
        omni.setPosition(RobotConstants.flippedClawRotate);
    }

    /*--------------------------CLAW WRIST----------------------*/

    public void WallspecimenWrist() {
        wrist.setPosition(RobotConstants.pickupspecimenClawpos);
    }

    public void ScorespecimenWrist() {
        wrist.setPosition(RobotConstants.scorespecimenClawpos);
    }

    /*-------------------------PIVOT ARM-----------------------*/

    public void WallspecimenPivotarm() {
        pivotarmL.setPosition(RobotConstants.pickupspecimenPivotpos);
        pivotarmR.setPosition(RobotConstants.pickupspecimenPivotpos);
    }

    public void ScorespecimenPivotarm() {
        pivotarmL.setPosition(RobotConstants.scorespecimenPivotpos);
        pivotarmR.setPosition(RobotConstants.scorespecimenPivotpos);
    }



    /*------------GET POSITION FUNCTION--------------*/
    public double getGrabPosition() {
        return grab.getPosition();
    }


    public double getPivotPosition() {
        return wrist.getPosition();
    }


}
