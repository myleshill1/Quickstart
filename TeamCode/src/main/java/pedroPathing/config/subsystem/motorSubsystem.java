package pedroPathing.config.subsystem;

import static pedroPathing.config.RobotConstants.scorespecimenLiftpos;
import static pedroPathing.config.RobotConstants.zeroLiftpos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.config.RobotConstants;

public class motorSubsystem {

    public DcMotor rightLift, leftLift, pivot;

    public motorSubsystem(HardwareMap hardwareMap) {
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        pivot=  hardwareMap.get(DcMotor.class, "pivot");
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //do stuff here like reversing motors and running with  encoders
        /*
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         */

    }

    /*----------------------LIFT-------------------*/
    public void zeroLift() {
        rightLift.setTargetPosition(zeroLiftpos);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(1);

        leftLift.setTargetPosition(zeroLiftpos);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(1);
    }

    public void specimenscoreLift() {
        rightLift.setTargetPosition(scorespecimenLiftpos);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(1);

        leftLift.setTargetPosition(scorespecimenLiftpos);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(1);
    }



}
