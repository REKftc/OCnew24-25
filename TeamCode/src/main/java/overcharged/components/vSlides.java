package overcharged.components;

import static overcharged.config.RobotConstants.TAG_H;
import static overcharged.config.RobotConstants.TAG_SL;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

import overcharged.config.RobotConstants;

public class vSlides {

    public final OcMotorEx vSlidesL;
    public final OcMotorEx vSlidesR;
    public OcSwitch switchSlideDown;
    public final List<OcSwitch> switchs = new ArrayList<>();

    public double start;

    public static int autoLevel = 0;

    public static final int START = 0;
    public static final int PRESET1 = 142;
    public static final int OUT = 1000;

    public static double p = 18;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public vSlides(HardwareMap hardwareMap) {
        vSlidesR = new OcMotorEx(hardwareMap, "hslidesR", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        vSlidesL = new OcMotorEx(hardwareMap, "hslidesL", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        vSlidesR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlidesL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlidesR.setTargetPositionPIDFCoefficients(p, i, d, f);
        vSlidesL.setTargetPositionPIDFCoefficients(p, i, d, f);

        String missing = "";
        int numberMissing = 0;
        OcMotorEx slideL = null;
        OcSwitch lswitch2 = null;
        try {
            lswitch2 = new OcSwitch(hardwareMap, "hlimitswitch", true);
            boolean isSwitchNull = switchSlideDown == null ? true : false;
            switchs.add(lswitch2);
            RobotLog.ii(TAG_H, "limitSwitch(isNull)? " + isSwitchNull);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: limitSwitch " + e.getMessage());
            missing = missing + ", hlimitswitch";
            numberMissing++;
            boolean isSwitchNull = switchSlideDown == null ? true : false;
            RobotLog.ii(TAG_H, "limitSwitch(catch)? " + isSwitchNull);
        }
        this.switchSlideDown = lswitch2;
        start = vSlidesR.getCurrentPosition();
        start = vSlidesL.getCurrentPosition();

        RobotLog.ii(TAG_SL, "Initialized the Slide component numberMissing=" + numberMissing + " missing=" + missing);
    }

    public void outF() {
        vSlidesR.setPower(1);
    }

    public void offF() {
        vSlidesL.setPower(0);
    }

    public void inF() {
        vSlidesR.setPower(-1f);
    }

    public void outB() {
        vSlidesL.setPower(1);
    }

    public void offB() {
        vSlidesR.setPower(0);
    }

    public void inB() {
        vSlidesL.setPower(-1f);
    }

    public void moveEncoderTo(int pos, float power) {
        vSlidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlidesR.setTargetPosition(pos);
        vSlidesL.setTargetPosition(pos);
        vSlidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlidesR.setPower(power);
        vSlidesL.setPower(power);
    }

    public boolean slideIn() {
        return switchSlideDown.isTouch() && vSlidesR.getCurrentPosition() <= start;
    }

    public void in(){
        vSlidesL.setPower(-0.7f);
        vSlidesR.setPower(-0.7f);
    }
    public void inAuto(){
        vSlidesL.setPower(-0.9f);
        vSlidesR.setPower(-0.9f);
    }

    public void setPower(float power)
    {
        int cnt = 1;
        //try {
        if (vSlidesR != null) {
            RobotLog.ii(TAG_SL, "Set slide left motor power to " + power);
            vSlidesR.setPower(power);
            if (power == 0f) {
                vSlidesR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            RobotLog.ii(TAG_SL, "Not setting power for motorL");
        }
        cnt = 2;
        if (vSlidesL != null) {
            RobotLog.ii(TAG_SL, "Set slide right motor power to " + power);
            vSlidesL.setPower(power);
            if (power == 0f) {
                vSlidesL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            RobotLog.ii(TAG_SL, "Not setting power for motorR");
        }
    }

    public void reset(OcMotorEx motor) {
        motor.setPower(0f);
        motor.setPower(0f);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.resetPosition();
    }
    public float getPower() {
        return vSlidesR.getPower();
    }

    public float getPowerL() {
        return vSlidesL.getPower();
    }


    public void forceStop() {
        //if (prev_state != State.STOP) {
        RobotLog.ii(TAG_SL, "Force stop the Slide component");
        vSlidesR.setPower(0f);
        vSlidesL.setPower(0f);
    }
}