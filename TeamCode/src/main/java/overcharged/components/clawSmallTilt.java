package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class clawSmallTilt {
    //public OcServo intakeTilt;
    public OcServo clawSmallTilt;
    public VoltageSensor intakeVolt;
    public static final float INIT = 70f;//230f;
    public static final float TRANSFER = 70f;//175f;
    public static final float FLAT = 171f;//158f;
    public static final float OUT = 224f;//52f;


    public clawSmallTilt(HardwareMap hardwareMap) {
        clawSmallTilt = new OcServo(hardwareMap, "clawSmallTilt", INIT);
        //intakeTilt = hardwareMap.get(OcServo.class, "intakeTilt");
        //intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        clawSmallTilt.setPosition(pos);
    }

    public void setInit() { clawSmallTilt.setPosition(INIT); }

    public void setTransfer() { clawSmallTilt.setPosition(TRANSFER); }

    public void setFlat() { clawSmallTilt.setPosition(FLAT); }

    public void setOut() { clawSmallTilt.setPosition(OUT); }

    //public void getVoltage() { intakeVolt.getVoltage();}
}
