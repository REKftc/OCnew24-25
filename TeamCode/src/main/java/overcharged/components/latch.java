package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class latch {
    //public OcServo intakeTilt;
    public OcServo latch;
    //public VoltageSensor intakeVolt;
    public static final float INIT = 112f;
    // public static final float TRANSFER = 70f;//175f;
    // public static final float FLAT = 171f;//158f;
    public static final float OUT = 150f;


    public latch(HardwareMap hardwareMap) {
        latch = new OcServo(hardwareMap, "latch", INIT);
        //intakeTilt = hardwareMap.get(OcServo.class, "intakeTilt");
        //intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        latch.setPosition(pos);
    }

    public void setInit() { latch.setPosition(INIT); }

    //public void setTransfer() { intakeTilt.setPosition(TRANSFER); }

    // public void setFlat() { intakeTilt.setPosition(FLAT); }

    public void setOut() { latch.setPosition(OUT); }

    //public void getVoltage() { intakeVolt.getVoltage();}
}

