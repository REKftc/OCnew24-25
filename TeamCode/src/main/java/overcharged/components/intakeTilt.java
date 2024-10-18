package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class intakeTilt {
    public OcServo intakeTilt;
    public VoltageSensor intakeVolt;
    public static final float INIT = 175f;//230f;
    public static final float TRANSFER = 187f;//175f;
    public static final float FLAT = 171f;//158f;
    public static final float OUT = 62f;//52f;


    public intakeTilt(HardwareMap hardwareMap) {
        intakeTilt = new OcServo(hardwareMap, "intakeTilt", INIT);
        intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        intakeTilt.setPosition(pos);
    }

    public void setInit() { intakeTilt.setPosition(INIT); }

    public void setTransfer() { intakeTilt.setPosition(TRANSFER); }

    public void setFlat() { intakeTilt.setPosition(FLAT); }

    public void setOut() { intakeTilt.setPosition(OUT); }

    public void getVoltage() { intakeVolt.getVoltage();}
}
