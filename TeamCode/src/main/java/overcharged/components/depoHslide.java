package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class depoHslide {
    //public OcServo intakeTilt;
    public OcServo depoHslide;
    //public VoltageSensor intakeVolt;
    public static final float INIT = 56f;
    // public static final float TRANSFER = 70f;//175f;
    // public static final float FLAT = 171f;//158f;
    public static final float OUT = 0f;


    public depoHslide(HardwareMap hardwareMap) {
        depoHslide = new OcServo(hardwareMap, "depoHslide", INIT);
        //intakeTilt = hardwareMap.get(OcServo.class, "intakeTilt");
        //intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        depoHslide.setPosition(pos);
    }

    public void setInit() { depoHslide.setPosition(INIT); }

    //public void setTransfer() { intakeTilt.setPosition(TRANSFER); }

    // public void setFlat() { intakeTilt.setPosition(FLAT); }

    public void setOut() { depoHslide.setPosition(OUT); }

    //public void getVoltage() { intakeVolt.getVoltage();}
}


