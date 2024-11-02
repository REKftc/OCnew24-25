package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class claw {
    //public OcServo intakeTilt;
    public OcServo claw;
    //public VoltageSensor intakeVolt;
    public static final float INIT = 49f;
    // public static final float TRANSFER = 70f;//175f;
    // public static final float FLAT = 171f;//158f;
    public static final float CLOSE = 49f;
    public static final float OPEN = 49f;


    public claw(HardwareMap hardwareMap) {
        claw = new OcServo(hardwareMap, "claw", INIT);
        //intakeTilt = hardwareMap.get(OcServo.class, "intakeTilt");
        //intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        claw.setPosition(pos);
    }

    public void setInit() { claw.setPosition(INIT); }

    public void setClose() { claw.setPosition(CLOSE); }

    // public void setFlat() { intakeTilt.setPosition(FLAT); }

    public void setOpen() { claw.setPosition(OPEN); }

    //public void getVoltage() { intakeVolt.getVoltage();}
}

