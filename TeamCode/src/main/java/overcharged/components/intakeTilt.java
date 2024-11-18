package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class intakeTilt {
    //public OcServo intakeTilt;
    public OcServo intakeTilt;
    public VoltageSensor intakeVolt;
    public static final float INIT = 110f;//36f;//230f;
    public static final float TRANSFER = 95f;//84f;//36f;//175f;
    public static final float FLAT = 188f;//158f;
    public static final float OUT = 202f;//255f;//52f;
    public static final float MOVE_TO_WALL = 124f;//136f;

    public intakeTilt(HardwareMap hardwareMap) {
        intakeTilt = new OcServo(hardwareMap, "intakeTilt", TRANSFER);
        //intakeTilt = hardwareMap.get(OcServo.class, "intakeTilt");
        //intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        intakeTilt.setPosition(pos);
    }

    public void setInit() { intakeTilt.setPosition(INIT); }

    public void setTransfer() { intakeTilt.setPosition(TRANSFER); }

    public void setFlat() { intakeTilt.setPosition(FLAT); }

    public void setOut() { intakeTilt.setPosition(OUT); }

    //public void getVoltage() { intakeVolt.getVoltage();}
}
