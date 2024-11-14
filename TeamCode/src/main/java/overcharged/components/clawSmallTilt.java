package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class clawSmallTilt {
    //public OcServo intakeTilt;
    public OcServo clawSmallTilt;
    public VoltageSensor intakeVolt;
    public static final float INIT = 190f;//230f;
    public static final float TRANSFER = 130f;//188f;//183f;//177f;//171f;//175f;
    public static final float FLAT = 78f;
    public static final float SPEC = 149f;//140f;//117f;//158f;
    public static final float BUCKET = 158f;//175f;//52f;
    public static final float WALL = 138f;
    public static final float MOVE_TO_WALL = 97f;

    public clawSmallTilt(HardwareMap hardwareMap) {
        clawSmallTilt = new OcServo(hardwareMap, "clawSmallTilt", TRANSFER);
        //intakeTilt = hardwareMap.get(OcServo.class, "intakeTilt");
        //intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        clawSmallTilt.setPosition(pos);
    }

    public void setInit() { clawSmallTilt.setPosition(INIT); }

    public void setTransfer() { clawSmallTilt.setPosition(TRANSFER); }

    public void setFlat() { clawSmallTilt.setPosition(SPEC); }

    public void setOut() { clawSmallTilt.setPosition(BUCKET); }

    public void setTranSeq() { clawSmallTilt.setPosition(FLAT); }

    //public void getVoltage() { intakeVolt.getVoltage();}
}
