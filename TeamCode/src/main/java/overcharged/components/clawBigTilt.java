package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class clawBigTilt {
    //public OcServo intakeTilt;
    public OcServo clawBigTilt;
    public VoltageSensor intakeVolt;
    public static final float INIT = 70f;//230f;
    public static final float TRANSFER = 70f;//175f;
    public static final float FLAT = 107f;//158f;
    public static final float OUT = 51f;//52f;
    public static final float WALL = 9f;//52f;
    public static final float BUCKET = 136f;


    public clawBigTilt(HardwareMap hardwareMap) {
        clawBigTilt = new OcServo(hardwareMap, "clawBigTilt", INIT);
        //intakeTilt = hardwareMap.get(OcServo.class, "intakeTilt");
        //intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        clawBigTilt.setPosition(pos);
    }

    public void setInit() { clawBigTilt.setPosition(INIT); }

    public void setTransfer() { clawBigTilt.setPosition(TRANSFER); }

    public void setFlat() { clawBigTilt.setPosition(FLAT); }

    public void setOut() { clawBigTilt.setPosition(OUT); }
    public void setBucket() { clawBigTilt.setPosition(BUCKET); }

    //public void getVoltage() { intakeVolt.getVoltage();}
}
