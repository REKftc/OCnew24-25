package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;

//import revAmped.components.Button;

@TeleOp(name = "SwitchTest", group = "Test")
//@Disabled
public class SwitchTest extends LinearOpMode {

    private DigitalChannel digital;

    @Override
    public void runOpMode() throws InterruptedException {
        digital = hardwareMap.get(DigitalChannel.class, "hlimitswitch");

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            telemetry.addData("limitswitch", digital.getState());

            telemetry.update();
            idle();
        }
    }
}
