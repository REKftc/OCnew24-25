package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_T;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.RobotMecanum;
import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.pathGeneration.Vector;

@Config
@TeleOp(name="first tele", group="Teleop")
public class teleop1 extends OpMode {
    RobotMecanum robot;
    double slowPower = 1;
    long startTime;
    private Follower follower;

    private DcMotorEx driveLF;
    private DcMotorEx driveLB;
    private DcMotorEx driveRF;
    private DcMotorEx driveRB;

    private Vector driveVector;
    private Vector headingVector;

    @Override
    public void init() {

        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            robot = new RobotMecanum(this, false, false);
            startTime = System.currentTimeMillis();
            robot.setBulkReadManual();
            //robot.vSlides.vSlidesB.setTargetPositionPIDFCoefficients(21,0,0,0);
        } catch (Exception e){
            RobotLog.ee(TAG_T, "Teleop init failed: " + e.getMessage());
            telemetry.addData("Init Failed", e.getMessage());
            telemetry.update();
        }

        follower = new Follower(hardwareMap);

        driveLF = hardwareMap.get(DcMotorEx.class, "driveLF");
        driveLB = hardwareMap.get(DcMotorEx.class, "driveLB");
        driveRB = hardwareMap.get(DcMotorEx.class, "driveRB");
        driveRF = hardwareMap.get(DcMotorEx.class, "driveRF");

        driveLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();

    }

    @Override
    public void loop() {
        //robot.clearBulkCache();
        follower.setTeleOpMovementVectors(gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.left_stick_y);
        follower.update();
    }
}
