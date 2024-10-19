package overcharged.opmode;
import static overcharged.config.RobotConstants.TAG_SL;
import static overcharged.config.RobotConstants.TAG_T;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;
import overcharged.components.Button;
import overcharged.components.RobotMecanum;
import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.pathGeneration.Vector;
@Config
@TeleOp(name="teleop", group="Teleop")
public class teleop extends OpMode {
    RobotMecanum robot;
    double slowPower = 0.9f;
    long startTime;
    boolean hSlideGoBottom = false;
    boolean intakeOn = false;
    boolean hSlideisOut = false;
    boolean intakeTransfer = false;
    private DigitalChannel hlimitswitch;
    boolean firstLoop = true;
    SlideLength slideLength = SlideLength.IN;
    IntakeMode intakeMode = IntakeMode.OFF;
    public enum IntakeMode {
        IN,
        OUT,
        OFF;
    }
    public enum SlideLength {
        IN,
        MID,
        LONG;
    }
    @Override
    public void init() {
        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            robot = new RobotMecanum(this, false, false);
            hlimitswitch = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
            startTime = System.currentTimeMillis();
            robot.setBulkReadManual();
            //robot.vSlides.vSlidesB.setTargetPositionPIDFCoefficients(21,0,0,0);
        } catch (Exception e) {
            RobotLog.ee(TAG_T, "Teleop init failed: " + e.getMessage());
            telemetry.addData("Init Failed", e.getMessage());
            telemetry.update();
        }
    }
    @Override
    public void loop() {
        if(firstLoop) {
            robot.intakeTilt.setInit();
            firstLoop = false;
           // robot.hslides.in();
        }
        robot.clearBulkCache();
        long timestamp = System.currentTimeMillis();
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator) * slowPower;
        double backLeftPower = ((y - x + rx) / denominator) * slowPower;
        double frontRightPower = ((y - x - rx) / denominator) * slowPower;
        double backRightPower = ((y + x - rx) / denominator) * slowPower;
        robot.driveLeftFront.setPower(frontLeftPower);
        robot.driveLeftBack.setPower(backLeftPower);
        robot.driveRightFront.setPower(frontRightPower);
        robot.driveRightBack.setPower(backRightPower);

        if(slideLength==SlideLength.LONG||slideLength==SlideLength.MID){
            slowPower = 0.9f;
        }
        // Bring hSlides out
        if (gamepad1.right_bumper && Button.BTN_HORIZONTAL.canPress(timestamp)) {
            if (slideLength==SlideLength.IN||slideLength == SlideLength.LONG) {
                hSlideisOut = true;
                slideLength = SlideLength.MID;
                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.hslides.moveEncoderTo(300, 0.6f);
            }
            else if (slideLength==SlideLength.MID){
                hSlideisOut = true;
                slideLength = SlideLength.LONG;
                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.hslides.moveEncoderTo(500, 0.6f);
            }

        }

        if (gamepad1.left_bumper && Button.BTN_HORIZONTAL.canPress(timestamp)) {
                hSlideGoBottom = true;
                slideLength = SlideLength.IN;
                robot.intakeTilt.setTransfer();
                intakeTransfer = true;

        }


        // Bring hSlides in
        if (!hlimitswitch.getState() && (hSlideGoBottom)) {// && robot.vSlides.getCurrentPosition() > robot.vSlides.start){//!robot.vSlides.slideReachedBottom()){
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hslides.hslides.setPower(-1);
            RobotLog.ii(TAG_SL, "Going down");
        } else if (hlimitswitch.getState() && (hSlideGoBottom)) {
            robot.hslides.forceStop();
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlideGoBottom = false;
            hSlideisOut = false;
            slideLength=SlideLength.IN;
            RobotLog.ii(TAG_SL, "Force stopped");
        }
        // Change intake tilt
        if (gamepad2.right_bumper && Button.TRANSFER.canPress(timestamp)) {//bumper && Button.INTAKEOUT.canPress(timestamp)){
            if (!intakeTransfer) {
                robot.intakeTilt.setTransfer();
                intakeTransfer = true;
            } else {
                robot.intakeTilt.setOut();
                intakeTransfer = false;
            }
        }
        // Intake On and Off (in)
        if (gamepad2.right_trigger > 0.9 && Button.INTAKE.canPress(timestamp)) {//gamepad1.right_bumper && Button.INTAKE.canPress(timestamp)){
            if (intakeMode == IntakeMode.OFF) {
                robot.intake.in();
                intakeMode = IntakeMode.IN;
            } else{
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }
        }
        if(gamepad2.left_trigger > 0.9 && Button.INTAKEOUT.canPress(timestamp)){//bumper && Button.INTAKEOUT.canPress(timestamp)){
            if(intakeMode == IntakeMode.OFF) {
                robot.intake.out();
                intakeMode = IntakeMode.OUT;
            }
            else {
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }


        }
        //joy stick controls

        if(gamepad2.left_stick_y > 0.1 && !hlimitswitch.getState()) {
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition()-(int)gamepad2.left_stick_y / 2, 0.3f);
            robot.hslides.hslides.setPower(-gamepad2.left_stick_y / 2);
        }
        if(gamepad2.left_stick_y < -0.1 && robot.hslides.hslides.getCurrentPosition() < 500){
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //robot.hslides.moveEncoderTo(robot.hslides.hslides.getCurrentPosition()-(int)gamepad2.left_stick_y / 2, 0.3f);
            robot.hslides.hslides.setPower(-gamepad2.left_stick_y/2);
        }
        telemetry.addData("h limit switch: ",   hlimitswitch.getState());
        telemetry.addData("test:", robot.hslides.hslides.getPower());
        telemetry.addData("test2: ", robot.hslides.hslides.getCurrentPosition());
        telemetry.addData("driveLF", robot.driveLeftFront.getCurrentPosition());
        telemetry.addData("driveLB", robot.driveLeftBack.getCurrentPosition());
        telemetry.addData("driveRF", robot.driveRightFront.getCurrentPosition());
        telemetry.addData("intake", robot.intake.intake.getCurrentPosition());
        telemetry.addData("driveRB", robot.driveRightBack.getCurrentPosition());
        telemetry.addData("hslidePower", robot.hslides.getPower());

        telemetry.update();
    }
}
