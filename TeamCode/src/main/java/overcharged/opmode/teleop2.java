package overcharged.opmode;


import static overcharged.config.RobotConstants.TAG_SL;
import static overcharged.config.RobotConstants.TAG_T;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

// BUTTON MAP
/// Base Driver
// Right Trigger - Intake on/off
// Left Trigger - Outtake on/off
// Right Bumper - Transfer positions
// Left Bumper - hSlide toggle
//


@Config
@TeleOp(name="teleop backup", group="Teleop")
public class teleop2 extends OpMode {
    RobotMecanum robot;
    double slowPower = 1;
    long startTime;
    boolean hSlideGoBottom = false;
    long intakeTiltDelay;
    boolean intakeDelay = false;
    boolean intakeOn = false;
    boolean clawOpen = true;
    boolean hSlideisOut = false;
    boolean intakeTransfer = false;
    boolean firstLoop = true;
    private DigitalChannel hlimitswitch;
    IntakeMode intakeMode = IntakeMode.OFF;
    boolean latch = true;

    public enum IntakeMode {
        IN,
        OUT,
        OFF;
    }

    @Override
    public void init() {

        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            robot = new RobotMecanum(this, false, false);
            startTime = System.currentTimeMillis();
            hlimitswitch = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
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
        if (firstLoop) {
            robot.intakeTilt.setInit();
            firstLoop = false;
            robot.hslides.in();
        }
        // Clear bulk cache
        robot.clearBulkCache();
        long timestamp = System.currentTimeMillis();

        // Joystick and bumper handling
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Check if left bumper is pressed to enable hslide control
        if (hSlideisOut) {
            // Use the left joystick Y-axis to control hslide movement
            float slidePower = -gamepad1.left_stick_y;  // Invert to match expected joystick behavior

            // Control the hslide movement with the joystick
            if (Math.abs(slidePower) > 0.1) { // Add deadzone check
                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.hslides.hslides.setPower(slidePower);
                hSlideisOut = true;
            } else {
                // Stop the hslides if joystick is not being pushed
                robot.hslides.hslides.setPower(0);
            }
            double frontLeftPower = ((x + rx) / denominator) * slowPower;
            double backLeftPower = ((-x + rx) / denominator) * slowPower;
            double frontRightPower = ((- x - rx) / denominator) * slowPower;
            double backRightPower = ((x - rx) / denominator) * slowPower;

            robot.driveLeftFront.setPower(frontLeftPower);
            robot.driveLeftBack.setPower(backLeftPower);
            robot.driveRightFront.setPower(frontRightPower);
            robot.driveRightBack.setPower(backRightPower);
        } else {
            // Regular robot movement control when left bumper is not pressed
            double frontLeftPower = ((y + x + rx) / denominator) * slowPower;
            double backLeftPower = ((y - x + rx) / denominator) * slowPower;
            double frontRightPower = ((y - x - rx) / denominator) * slowPower;
            double backRightPower = ((y + x - rx) / denominator) * slowPower;

            robot.driveLeftFront.setPower(frontLeftPower);
            robot.driveLeftBack.setPower(backLeftPower);
            robot.driveRightFront.setPower(frontRightPower);
            robot.driveRightBack.setPower(backRightPower);
        }
        if (gamepad1.left_bumper && Button.TRANSFER.canPress(timestamp)) {
            hSlideisOut = !hSlideisOut;
        }

        // Logic for bringing hslides back in
        if (!hlimitswitch.getState() && hSlideGoBottom) {
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hslides.hslides.setPower(-1);
            RobotLog.ii(TAG_SL, "Going down");
        } else if (hlimitswitch.getState() && hSlideGoBottom) {
            robot.hslides.forceStop();
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlideGoBottom = false;
            hSlideisOut = false;
            RobotLog.ii(TAG_SL, "Force stopped");
        }

        // Change intake tilt
        if (gamepad1.right_bumper && Button.TRANSFER.canPress(timestamp)) {//bumper && Button.INTAKEOUT.canPress(timestamp)){
            if (!intakeTransfer) {
                robot.intakeTilt.setTransfer();
                intakeTransfer = true;
            } else {
                robot.intakeTilt.setOut();
                intakeTransfer = false;
            }
        }

        // Intake On and Off (in)
        if (gamepad1.right_trigger > 0.9 && Button.INTAKE.canPress(timestamp)) {//gamepad1.right_bumper && Button.INTAKE.canPress(timestamp)){
            if (intakeMode == teleop2.IntakeMode.OFF||intakeMode == teleop2.IntakeMode.OUT) {
                robot.intake.in();
                intakeMode = teleop2.IntakeMode.IN;
            } else{
                robot.intake.off();
                intakeMode = teleop2.IntakeMode.OFF;
            }
        }
        if(gamepad1.left_trigger > 0.9 && Button.INTAKEOUT.canPress(timestamp)){//bumper && Button.INTAKEOUT.canPress(timestamp)){
            if(intakeMode == teleop2.IntakeMode.OFF|| intakeMode == teleop2.IntakeMode.IN) {
                robot.intake.out();
                intakeMode = teleop2.IntakeMode.OUT;
            }
            else {
                robot.intake.off();
                intakeMode = teleop2.IntakeMode.OFF;
            }
        }

        /*
        if (gamepad2.a && Button.SLIGHT_DOWN.canPress(timestamp)) {
            if (latch) {
                robot.latch.setOut();
                latch = false;
                if (!latch) {
                    robot.latch.setInit();
                    latch = true;
                }
            }
        }*/

        if (gamepad2.a && Button.CLAW.canPress(timestamp)) { // Claw
            if (!clawOpen) {
                robot.claw.setOpen();
                clawOpen = true;
            }
            if (clawOpen) {
                robot.claw.setClose();
                clawOpen = false;
            }
        }

        if (gamepad2.x && Button.DEPOTILT.canPress(timestamp)) { // Depo to Bucket
            robot.clawBigTilt.setBucket();
            robot.clawSmallTilt.setOut();
        }

        if (gamepad2.b && Button.DEPOTILT.canPress(timestamp)) { // Depo to Bucket
            robot.clawBigTilt.setOut();
            robot.depoHslide.setOut();
            robot.clawSmallTilt.setFlat();
        }

        if (gamepad2.y && Button.DEPOTILT.canPress(timestamp)) { // Transfer
            robot.depoHslide.setInit();
            robot.clawBigTilt.setTransfer();
            robot.claw.setOpen();
            robot.clawSmallTilt.setTransfer();
        }

        telemetry.addData("h limit switch: ", hlimitswitch.getState());
        telemetry.addData("driveLF", robot.driveLeftFront.getCurrentPosition());
        telemetry.addData("driveLB", robot.driveLeftBack.getCurrentPosition());
        telemetry.addData("driveRF", robot.driveRightFront.getCurrentPosition());
        telemetry.addData("intake power", robot.intake.intake.getPower());
        telemetry.addData("driveRB", robot.driveRightBack.getCurrentPosition());
        telemetry.addData("hslideOut", robot.hslides.slideIn());
        telemetry.addData("hslidePower", robot.hslides.getPower());
        telemetry.update();
    }
}
