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
import overcharged.components.colorSensor;
import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.pathGeneration.Vector;

///////// TODO:BUTTON MAP
///// Base Driver
/// Intake
// Right Trigger - Intake on/off
// Left Trigger - Outtake on/off
/// hSlides
// Left Bumper - hSlide toggle
// y - force hslide back
/// Others
// Right Bumper - Transfer positions
//
///// Arm Driver
///Others
// a - Claw Open/Close
// Right Trigger - score
// Left Bumper - Specimen Score
/// vSlide controls
// dpad Up - Slides to bucket
// dpad Right - slides to specimen
// dpad Left - slides to wall
// dpad down - reset slides

@Config
@TeleOp(name="sexy smooth teleop", group="Teleop")
public class teleop4 extends OpMode {
    RobotMecanum robot;
    double slowPower = 1;
    long startTime;
    boolean hSlideGoBottom = false;
    long intakeTiltDelay;
    long depoDelay;
    long clawDelay;
    long outDelay;
    long transferDelay;
    long intakeStop;
    int wallStep = 0;
    boolean intakeDelay = false;
    boolean intakeOn = false;
    boolean cDelay = false;
    boolean intDelay = false;
    boolean clawOpen = true;
    boolean hSlideisOut = false;
    boolean latched = true;
    boolean vslideGoBottom = false;
    boolean vslideOut = false;
    boolean intakeTransfer = false;
    boolean firstLoop = true;
    boolean dDelay = false;
    private DigitalChannel hlimitswitch;
    private DigitalChannel vlimitswitch;
    IntakeMode intakeMode = IntakeMode.OFF;
    SlideLength slideLength = SlideLength.IN;
    SlideHeight slideHeight = SlideHeight.DOWN;
    ScoreType score = ScoreType.NONE;
    boolean latch = true;

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
    public enum SlideHeight {
        DOWN,
        WALL,
        MID,
        LOWER,
        HIGH1,
        HIGH2;
    }

    public enum ScoreType {
        BUCKET,
        SPECIMEN,
        NONE
    }

    @Override
    public void init() {

        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            robot = new RobotMecanum(this, false, false);
            startTime = System.currentTimeMillis();
            hlimitswitch = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
            vlimitswitch = hardwareMap.get(DigitalChannel.class, "vlimitswitch");

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
            //robot.intakeTilt.setInit();
            firstLoop = false;
            //robot.hslides.in();
        }
        // Clear bulk cache
        robot.clearBulkCache();
        long timestamp = System.currentTimeMillis();

        // Joystick and bumper handling
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);


        if (slideHeight == SlideHeight.HIGH1){
            slowPower = 0.5;
        } else if (slideHeight == SlideHeight.WALL){
            slowPower = 0.75;
        } else if (slideHeight == SlideHeight.MID){
            slowPower = 0.75;
        } else{
            slowPower = 1;
        }


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
            if (hlimitswitch.getState()) { //huh
                hSlideGoBottom = true;
                hSlideisOut = false;
            }

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
        if (gamepad1.left_bumper && Button.TRANSFER.canPress(timestamp)) { // hSlide mode
            hSlideisOut = !hSlideisOut;
            robot.latch.setOut();
            latched = !latched;
        }

        // Logic for bringing hslides back in
        if (!hlimitswitch.getState() && hSlideGoBottom) {
            robot.latch.setInit();
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hslides.hslides.setPower(-1);
            robot.clawBigTilt.setFlat();
            robot.clawSmallTilt.setTranSeq();
            robot.intakeTilt.setFlat();
            RobotLog.ii(TAG_SL, "Going down");
        } else if (hlimitswitch.getState() && hSlideGoBottom) {
            //robot.hslides.forceStop();
            robot.latch.setInit();
            latched = true;
            robot.hslides.hslides.setPower(0);
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlideisOut = false;
            clawDelay = System.currentTimeMillis();
            cDelay = true;
            hSlideGoBottom = false;
            RobotLog.ii(TAG_SL, "Force stopped");
        }


        // Change intake tilt
        if (gamepad1.right_bumper && Button.TRANSFER.canPress(timestamp)) {//bumper && Button.INTAKEOUT.canPress(timestamp)){
            if (!intakeTransfer) {
                robot.intakeTilt.setTransfer();
                intakeTransfer = true;
                if(hlimitswitch.getState() && latched) {
                    clawDelay = System.currentTimeMillis();
                    cDelay = true;
                }
                waitFor(350);
                robot.intake.out();
                intakeMode = IntakeMode.OUT;
                waitFor(400);
                robot.intake.in();
                intakeMode = IntakeMode.IN;
            } else {
                robot.intakeTilt.setOut();
                intakeTransfer = false;
            }
        }

        // Intake On and Off (in)
        if (gamepad1.right_trigger > 0.9 && Button.INTAKE.canPress(timestamp)) {//gamepad1.right_bumper && Button.INTAKE.canPress(timestamp)){
            if (intakeMode == IntakeMode.OFF||intakeMode == IntakeMode.OUT) {
                robot.intake.in();
                intakeMode = IntakeMode.IN;
            } else{
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }
        }
        if(gamepad1.left_trigger > 0.9 && Button.INTAKEOUT.canPress(timestamp)){//bumper && Button.INTAKEOUT.canPress(timestamp)){
            if(intakeMode == IntakeMode.OFF|| intakeMode == IntakeMode.IN) {
                robot.intake.out();
                intakeMode = IntakeMode.OUT;
            }
            else {
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }
        }
        //H Slides go back
        if(gamepad1.y && Button.TRANSFER.canPress(timestamp)){
            robot.intakeTilt.setTransfer();
            robot.claw.setOpen();
            clawOpen = true;
            hSlideGoBottom = true;
            vslideOut = false;
            slideLength = SlideLength.IN;
            cDelay = true;
            intakeTransfer = true;

        }
        if(intakeTransfer && cDelay && System.currentTimeMillis()- clawDelay>500){
            robot.clawBigTilt.setTransfer();
            robot.clawSmallTilt.setTransfer();
            robot.intakeTilt.setTransfer();
            waitFor(1000);
            clawOpen = false;
            robot.claw.setClose();
            cDelay = false;
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
        }


        //TODO: latch
        /*
        if (gamepad2.x && Button.SLIGHT_DOWN.canPress(timestamp)) {
            if (latch) {
                robot.latch.setOut();
                latch = false;
            }
            if (!latch) {
                robot.latch.setInit();
                latch = true;
            }

        }*/

        if (gamepad2.a && Button.CLAW.canPress(timestamp)) { // claw
            if(!clawOpen) {
                robot.claw.setOpen();
                clawOpen = true;
            }
            else if(clawOpen){
                robot.claw.setClose();
                clawOpen = false;
            }
        }

        if (slideHeight == SlideHeight.HIGH1 && System.currentTimeMillis()-depoDelay>750 &dDelay || slideHeight == SlideHeight.LOWER && System.currentTimeMillis()-depoDelay>550 &dDelay) { // Depo to Bucket
            //robot.claw.setClose();
            //clawOpen = false;
            robot.clawBigTilt.setBucket();
            robot.depoHslide.setInit();
            robot.clawSmallTilt.setOut();
            score = ScoreType.BUCKET;
            dDelay = false;
        }

        if (slideHeight == SlideHeight.MID && System.currentTimeMillis()-depoDelay>750 && dDelay) { // Depo to Specimen
           // robot.claw.setClose();
            //clawOpen = false;
            robot.clawBigTilt.setFlat();
            robot.depoHslide.setOut();
            robot.clawSmallTilt.setFlat();
            score = ScoreType.SPECIMEN;
            dDelay = false;
        }

        if (gamepad2.right_bumper && Button.DEPOTILT.canPress(timestamp)) { // Transfer
            robot.depoHslide.setInit();
            robot.clawBigTilt.setTransfer();
            robot.claw.setOpen();
            clawOpen = true;
            vslideOut = false;
            robot.clawSmallTilt.setTransfer();
            robot.intakeTilt.setTransfer();
        }

        if (gamepad2.left_bumper && Button.BTN_LEVEL2.canPress(timestamp)){
            slideHeight = SlideHeight.LOWER;

            robot.vSlides.moveEncoderTo(robot.vSlides.lower, 1f);
            vslideOut = true;
            dDelay = true;
            depoDelay = System.currentTimeMillis();
        }

        if (gamepad2.right_trigger > 0.9 && Button.CLAW.canPress(timestamp)){ // scoring button
            if(score == ScoreType.BUCKET){
                robot.depoHslide.setInit();
                robot.claw.setClose();
                robot.clawBigTilt.setBucket();
                robot.clawSmallTilt.setOut();
                robot.claw.setOpen();
                clawOpen = true;
            } else if (score == ScoreType.SPECIMEN) {
                robot.claw.setClose();
                robot.clawBigTilt.setOut();
                robot.depoHslide.setOut();
                robot.clawSmallTilt.setFlat();
                robot.claw.setOpen();
                clawOpen = true;
            } else {

            }
        }
        //Bring slides slightly up/ Specimen score
        if(gamepad2.left_trigger>0.9 && Button.HIGH2.canPress(timestamp)){
            slideHeight = SlideHeight.WALL;
            vslideOut = true;
            robot.vSlides.moveEncoderTo(robot.vSlides.mid+300, 1f);
        }
        //TODO: theory vSlide code

        if(gamepad2.dpad_up && Button.HIGH1.canPress(timestamp)){ //vSlides Up to Bucket
            slideHeight = SlideHeight.HIGH1;

            robot.vSlides.moveEncoderTo(robot.vSlides.high1, 1f);
            vslideOut = true;
            dDelay = true;
            depoDelay = System.currentTimeMillis();
        }

        if(gamepad2.dpad_left && Button.BTN_MID.canPress(timestamp)){ // Specimen
            slideHeight = SlideHeight.MID;
            vslideOut = true;
            dDelay = true;
            robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1f);
            depoDelay = System.currentTimeMillis();
        }
        // Pick up from wall sequence
        if(gamepad2.dpad_right && Button.WALL.canPress(timestamp)) {
            slideHeight = SlideHeight.DOWN;
            vslideOut = true;
            wallStep = 0;
            robot.clawSmallTilt.setPosition(robot.clawSmallTilt.MOVE_TO_WALL);
            //robot.claw.setPosition(robot.claw.HALF_CLOSE);
            depoDelay = System.currentTimeMillis();
            //waitFor(1000);
            wallStep++;

           // depoDelay = System.currentTimeMillis();
            //wallStep++;
        }
        if(wallStep==1 && System.currentTimeMillis() - depoDelay > 250){
            robot.intakeTilt.setFlat();
            robot.clawBigTilt.setFlat();
            robot.clawSmallTilt.setTranSeq();
            robot.clawBigTilt.setPosition(robot.clawBigTilt.WALL);
            robot.depoHslide.setInit();

            depoDelay = System.currentTimeMillis();
            wallStep++;
        }
        if(wallStep==2 && System.currentTimeMillis() - depoDelay > 500){
            robot.clawSmallTilt.setPosition(robot.clawSmallTilt.WALL);
            robot.claw.setClose();
            clawOpen = false;
            depoDelay = 0;
            depoDelay = System.currentTimeMillis();
            wallStep++;
        }
        if(wallStep==3 && System.currentTimeMillis() - depoDelay > 350){
            robot.claw.setOpen();
            clawOpen = true;
            wallStep=0;
            depoDelay = 0;
        }


        if(gamepad2.dpad_down && Button.SLIDE_RESET.canPress(timestamp)) { // Slide reset
            slideHeight = SlideHeight.DOWN;
            clawOpen = true;
            vslideGoBottom = true;
            vslideOut = false;
            robot.depoHslide.setInit();
            robot.intakeTilt.setFlat();
            robot.clawBigTilt.setFlat();
            robot.clawSmallTilt.setTranSeq();
            waitFor(400);
            robot.clawSmallTilt.setTransfer();
            robot.clawBigTilt.setTransfer();
            robot.claw.setOpen();
            waitFor(400);
            robot.intakeTilt.setTransfer();
        }
        if(vslideGoBottom){
            slideBottom();
            vslideOut = false;
        }
        if(intakeMode == IntakeMode.IN){
            if(robot.sensorF.getColor() == colorSensor.Color.RED || robot.sensorF.getColor() == colorSensor.Color.YELLOW){
                intakeOn = false;
                intakeMode = IntakeMode.OFF;
                robot.intake.off();
            }
            if(robot.sensorF.getColor() == colorSensor.Color.BLUE){
                intakeOn = false;
                intakeMode = IntakeMode.OUT;
                robot.intake.out();
                intakeDelay = true;
                outDelay = System.currentTimeMillis();
            }
        }
        if(intakeDelay && System.currentTimeMillis()-outDelay>250){
            intakeOn = true;
            intakeMode = IntakeMode.IN;
            robot.intake.in();
            intakeDelay = false;
            outDelay =0;
        }
        robot.drawLed();
        if(robot.sensorF.getColor() == colorSensor.Color.RED){
            robot.ledRedOn(true);
        }
        else{
            robot.ledRedOn(false);
        }
        if(robot.sensorF.getColor() == colorSensor.Color.YELLOW){
            robot.ledYellowOn(true);
        }
        else{
            robot.ledYellowOn(false);
        }
        if(robot.sensorF.getColor() == colorSensor.Color.BLUE){
            robot.ledBlueOn(true);
        }
        else{
            robot.ledBlueOn(false);
        }
       // robot.ledYellowOn(true);


        telemetry.addData("h limit switch: ",   hlimitswitch.getState());
        telemetry.addData("v limit switch: ",   vlimitswitch.getState());
        telemetry.addData("vslideRPower:", robot.vSlides.vSlidesL.getPower());
        telemetry.addData("vslideLPower:", robot.vSlides.vSlidesR.getPower());
        telemetry.addData("vslideLencoder: ", robot.vSlides.vSlidesL.getCurrentPosition());
        telemetry.addData("vslideRencoder: ", robot.vSlides.vSlidesR.getCurrentPosition());
        telemetry.addData("h slide power:", robot.hslides.hslides.getPower());
        telemetry.addData("hslide pos: ", robot.hslides.hslides.getCurrentPosition());
        telemetry.addData("driveLF", robot.driveLeftFront.getCurrentPosition());
        telemetry.addData("driveLB", robot.driveLeftBack.getCurrentPosition());
        telemetry.addData("driveRF", robot.driveRightFront.getCurrentPosition());
        telemetry.addData("intake", robot.intake.intake.getCurrentPosition());
        telemetry.addData("driveRB", robot.driveRightBack.getCurrentPosition());
        telemetry.addData("hslidePower", robot.hslides.getPower());

        telemetry.addData("sensorF color", robot.sensorF.getColor());

        telemetry.addData("sensorF h", robot.sensorF.getHSV()[0]);
        telemetry.addData("sensorF s", robot.sensorF.getHSV()[1]);
        telemetry.addData("sensorF v", robot.sensorF.getHSV()[2]);
        //telemetry.addData("sensorF wavelength", robot.sensorF.getWavelength());

        telemetry.update();
    }

    public void slideBottom() { //Slide bottom
        if (vlimitswitch.getState() && vslideGoBottom) {
            robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setPower(-1);
            robot.vSlides.vSlidesL.setPower(-1);
            RobotLog.ii(TAG_SL, "Going down");
        } else if (!vlimitswitch.getState() && vslideGoBottom) {
            //robot.hslides.forceStop();
            robot.vSlides.vSlidesR.setPower(0);
            robot.vSlides.vSlidesL.setPower(0);
            robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            vslideGoBottom = false;
            RobotLog.ii(TAG_SL, "Force stopped");
        }
    }
    public static void waitFor(int milliseconds) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
            // loop
        }
    }
}