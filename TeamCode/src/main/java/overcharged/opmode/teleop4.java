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
import overcharged.components.vSlides;
import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.pathGeneration.Vector;

// TODO:BUTTON MAP
/*
        --=BASE DRIVE=--
    (joysticks move the robot)

    *-Slides & Intake-*
    Right Bumper - hSlide mode On/Off
    Right Joystick Y - hSlide forward/back
    Right Trigger - Intake On/Off
    Left Trigger - Outtake On/Off
    Left Bumper - Transfer Up/Down
    y - reset hSlides and transfer

    *-Misc-*
    Touchpad - starts with only allow red, press to rotate between: - Only Red - Red & Yellow - Only Blue - Blue & Yellow -
    PS Button - factory reset

        --=ARM DRIVER=--
    (buttons do stuff yeah)

    *-Scoring-*
    a - claw Open/Close to Grab/Score
    Dpad Up - Higher Bucket
    Dpad Left - Specimen
    Left Bumper - Lower Bucket

    *-Intake-*
    Dpad Right - grab from wall

    *-Transfer-*
    Dpad Down - Reset vSlides & transfer

    *-Misc-*
    x - Slides go Slightly Down
    b - Slides go Slightly Up

 */

@Config
@TeleOp(name="RED sexy smooth teleop", group="Teleop")
public class teleop4 extends OpMode {
    RobotMecanum robot;
    double slowPower = 1;
    long startTime;
    boolean hSlideGoBottom = false;
    long intakeTiltDelay;
    long depoDelay;
    long clawDelay;
    long outDelay;
    long outakeTime;
    long transferDelay;
    long intakeStop;
    int wallStep = 0;
    int resetStep = 0;
    int transferStep = 0;
    int intakeStep = 0;
    int modeCount = 1;
    int tempLocation;
    boolean intakeDelay = false;
    boolean intakeOn = false;
    boolean intTiltDelay = false;
    boolean cDelay = false;
    boolean clawOpen = true;
    boolean hSlideisOut = false;
    boolean latched = true;
    boolean vslideGoBottom = false;
    boolean vslideOut = false;
    boolean vslideManual = false;
    boolean intakeTransfer = true;
    boolean firstLoop = true;
    boolean dDelay = false;
    boolean intakeOutDelay = false;
    boolean moveCount = false;
    boolean sense = false;
    boolean highTransfer = false;
    boolean redSpec = true;
    boolean red = false;
    boolean blueSpec = false;
    boolean blue = false;
    boolean bucketSeq = false;
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
        gamepad1.setLedColor(255,0,0,500);

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
            firstLoop = false;
        }
        // Clear bulk cache
        robot.clearBulkCache();
        long timestamp = System.currentTimeMillis();

        // Joystick and bumper handling
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Slowmode Settings for Heights
        if (slideHeight == SlideHeight.HIGH1){
            slowPower = 0.65;
        } else if (slideHeight == SlideHeight.WALL){
            slowPower = 0.90;
        } else if (slideHeight == SlideHeight.MID){
            slowPower = 0.80;
        } else{
            slowPower = 1;
        }


        // Check if left bumper is pressed to enable hslide control
        if (hSlideisOut) {
            // Use the left joystick Y-axis to control hslide movement
            float slidePower = -gamepad1.right_stick_y;  // Invert to match expected joystick behavior

            // Control the hslide movement with the joystick
            if (Math.abs(slidePower) > 0.1) { // Add deadzone check
                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.hslides.hslides.setPower(slidePower);
                hSlideisOut = true;
            } else {
                // Stop the hslides if joystick is not being pushed
                robot.hslides.hslides.setPower(0);
            }
        }
        if (hSlideGoBottom){
            float failureCheck = -gamepad1.right_stick_y;
            if (Math.abs(failureCheck) > 0.25) { // Add deadzone check
                robot.hslides.setPower(0);
                hSlideGoBottom = false;
                robot.latch.setOut();
                hSlideisOut = true;
            }
        }
        // Regular robot movement control when left bumper is not pressed
        double frontLeftPower = ((y + x + rx) / denominator) * slowPower;
        double backLeftPower = ((y - x + rx) / denominator) * slowPower;
        double frontRightPower = ((y - x - rx) / denominator) * slowPower;
        double backRightPower = ((y + x - rx) / denominator) * slowPower;

        robot.driveLeftFront.setPower(frontLeftPower);
        robot.driveLeftBack.setPower(backLeftPower);
        robot.driveRightFront.setPower(frontRightPower);
        robot.driveRightBack.setPower(backRightPower);

        if (gamepad1.right_bumper && Button.TRANSFER.canPress(timestamp)) { // hSlide mode
            hSlideisOut = !hSlideisOut;
            robot.latch.setOut();
            latched = !latched;
            if(!intakeTransfer) {
                robot.intakeTilt.setTransfer();
                intakeTransfer = true;
            }
        }

        // Logic for bringing hslides back in
        if (!hlimitswitch.getState() && hSlideGoBottom) {
            robot.latch.setInit();
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hslides.hslides.setPower(-1);
            /*
            robot.clawBigTilt.setFlat();
            robot.clawSmallTilt.setTranSeq();
            robot.intakeTilt.setFlat();
             */
            RobotLog.ii(TAG_SL, "Going down");
        } else if (hlimitswitch.getState() && hSlideGoBottom) {
            robot.latch.setInit();
            latched = true;
            robot.intakeTilt.setTransfer();
            robot.hslides.hslides.setPower(0);
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlideisOut = false;
            clawDelay = System.currentTimeMillis();
            gamepad1.rumble(500);
            cDelay = true;
            hSlideGoBottom = false;
            sense = false;
            RobotLog.ii(TAG_SL, "Force stopped");
        }
        if (hlimitswitch.getState() && highTransfer){
            robot.intakeTilt.setTransfer();
            highTransfer = false;
        }



        // mode switcher
        if (gamepad1.touchpad && Button.CYCLE_MODE.canPress(timestamp)) {
            modeCount += 1;
            gamepad1.rumble(250);
            if (2 % modeCount == 0){
                redSpec = false;
                red = true;
                gamepad1.rumble(20,0,500);
            }
            else if (3 % modeCount == 0){
                red = false;
                blueSpec = true;;
                gamepad1.rumble(0,20,500);
            }
            else if (4 % modeCount == 0){
                blueSpec = false;
                blue = true;
                gamepad1.rumble(0,20,500);
            }
            else{
                blue = false;
                redSpec = true;
                gamepad1.rumble(20,0,500);
            }
        }
        if(redSpec){
            gamepad1.setLedColor(255,0,0,250);
        }
        else if (blueSpec){
            gamepad1.setLedColor(0,0,255,250);
        }
        else if (red||blue){
            gamepad1.setLedColor(255,255,0,250);
        }




        // Change intake tilt
        if (gamepad1.left_bumper && Button.TRANSFER.canPress(timestamp)) {//bumper && Button.INTAKEOUT.canPress(timestamp)){
            if (!intakeTransfer) {
                robot.intakeTilt.setTransfer();
                intakeTransfer = true;
                sense = false;
                if(hlimitswitch.getState() && latched) {
                    clawDelay = System.currentTimeMillis();
                    cDelay = true;
                }
                //outakeTime = System.currentTimeMillis();
                //intakeOutDelay = true;
            } else {
                robot.intakeTilt.setFlat();
                if(hSlideisOut) {
                    intakeTiltDelay = System.currentTimeMillis();
                    intTiltDelay = true;
                }
                else{
                    robot.intakeTilt.setOut();
                }
                robot.intake.in();
                intakeMode = IntakeMode.IN;
                sense = true;
                intakeTransfer = false;
            }
        }


        if(intakeOutDelay){
            intakeOutDelay = false;
            robot.intake.in();
            intakeMode = IntakeMode.IN;
            intakeStep = 0;
            intakeStep++;
            outakeTime = System.currentTimeMillis();
        }
        if(intakeStep == 1 && System.currentTimeMillis()-outakeTime>185){
            robot.intake.out();
            intakeMode = IntakeMode.OUT;
            intakeStep++;
            outakeTime = System.currentTimeMillis();
        }
        if(intakeStep == 2 && System.currentTimeMillis()-outakeTime>100){
            robot.intake.in();
            intakeMode = IntakeMode.IN;
            hSlideGoBottom = true;
            intakeStep = 0;
            outakeTime = 0;
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
            robot.intakeTilt.setHigh();
            robot.depoWrist.setIn();
            outakeTime = System.currentTimeMillis();
            intakeOutDelay = true;
            hSlideisOut = false;
            robot.intake.in();
            intakeMode = IntakeMode.IN;
            highTransfer = true;
            intakeTransfer = true;
            robot.claw.setOpen();
            clawOpen = true;
            vslideOut = false;
            slideLength = SlideLength.IN;
            intakeTransfer = true;
        }

        if(intakeTransfer && cDelay && System.currentTimeMillis()-clawDelay>150){ // Transfer System
            cDelay = false;
            robot.depoWrist.setIn();
            robot.clawBigTilt.setTransfer();
            robot.clawSmallTilt.setTransfer();
            transferStep = 0;
            transferStep++;
            clawDelay = System.currentTimeMillis();
        }
        if (transferStep ==1 & System.currentTimeMillis()-clawDelay>80){
            robot.intakeTilt.setTransfer();
            transferStep++;
            clawDelay = System.currentTimeMillis();
        }
        if (transferStep ==2 & System.currentTimeMillis()-clawDelay>100){
            robot.claw.setClose();
            clawOpen = false;
            gamepad2.rumble(750);
            gamepad2.setLedColor(0,255,0,1000);
            transferStep = 0;
            clawDelay = 0;
        }

        if(!intakeTransfer && intTiltDelay && System.currentTimeMillis()- intakeTiltDelay>400){ //delay in submersible
            robot.intakeTilt.setOut();
            intTiltDelay = false;
        }

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

        // Bucket(High & Low) sequence
        if (slideHeight == SlideHeight.HIGH1 && System.currentTimeMillis()-depoDelay>150 &dDelay || slideHeight == SlideHeight.LOWER && System.currentTimeMillis()-depoDelay>150 &dDelay) { // Depo to Bucket
            robot.depoWrist.setOut();
            robot.depoHslide.setInit();
            score = ScoreType.BUCKET;
            bucketSeq = true;
            depoDelay = System.currentTimeMillis();
            dDelay = false;
        }
        if (bucketSeq && slideHeight == SlideHeight.HIGH1 && System.currentTimeMillis()-depoDelay>500 || bucketSeq && slideHeight == SlideHeight.LOWER && System.currentTimeMillis()-depoDelay>350){
            bucketSeq = false;
            depoDelay = 0;
            robot.clawBigTilt.setBucket();
            robot.clawSmallTilt.setOut();
        }


        if (slideHeight == SlideHeight.MID && System.currentTimeMillis()-depoDelay>680 && dDelay) { // Depo to Specimen
            robot.claw.setSpec();
            robot.clawBigTilt.setOut();
            robot.depoHslide.setOut();
            robot.clawSmallTilt.setFlat();
            score = ScoreType.SPECIMEN;
            depoDelay = 0;
            dDelay = false;
        }

        if (gamepad2.right_bumper && Button.DEPOTILT.canPress(timestamp)) { // Arm Driver Force Transfer
            vslideManual = !vslideManual;
        }

        if(vslideManual){
            float vSlidePower = -gamepad2.left_stick_y;
            if (Math.abs(vSlidePower) > 0.1) { // Add deadzone check
                robot.vSlides.vSlidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.vSlides.vSlidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.vSlides.vSlidesL.setPower(vSlidePower);
                robot.vSlides.vSlidesR.setPower(vSlidePower);
                vslideOut = true;
            } else {
                // Stop the hslides if joystick is not being pushed
                robot.hslides.hslides.setPower(0);
            }
        }

        if (gamepad2.left_bumper && Button.BTN_LEVEL2.canPress(timestamp)){ // Lower Bucket
            slideHeight = SlideHeight.LOWER;

            robot.vSlides.moveEncoderTo(robot.vSlides.lower, 1f);
            vslideOut = true;
            dDelay = true;
            depoDelay = System.currentTimeMillis();
        }

        if (gamepad2.right_trigger > 0.9 && Button.CLAW.canPress(timestamp)){ // TODO: decide if this is real
            if(score == ScoreType.BUCKET){
                robot.depoHslide.setInit();
                robot.claw.setClose();
                robot.clawBigTilt.setBucket();
                robot.clawSmallTilt.setOut();
                robot.claw.setOpen();
                clawOpen = true;
            } else if (score == ScoreType.SPECIMEN) {
                robot.depoHslide.setOut();
                robot.claw.setClose();
                robot.clawBigTilt.setOut();
                robot.depoHslide.setOut();
                robot.clawSmallTilt.setFlat();
                waitFor(200);
                robot.claw.setOpen();
                clawOpen = true;
            } else {
                robot.claw.setOpen();
                clawOpen = true;
            }
        }
        //Bring slides slightly up/ Specimen score
        float slight = gamepad2.left_trigger;
        while(slight>0.9){
            vslideOut = true;
            tempLocation = robot.vSlides.vSlidesL.getCurrentPosition();
            robot.vSlides.moveEncoderTo(tempLocation+150, 1f);
            moveCount = true;
        }

        if(gamepad2.dpad_up && Button.HIGH1.canPress(timestamp)){ //vSlides Up to Bucket
            slideHeight = SlideHeight.HIGH1;

            robot.vSlides.moveEncoderTo(robot.vSlides.high1, 1f);
            vslideOut = true;
            dDelay = true;
            depoDelay = System.currentTimeMillis();
        }

        if(gamepad2.dpad_left && Button.BTN_MID.canPress(timestamp)){ // High Specimen
            slideHeight = SlideHeight.MID;
            vslideOut = true;
            dDelay = true;
            robot.vSlides.moveEncoderTo(robot.vSlides.mid, 1f);
            depoDelay = System.currentTimeMillis();
        }

        // Wall
        if(gamepad2.dpad_right && Button.WALL.canPress(timestamp)) {
            slideHeight = SlideHeight.WALL;
            vslideOut = true;
            wallStep = 0;
            robot.claw.setClose();
            clawOpen = false;
            //robot.clawSmallTilt.setPosition(robot.clawSmallTilt.MOVE_TO_WALL);
            depoDelay = System.currentTimeMillis();
            wallStep++;
        }

        // Wall pickup Sequence
        if(wallStep==1 && System.currentTimeMillis() - depoDelay > 220){
            robot.claw.setClose();
            clawOpen = false;
            robot.intakeTilt.setFlat();
            robot.clawBigTilt.setFlat();
            robot.clawSmallTilt.setTranSeq();
            robot.clawBigTilt.setWall();
            robot.depoHslide.setInit();

            depoDelay = System.currentTimeMillis();
            wallStep++;
        }
        if(wallStep==2 && System.currentTimeMillis() - depoDelay > 360){
            robot.claw.setClose();
            clawOpen = false;
            robot.clawSmallTilt.setWall();
            robot.clawBigTilt.setWall();
            depoDelay = System.currentTimeMillis();
            wallStep++;
        }
        if(wallStep==3 && System.currentTimeMillis() - depoDelay > 140){
            robot.claw.setOpen();
            clawOpen = true;
            wallStep=0;
            depoDelay = 0;
        }

        if(gamepad2.dpad_down && Button.SLIDE_RESET.canPress(timestamp)) { // Slide reset
            clawOpen = true;
            //vslideGoBottom = true;
            vslideOut = false;
            robot.depoHslide.setInit();
            if(slideHeight == SlideHeight.DOWN || slideHeight == SlideHeight.WALL || slideHeight == SlideHeight.LOWER) {
                vslideGoBottom = true;
                robot.depoWrist.setIn();
                robot.intakeTilt.setFlat();
                robot.clawBigTilt.setFlat();
                robot.clawSmallTilt.setTranSeq();
                depoDelay = System.currentTimeMillis();
                resetStep++;
            }
            else{
                robot.clawSmallTilt.setTransfer();
                robot.clawBigTilt.setTransfer();
                robot.depoWrist.setIn();
                robot.claw.setOpen();
                robot.intakeTilt.setTransfer();
                slideHeight = SlideHeight.DOWN;
                depoDelay = System.currentTimeMillis();
                dDelay=true;
            }
        }
        if(slideHeight == SlideHeight.DOWN && System.currentTimeMillis()-depoDelay>250 &&dDelay){
            vslideGoBottom = true;
            depoDelay =0;
            dDelay =false;
        }
        if(resetStep==1 && System.currentTimeMillis() - depoDelay > 400){
            robot.clawSmallTilt.setTransfer();
            robot.clawBigTilt.setTransfer();
            robot.claw.setOpen();
            clawOpen = true;

            depoDelay = System.currentTimeMillis();
            resetStep++;
        }
        if(resetStep==2 && System.currentTimeMillis() - depoDelay > 400){
            robot.intakeTilt.setTransfer();
            slideHeight = SlideHeight.DOWN;
            gamepad2.rumble(500);
            resetStep=0;
            depoDelay = 0;
        }



        if(vslideGoBottom){ //Reset vSlide check
            slideBottom();
            vslideOut = false;
        }

        if(gamepad2.x && Button.SLIGHT_UP.canPress(timestamp)){
            if(robot.vSlides.vSlidesL.getCurrentPosition() < robot.vSlides.high1){
                robot.vSlides.moveEncoderTo((int)(robot.vSlides.vSlidesL.getCurrentPosition())+90, 0.9f);
            }
        }


        if(gamepad2.b && Button.SLIGHT_DOWN.canPress(timestamp)){
            if(robot.vSlides.vSlidesL.getCurrentPosition() > 100){
                robot.vSlides.moveEncoderTo((int)(robot.vSlides.vSlidesL.getCurrentPosition())-90, 0.9f);
            }
        }

        //TODO: QoL functions
        if(gamepad1.ps && Button.BTN_REJECT.canPress(timestamp)){ //Force all reset
            gamepad1.rumble(20, 20, 1600);
            gamepad2.rumble(20,20,1600);
            robot.latch.setOut();
            robot.claw.setOpen();
            robot.hslides.moveEncoderTo(600,1f);
            robot.vSlides.moveEncoderTo(robot.vSlides.high2, 1f);
            robot.intakeTilt.setTransfer();
            robot.intake.off();
            robot.depoWrist.setIn();
            robot.clawBigTilt.setTransfer();
            robot.clawSmallTilt.setTransfer();
            robot.depoHslide.setInit();
            waitFor(500);
            robot.latch.setInit();
            hSlideGoBottom = true;
            waitFor(500);
            slideBottom();
            slideHeight = SlideHeight.DOWN;
            clawOpen = true;
            hSlideisOut = false;
        }
        if(gamepad1.share){

        }



        //Intake Color sensor
        if(intakeMode == IntakeMode.IN){
            if (robot.sensorF.getColor() == colorSensor.Color.RED && sense){
                if(red || redSpec){
                    intakeOn = false;
                    intakeMode = IntakeMode.OFF;
                    robot.intake.off();
                }
                else{
                    gamepad1.rumble(750);
                    intakeOn = true;
                    intakeMode = IntakeMode.OUT;
                    robot.intake.out();
                    intakeDelay = true;
                    outDelay = System.currentTimeMillis();
                }
            }
            if (robot.sensorF.getColor() == colorSensor.Color.YELLOW && sense){
                if(blue || red){
                    intakeOn = false;
                    intakeMode = IntakeMode.OFF;
                    robot.intake.off();
                }
                else{
                    gamepad1.rumble(750);
                    intakeOn = true;
                    intakeMode = IntakeMode.OUT;
                    robot.intake.out();
                    intakeDelay = true;
                    outDelay = System.currentTimeMillis();
                }
            }
            /*
            if (robot.sensorF.getColor() == colorSensor.Color.BLUE && sense){
                if(blue || blueSpec){
                    intakeOn = false;
                    intakeMode = IntakeMode.OFF;
                    robot.intake.off();
                    gamepad1.rumble(500);
                }
                else{
                    intakeOn = true;
                    intakeMode = IntakeMode.OUT;
                    robot.intake.out();
                    intakeDelay = true;
                    outDelay = System.currentTimeMillis();
                }
            }

             */


        }

        // Intake Delay
        if(intakeDelay && System.currentTimeMillis()-outDelay>300){
            intakeDelay = false;
            outDelay =0;
            intakeMode = IntakeMode.IN;
            robot.intake.in();
        }

        // LED code
        robot.drawLed();
        if(robot.sensorF.getColor() == colorSensor.Color.RED){
            robot.ledRedOn(true);
            gamepad1.setLedColor(255,0,0,500);;
        }
        else{
            robot.ledRedOn(false);
        }
        if(robot.sensorF.getColor() == colorSensor.Color.YELLOW){
            robot.ledYellowOn(true);
            gamepad1.setLedColor(255,255,0,500);;
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

        /// Telems
        telemetry.addData("intake mode", intakeMode);
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
            vslideManual = false;
            RobotLog.ii(TAG_SL, "Force stopped");
        }
    }
    //TODO: Better delay function
    public static void waitFor(int milliseconds) { //Waitor Function
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
            // loop
        }
    }
}
//早上好中国现在我有冰淇淋
