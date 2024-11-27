package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.RobotMecanum;
import overcharged.drive.SampleMecanumDrive;
import overcharged.linear.util.SelectLinear;

import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.localization.Pose;
import overcharged.pedroPathing.pathGeneration.BezierCurve;
import overcharged.pedroPathing.pathGeneration.BezierLine;
import overcharged.pedroPathing.pathGeneration.BezierPoint;
import overcharged.pedroPathing.pathGeneration.MathFunctions;
import overcharged.pedroPathing.pathGeneration.Path;
import overcharged.pedroPathing.pathGeneration.PathChain;
import overcharged.pedroPathing.pathGeneration.Point;
import overcharged.pedroPathing.util.Timer;

// Main Class
@Autonomous(name = "red bucket", group = "Autonomous")
public class autoRedBucket extends OpMode{

    //stuff
    boolean vslideGoBottom = false;
    boolean hSlideGoBottom = false;

    // Init
    private RobotMecanum robot;
    private DigitalChannel hlimitswitch;
    private DigitalChannel vlimitswitch;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleMecanumDrive drive;
    MultipleTelemetry telems;
    private Timer pathTimer, opmodeTimer, scanTimer, distanceSensorUpdateTimer, distanceSensorDecimationTimer;

    // Other init
    private int pathState;

    // LOCATIONS
    // GUIDE:
    // (0,0) is the corner. (144, 144) is the opposite.

    // TODO: Here are all the Sample Poses
    // Blue side Blue Element Poses
    //
    private Pose blueBlueLeftSample = new Pose(2.5+9.75+10.5, 4*24+1.5);
    private Pose blueBlueMidSample = new Pose(2.5+9.75, 4*24+1.5);
    private Pose blueBlueRightSample = new Pose(2.5, 4*24+1.5);
    // Blue side Neutral Element Poses
    //
    private Pose blueNeutLeftSample = new Pose(144-2.5, 4*24+1.5);
    private Pose blueNeutMidSample = new Pose(144-2.5-9.75, 4*24+1.5);
    private Pose blueNeutRightSample = new Pose(144-2.5-9.75-10.5, 4*24+1.5);
    // Red Side Red Element Poses
    //
    private Pose redRedLeftSample = new Pose(144-2.5-9.75-10.5, 2*24-2.5);
    private Pose redRedMidSample = new Pose(144-2.5-9.75, 2*24-2.5);
    private Pose redRedRightSample = new Pose(144-2.5, 2*24-2.5);
    // Red Side Neutral Element Poses
    //
    private Pose redNeutLeftSample = new Pose(2.5, 2*24-2.5);
    private Pose redNeutMidSample = new Pose(2.5+9.75, 2*24-2.5);
    private Pose redNeutRightSample = new Pose(2.5+9.75+10.5, 2*24-2.5);

    // TODO: Here are the Basket and Observation Zone positions
    // Blue side Left Basket Pose
    private Pose blueLeftBasket = new Pose();
    // Blue side Right Basket Pose
    private Pose blueRightBasket = new Pose();
    // Red side Left Basket Pose
    private Pose redLeftBasket = new Pose();
    // Red side Right Basket Pose
    private Pose redRightBasket = new Pose();

    // OTHER POSES
    private Pose beforeBucket, ready2Score;
    private Pose startPose = new Pose(130, 60, -Math.PI/2);

    private Path firstScore, inchBucket, goSafe, goBack;

    private PathChain preload;

    private Follower follower;

    //TODO: Starting from here are the poses for the paths
    public void firstBucket(){
        beforeBucket = new Pose(120,34,-3*Math.PI/4);
        ready2Score = new Pose(124,27,-3*Math.PI/4);
    }


    //TODO: here are where the paths are defined
    public void buildPaths() {
        firstScore = new Path(new BezierLine(new Point(startPose),new Point(beforeBucket)));
        firstScore.setConstantHeadingInterpolation(-Math.PI/2);

        inchBucket = new Path(new BezierLine(new Point(beforeBucket), new Point(ready2Score)));
        inchBucket.setConstantHeadingInterpolation(-3*Math.PI/4);

        PathChain preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose),new Point(beforeBucket)))
                .setLinearHeadingInterpolation(startPose.getHeading(), beforeBucket.getHeading())
                .addPath(new BezierLine(new Point(beforeBucket), new Point(ready2Score)))
                .setConstantHeadingInterpolation(ready2Score.getHeading())
                .setPathEndTimeoutConstraint(4.0)
                .build();



        goSafe = new Path(new BezierLine(new Point(ready2Score), new Point(beforeBucket)));
        inchBucket.setConstantHeadingInterpolation(-Math.PI/2);
    }




    // TODO: HERE IS WHERE THE MAIN PATH IS
    // Main pathing
    public void autoPath() {
        switch (pathState) {
            // Auto Body
            //
            case 10: // scores initial specimen
                follower.followPath(preload, true);
                setPathState(12);
                break;
            case 101:
                if (follower.getCurrentTValue() > 0.1) {
                    //firstScore.setLinearHeadingInterpolation(startPose.getHeading() - 0.1 * MathFunctions.getTurnDirection(startPose.getHeading(), firstScore.getEndTangent().getTheta()) * MathFunctions.getSmallestAngleDifference(startPose.getHeading(), firstScore.getEndTangent().getTheta()), -3*Math.PI / 4);
                    firstScore.setLinearHeadingInterpolation(-Math.PI/2, -3*Math.PI / 4);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(inchBucket);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()){
                    robot.vSlides.moveEncoderTo(robot.vSlides.high1, 1f);
                    waitFor(600);
                    setPathState(13);
                }
                break;
            case 13:
                robot.clawBigTilt.setBucket();
                robot.depoHslide.setInit();
                robot.clawSmallTilt.setOut();
                waitFor(780);
                robot.claw.setOpen();
                setPathState(14);
                break;
            case 14:
                waitFor(700);
                follower.followPath(goSafe);
                goSafe.setLinearHeadingInterpolation(beforeBucket.getHeading(), 0);
                setPathState(15);
                break;
            case 15:
                if(follower.getCurrentTValue() >0.25){
                    slideBottom();
                    robot.clawBigTilt.setTransfer();
                    robot.clawSmallTilt.setTransfer();
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    robot.latch.setOut();
                    robot.hslides.moveEncoderTo(robot.hslides.PRESET1, 1f);
                    robot.intakeTilt.setOut();
                    robot.intake.in();
                    setPathState(100);
                }
                break;
            case 17:
                follower.followPath(inchBucket);
                inchBucket.setLinearHeadingInterpolation(ready2Score.getHeading(), -3*Math.PI/4);
                setPathState(100);
                break;



            case 100: // EMPTY TEST CASE
                //follower.holdPoint(new BezierPoint(firstScore.getLastControlPoint()), Math.toRadians(-90));
                telems.addLine("CASE 100 - IN TEST CASE!!");
                break;

        }
    }


    // path setter
    public void setPathState(int state){
        pathState = state;
        pathTimer.resetTimer();
        autoPath();
    }

    // Distance Sensor Checker
    public void startDistanceSensorDisconnectDetection(int state) {
    }

    //loop de loop
    @Override
    public void loop() {
        follower.update();
        autoPath();
        telemetry.addLine("TValue: "+follower.getCurrentTValue());
        telemetry.addLine("Path: " + pathState);

        //functions
        if (!hlimitswitch.getState() && hSlideGoBottom) {
            robot.latch.setInit();
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hslides.hslides.setPower(-1);
            RobotLog.ii(TAG_SL, "Going down");
        } else if (hlimitswitch.getState() && hSlideGoBottom) {
            robot.latch.setInit();
            robot.intakeTilt.setTransfer();
            robot.hslides.hslides.setPower(0);
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlideGoBottom = false;
            RobotLog.ii(TAG_SL, "Force stopped");
        }
    }

    // initialize robot
    @Override
    public void init() {

        // Robot things init
        telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        robot = new RobotMecanum(this, true, false);
        drive = new SampleMecanumDrive(hardwareMap);
        pathTimer = new Timer();

        //Pose init
        firstBucket();
        buildPaths();

        //follower init
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        //robot init
        hlimitswitch = hardwareMap.get(DigitalChannel.class, "hlimitswitch");
        vlimitswitch = hardwareMap.get(DigitalChannel.class, "vlimitswitch");
        robot.intakeTilt.setTransfer();
        robot.clawBigTilt.setTransfer();
        robot.clawSmallTilt.setTransfer();
        robot.claw.setClose();
    }

    //loop de loop but initialized
    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        // starts auto paths
        setPathState(10);

        // safety net if auto doesn't start for some reason
        autoPath();
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
    public static void waitFor(int milliseconds) { //Waitor Function
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
            // loop
        }
    }
}

