package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class  StrafeTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    //Instantiate PID controllers for arm and wind motors
    private PIDController armController;
    private PIDController windController;
    //Arm motor PID Constants
    public static double ap = 0.002, ai = 0, ad = 0.0001; //0.002, 0, 0.0001
    public static double af = -0.15; //-0.15
    public static int armDeployTarget = 0;
    //Wind Motor PID Constants
//    public static double wp = 0.07, wi = 0, wd = 0.00001;
//    public static double wf = 0.02;
//    public static int windTarget = 0;

    //Ticks in Degrees for GoBuilda Motors
    private final double ticks_in_degree = 700 / 180.0;

    //HuskyLens Stuff
    private final int READ_PERIOD = 1;
//    private HuskyLens frontCam;

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    private DcMotor climbLeft    = null;  //  Left hanging actuator
    private DcMotor climbRight   = null;  //  Right hanging actuator
    private DcMotorEx armMotor = null; //Used to control the arm's up and down movement
    private DcMotorEx windMotor = null; //Used to control the arm's in and out movement
    private Servo clawUD = null; //Used to control the servo's up and down position
    private Servo clawLeft;
    private Servo clawRight;
    private Servo hookLeft; //Left hanging hook
    private Servo hookRight; //Right hanging hook
    private DistanceSensor dist;

    boolean clampClose = false;
    int armStage = 1;
    ElapsedTime liftTimer = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {

        climbLeft = hardwareMap.get(DcMotor.class, "climbLeft");
        climbRight = hardwareMap.get(DcMotor.class, "climbRight");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        windMotor = hardwareMap.get(DcMotorEx.class, "wind");
        clawUD = hardwareMap.get(Servo.class, "clawUD");
        clawLeft = hardwareMap.get(Servo.class,"clawLeft");
        clawRight = hardwareMap.get(Servo.class,"clawRight");
        hookRight = hardwareMap.get(Servo.class, "hookRight");
        hookLeft = hardwareMap.get(Servo.class, "hookLeft");
//        -
        dist = hardwareMap.get(DistanceSensor.class, "dist");


        clawRight.setDirection(Servo.Direction.REVERSE);


        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        windMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        windMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        windMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Make sure HuskyLens is working
//        if(!frontCam.knock()) {
//            telemetry.addData("-> ", "Problem communicating with " + frontCam.getDeviceName());
//        }
//        else {
//            telemetry.addData("-> ", frontCam.getDeviceName() + " ready");
//        }


        //Init PID components
        armController = new PIDController(ap, ai, ad);
//        windController = new PIDController(wp, wi, wd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armController.setPID(ap, ai, ad);
        int armPos = armMotor.getCurrentPosition();
        double armPID = armController.calculate(armPos, armDeployTarget);
        double armFF = Math.cos(Math.toRadians(armDeployTarget / ticks_in_degree)) * af;
        double armPower = armPID + armFF;
        armMotor.setPower(armPower);


        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();
//
//        armDeployTarget = -475;
//
//        windMotor.setTargetPosition(0);
//        windMotor.setPower(1);
//        windMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        clawUD.setPosition(0.98);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
        //drive.turn(Math.toRadians(90));




        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
