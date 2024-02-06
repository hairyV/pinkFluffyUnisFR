package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
