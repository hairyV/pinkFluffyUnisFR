
package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Teleop Drive w/ Nav", group = "Concept")
//@Disabled
public class TeleopDrive extends LinearOpMode
{
    //Instantiate PID controllers for arm and wind motors
    private PIDController armController;
    private PIDController windController;
    //Arm motor PID Constants
    public static double ap = 0.002, ai = 0, ad = 0.0001;
    public static double af = -0.15;
    public static int armDeployTarget = 0;
    //Wind Motor PID Constants
    public static double wp = 0.07, wi = 0, wd = 0.00001;
    public static double wf = 0.02;
    public static int windTarget = 0;

    //Ticks in Degrees for GoBuilda Motors
    private final double ticks_in_degree = 700 / 180.0;

    //HuskyLens Stuff
    private final int READ_PERIOD = 1;
    private HuskyLens frontCam;

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    private DcMotorEx armMotor = null; //Used to control the arm's up and down movement
    private DcMotorEx windMotor = null; //Used to control the arm's in and out movement
    private Servo clawLeft;
    private Servo clawRight;
    private Servo clawUD;

    @Override
    public void runOpMode()
    {

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        windMotor = hardwareMap.get(DcMotorEx.class, "wind");
        frontCam = hardwareMap.get(HuskyLens.class, "frontCam");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawUD = hardwareMap.get(Servo.class, "clawUD");



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        clawLeft.setDirection(Servo.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        windMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        windMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        windMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Make sure HuskyLens is working
        if(!frontCam.knock()) {
            telemetry.addData("-> ", "Problem communicating with " + frontCam.getDeviceName());
        } else {
            telemetry.addData("-> ", frontCam.getDeviceName() + " ready");
        }
        frontCam.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


        //Init PID components
        armController = new PIDController(ap, ai, ad);
        windController = new PIDController(wp, wi, wd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive())
        {
            //telemetry.addData("arm: ", armMotor.getCurrentPosition());
            //telemetry.addData("wind: ", windMotor.getCurrentPosition());

            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
            turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
            moveRobot(drive, strafe, turn);

            //Arm up-down control
            armController.setPID(ap, ai, ad);
            int armPos = armMotor.getCurrentPosition();
            double armPID = armController.calculate(armPos, armDeployTarget);
            double armFF = Math.cos(Math.toRadians(armDeployTarget / ticks_in_degree)) * af;
            double armPower = armPID + armFF;
            armMotor.setPower(armPower);
            if (gamepad1.dpad_up) {
                armDeployTarget = -3000;

            }
            if (gamepad1.dpad_down) {
                armDeployTarget = 0;
            }
            /**
            telemetry.addData("pos: ", armPos);
            telemetry.addData("target: ", armDeployTarget);
            telemetry.update();
             **/
            windController.setPID(wp, wi, wd);
            int windPos  = windMotor.getCurrentPosition();
            double windPID = windController.calculate(windPos, windTarget);
            double windFF = Math.cos(Math.toRadians(windTarget / ticks_in_degree)) * wf;
            double windPower = windPID * windFF;
            windMotor.setPower(windPower);

            if (gamepad1.x) {
                windTarget = -500;
            }
            if (gamepad1.y) {
                windTarget = 0;

            }

            if (gamepad1.a) {
                clawRight.setPosition(0.85);
            } else {
                clawRight.setPosition(0.65);
            }
            if (gamepad1.b) {
                clawLeft.setPosition(0.85);

            } else {
                clawLeft.setPosition(0.65);
            }
            if (gamepad1.left_bumper) {
                clawUD.setPosition(0.47);

            } else {
                clawUD.setPosition(0.98);
            }


            telemetry.addData("pos: ", windMotor.getCurrentPosition());
            telemetry.addData("target: ", windTarget);
            telemetry.update();

        }
    }


    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
