package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RedLeftPipe.Location.LEFT;
import static org.firstinspires.ftc.teamcode.RedLeftPipe.Location.MIDDLE;
import static org.firstinspires.ftc.teamcode.RedLeftPipe.Location.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(group = "advanced")
public class RedLeft extends LinearOpMode {

    private DcMotor climbLeft    = null;  //  Left hanging actuator
    private DcMotor climbRight   = null;  //  Right hanging actuator
    private DcMotorEx armMotor = null; //Used to control the arm's up and down movement
    private DcMotorEx windMotor = null; //Used to control the arm's in and out movement
    private Servo clawUD = null; //Used to control the servo's up and down position
    private Servo clawLeft;
    private Servo clawRight;
    private Servo hookLeft; //Left hanging hook
    private Servo hookRight; //Right hanging hook
    private int armStage = 0;

    private HuskyLens huskyLens;

    private final int READ_PERIOD = 1;

    OpenCvCamera webcam;



    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TRAJECTORY_3,
        TRAJECTORY_4,

        TRAJECTORY_5,

        TRAJECTORY_6,

        TRAJECTORY_7,

        TRAJECTORY_8,



        WAIT_0,         // Then, we follow another lineTo() trajectory
        WAIT_1,         // Then we're gonna wait a second
        WAIT_2,
        WAIT_3,
        WAIT_4,

        WAIT_5,

        WAIT_6,

        WAIT_7,

        WAIT_8,
        // Finally, we're gonna turn again
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(-34, -61, Math.toRadians(90));

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

        clawRight.setDirection(Servo.Direction.REVERSE);

        windMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        windMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        RedLeftPipe detector = new RedLeftPipe(telemetry);
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }


        });

//        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
//
//        int globalX = 0;
//        int globalY = 0;
//
//        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
//
//
//        rateLimit.expire();
//
//        if (!huskyLens.knock()) {
//            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
//        } else {
//            telemetry.addData(">>", "Press start to continue");
//        }
//
//        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
//
//        telemetry.update();



        // Initialize our lift
        Lift lift = new Lift(hardwareMap);

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-40, -28, Math.toRadians(-360)))
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(-25, -28, Math.toRadians(-360)))
                .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(-40, -28, Math.toRadians(-360)))
                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .lineToLinearHeading(new Pose2d(-34, -57, Math.toRadians(180)))
                .build();
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .lineToLinearHeading(new Pose2d(37, -57, Math.toRadians(180)))
                .build();
        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .lineToLinearHeading(new Pose2d(49, -43, Math.toRadians(180)))
                .build();





        Trajectory trajectory1_MID = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-34, -29, Math.toRadians(90)))

                .build();
        Trajectory trajectory2_MID = drive.trajectoryBuilder(trajectory1_MID.end())
                .lineToLinearHeading(new Pose2d(-34, -58, Math.toRadians(180)))

                .build();
        Trajectory trajectory3_MID = drive.trajectoryBuilder(trajectory2_MID.end())
                .lineToLinearHeading(new Pose2d(37, -58, Math.toRadians(180)))
                .build();
        Trajectory trajectory4_MID = drive.trajectoryBuilder(trajectory3_MID.end())
                .lineToLinearHeading(new Pose2d(50, -37, Math.toRadians(180)))
                .build();






        Trajectory trajectory1_RIGHT = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-45, -38, Math.toRadians(90)))
                .build();
        Trajectory trajectory2_RIGHT = drive.trajectoryBuilder(trajectory1_RIGHT.end())
                .lineToLinearHeading(new Pose2d(-34, -58, Math.toRadians(180)))
                .build();
        Trajectory trajectory3_RIGHT = drive.trajectoryBuilder(trajectory2_RIGHT.end())
                .lineToLinearHeading(new Pose2d(37, -58, Math.toRadians(180)))
                .build();
        Trajectory trajectory4_RIGHT = drive.trajectoryBuilder(trajectory3_RIGHT.end())
                .lineToLinearHeading(new Pose2d(50, -24, Math.toRadians(180)))
                .build();
        //drop yellow



        //double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();





        int pos = 0;

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.WAIT_0;


        waitTimer1.reset();



//
        if(detector.getLocation() == LEFT){
            pos = 1;
        }

        if(detector.getLocation() == MIDDLE){
            pos = 2;
        }

        if(detector.getLocation() == RIGHT){
            pos = 3;
        }

        while (opModeIsActive() && !isStopRequested()) {



            if(pos == 3) {

                switch (currentState) {

                    case WAIT_0:
                        if (waitTimer1.seconds() >= 1) {
                            currentState = State.TRAJECTORY_1;
                            drive.followTrajectoryAsync(trajectory1);
                        }
                        break;
                    case TRAJECTORY_1:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_1;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_1:
                        if (waitTimer1.seconds() >= 7) {
                            currentState = State.TRAJECTORY_2;
                            drive.followTrajectoryAsync(trajectory2);
                        }
                        break;
                    case TRAJECTORY_2:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_2;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_2:
                        if (waitTimer1.seconds() >= 0.5) {
                            currentState = State.TRAJECTORY_3;
                            armStage = 1;
                        }
                        break;
                    case TRAJECTORY_3:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_3;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_3:
                        if (waitTimer1.seconds() >= 0.5) {
                            currentState = State.TRAJECTORY_4;
                            drive.followTrajectoryAsync(trajectory3);
                        }
                        break;
                    case TRAJECTORY_4:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_4;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_4:
                        if (waitTimer1.seconds() >= 0.5) {
                            currentState = State.TRAJECTORY_5;
                            drive.followTrajectoryAsync(trajectory4);
                        }

                        break;

                    case TRAJECTORY_5:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_5;
                            waitTimer1.reset();
                        }
                        break;

                    case WAIT_5:
                        if (waitTimer1.seconds() >= 0.5) {
                            currentState = State.TRAJECTORY_6;
                            drive.followTrajectoryAsync(trajectory5);
                        }
                        break;
                    case TRAJECTORY_6:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_6;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_6:
                        if (waitTimer1.seconds() >= 0.5) {
                            currentState = State.TRAJECTORY_7;
                            drive.followTrajectoryAsync(trajectory6);
                        }
                        break;
                    case TRAJECTORY_7:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_7;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_7:
                        if (waitTimer1.seconds() >= 1) {
                            armStage = 2;
                        }
                        if (waitTimer1.seconds() >= 2.25) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.15);


                        }
                        if (waitTimer1.seconds() >= 3.25) {
                            waitTimer1.reset();
                            currentState = State.TRAJECTORY_8;
                        }
                        break;
                    case TRAJECTORY_8:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_8;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_8:
                        if (waitTimer1.seconds() >= 0.5) {
                            armStage = 1;
                        }
                        break;
                }



            } else if (pos == 2){

                switch (currentState) {

                    case WAIT_0:
                        if (waitTimer1.seconds() >= 1) {
                            currentState = State.TRAJECTORY_1;
                            drive.followTrajectoryAsync(trajectory1_MID);
                        }
                        break;
                    case TRAJECTORY_1:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_1;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_1:
                        if (waitTimer1.seconds() >= 0.5) {
                            currentState = State.TRAJECTORY_2;
                            armStage = 1;
                        }
                        break;
                    case TRAJECTORY_2:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_2;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_2:
                        if (waitTimer1.seconds() >= 7) {
                            currentState = State.TRAJECTORY_3;
                            drive.followTrajectoryAsync(trajectory2_MID);
                        }

                        break;
                    case TRAJECTORY_3:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_3;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_3:
                        if (waitTimer1.seconds() >= 0.5) {
                            currentState = State.TRAJECTORY_4;
                            drive.followTrajectoryAsync(trajectory3_MID);
                        }
                        break;
                    case TRAJECTORY_4:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_4;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_4:
                        if (waitTimer1.seconds() >= 0.5) {
                            currentState = State.TRAJECTORY_5;
                            drive.followTrajectoryAsync(trajectory4_MID);
                        }

                        break;

                    case TRAJECTORY_5:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_5;
                            waitTimer1.reset();
                        }
                        break;

                    case WAIT_5:
                        if (waitTimer1.seconds() >= 1) {
                            armStage = 2;
                        }
                        if (waitTimer1.seconds() >= 2.25) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.15);


                        }
                        if (waitTimer1.seconds() >= 3.25) {
                            waitTimer1.reset();
                            currentState = State.TRAJECTORY_6;
                        }
                        break;
                    case TRAJECTORY_6:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_6;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_6:
                        if (waitTimer1.seconds() >= 0.5) {
                            armStage = 1;
                        }
                        break;


                }

            } else if (pos == 1){
                switch (currentState) {

                    case WAIT_0:
                        if (waitTimer1.seconds() >= 1) {
                            currentState = State.TRAJECTORY_1;
                            drive.followTrajectoryAsync(trajectory1_RIGHT);
                        }
                        break;
                    case TRAJECTORY_1:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_1;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_1:
                        if (waitTimer1.seconds() >= 0.5) {
                            currentState = State.TRAJECTORY_2;
                            armStage = 1;
                        }
                        break;
                    case TRAJECTORY_2:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_2;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_2:
                        if (waitTimer1.seconds() >= 7) {
                            currentState = State.TRAJECTORY_3;
                            drive.followTrajectoryAsync(trajectory2_RIGHT);
                        }

                        break;
                    case TRAJECTORY_3:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_3;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_3:
                        if (waitTimer1.seconds() >= 0.5) {
                            currentState = State.TRAJECTORY_4;
                            drive.followTrajectoryAsync(trajectory3_RIGHT);
                        }
                        break;
                    case TRAJECTORY_4:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_4;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_4:
                        if (waitTimer1.seconds() >= 0.5) {
                            currentState = State.TRAJECTORY_5;
                            drive.followTrajectoryAsync(trajectory4_RIGHT);
                        }

                        break;

                    case TRAJECTORY_5:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_5;
                            waitTimer1.reset();
                        }
                        break;

                    case WAIT_5:
                        if (waitTimer1.seconds() >= 1) {
                            armStage = 2;
                        }
                        if (waitTimer1.seconds() >= 2.25) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.15);


                        }
                        if (waitTimer1.seconds() >= 3.25) {
                            waitTimer1.reset();
                            currentState = State.TRAJECTORY_6;
                        }
                        break;
                    case TRAJECTORY_6:
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_6;
                            waitTimer1.reset();
                        }
                        break;
                    case WAIT_6:
                        if (waitTimer1.seconds() >= 0.5) {
                            armStage = 1;
                        }
                        break;


                }
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`


            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("time", waitTimer1.seconds());
            //telemetry.addData("location", detector.getLocation());
            telemetry.addData("pos", pos);
            telemetry.update();
        }
    }

    class Lift {
        private PIDController armController;
        private DcMotorEx armMotor;
        private final double ticks_in_degree = 700 / 180.0;
        private double ap = 0.002, ai = 0, ad = 0.0001;
        private double af = -0.15;
        private int armDeployTarget = -475; // set your initial target


        public Lift(HardwareMap hardwareMap) {
            // Initialize arm motor
            armMotor = hardwareMap.get(DcMotorEx.class, "arm");
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Initialize PID controller for the arm
            armController = new PIDController(ap, ai, ad);
        }

        public void update() {
            // Update PID controller for the arm
            armController.setPID(ap, ai, ad);
            int armPos = armMotor.getCurrentPosition();
            double armPID = armController.calculate(armPos, armDeployTarget);
            double armFF = Math.cos(Math.toRadians(armDeployTarget / ticks_in_degree)) * af;
            double armPower = armPID + armFF;
            armMotor.setPower(armPower);

            if(armStage == 0) {
                armDeployTarget = 0;

                windMotor.setTargetPosition(-70);
                windMotor.setPower(1);
                windMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                clawLeft.setPosition(0);
                clawRight.setPosition(0);

                clawUD.setPosition(0.98);
            }
            if(armStage == 1) {
                armDeployTarget = -475;

                windMotor.setTargetPosition(0);
                windMotor.setPower(1);
                windMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                clawUD.setPosition(0.98);
            }
            if(armStage == 2) {
                armDeployTarget = -4000;

                windMotor.setTargetPosition(0);
                windMotor.setPower(1);
                windMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



                clawUD.setPosition(0.9);
            }
            if(armStage == 3) {
                armDeployTarget = -3800;

                windMotor.setTargetPosition(-1800);
                windMotor.setPower(1);
                windMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                clawUD.setPosition(0.98);
            }

            // You can add other lift update logic here
        }

        // You can add other methods here, like setters for the armDeployTarget or to change the PID parameters
    }

}