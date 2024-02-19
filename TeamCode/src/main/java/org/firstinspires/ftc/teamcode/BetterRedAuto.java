package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RedPipeLine.Location.LEFT;
import static org.firstinspires.ftc.teamcode.RedPipeLine.Location.MIDDLE;
import static org.firstinspires.ftc.teamcode.RedPipeLine.Location.RIGHT;

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
public class BetterRedAuto extends LinearOpMode {

    private DcMotor climbLeft    = null;  //  Left hanging actuator
    private DcMotor climbRight   = null;  //  Right hanging actuator
    private DcMotorEx armMotor = null; //Used to control the arm's up and down movement
    private DcMotorEx windMotor = null; //Used to control the arm's in and out movement
    private Servo clawUD = null; //Used to control the servo's up and down position
    private Servo clawLeft;
    private Servo clawRight;
    //    private Servo hookLeft; //Left hanging hook
//    private Servo hookRight; //Right hanging hook
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
        TRAJECTORY_1_MID,   // First, follow a splineTo() trajectory
        TRAJECTORY_2_MID,   // Then, follow a lineTo() trajectory
        TRAJECTORY_3_MID,
        TRAJECTORY_1_R,   // First, follow a splineTo() trajectory
        TRAJECTORY_2_R,   // Then, follow a lineTo() trajectory
        TRAJECTORY_3_R,
        WAIT_0,         // Then, we follow another lineTo() trajectory
        WAIT_1,         // Then we're gonna wait a second
        WAIT_2,
        WAIT_3,
        WAIT_4,

        WAIT_5,

        TRAJECTORY_6, // Finally, we're gonna turn again
        WAIT_6, TRAJECTORY_7, WAIT_7, IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(11, -61, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

        climbLeft = hardwareMap.get(DcMotor.class, "climbLeft");
        climbRight = hardwareMap.get(DcMotor.class, "climbRight");
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        windMotor = hardwareMap.get(DcMotorEx.class, "wind");
        clawUD = hardwareMap.get(Servo.class, "clawUD");
        clawLeft = hardwareMap.get(Servo.class,"clawLeft");
        clawRight = hardwareMap.get(Servo.class,"clawRight");
//        hookRight = hardwareMap.get(Servo.class, "hookRight");
//        hookLeft = hardwareMap.get(Servo.class, "hookLeft");

        clawRight.setDirection(Servo.Direction.REVERSE);

//        windMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        windMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        RedPipeLine detector = new RedPipeLine(telemetry);
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




        // Initialize our lift
        Lift lift = new Lift(hardwareMap);

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(17, -40, Math.toRadians(90)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(0, -37, Math.toRadians(90)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(41.5, -36, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .splineToConstantHeading(new Vector2d(3, -57), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-34.5, -57), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-54, -38), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-45, -7), Math.toRadians(180))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(-56,-7,Math.toRadians(180)))
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .lineToLinearHeading(new Pose2d(-45,-7,Math.toRadians(180)))
                .build();

        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .splineToConstantHeading(new Vector2d(15, -7), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(41, -30), Math.toRadians(180))
                .build();

//middle trajectories
        Trajectory trajectory1_MID = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(22, -50, Math.toRadians(90)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(15, -28, Math.toRadians(90)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(41.5, -30, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory trajectory2_MID = drive.trajectoryBuilder(trajectory1_MID.end())
                .splineToConstantHeading(new Vector2d(35, -31), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-13, -31), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-40, -31), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-45, -31), Math.toRadians(180))

                .build();

        Trajectory trajectory3_MID = drive.trajectoryBuilder(trajectory2_MID.end())
                .lineToLinearHeading(new Pose2d(-56,-31,Math.toRadians(180)))
                .build();
        Trajectory trajectory4_MID = drive.trajectoryBuilder(trajectory3_MID.end())
                .lineToLinearHeading(new Pose2d(-45,-31,Math.toRadians(180)))
                .build();
        Trajectory trajectory5_MID = drive.trajectoryBuilder(trajectory4_MID.end())
                .splineToConstantHeading(new Vector2d(15, -31), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(41, -31), Math.toRadians(180))
                .build();



//right trajectories
        Trajectory trajectory1_RIGHT = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(22, -31.5, Math.toRadians(90)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(41.5, -42, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToSplineHeading(new Pose2d(22, -31.5, Math.toRadians(90)), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(41.5, -42, Math.toRadians(180)), Math.toRadians(0))
                .build();

        Trajectory trajectory2_RIGHT = drive.trajectoryBuilder(trajectory1_RIGHT.end())
                .splineToConstantHeading(new Vector2d(3, -60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-32, -60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-32, -38), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-45, -7), Math.toRadians(180))
                .build();

        Trajectory trajectory3_RIGHT = drive.trajectoryBuilder(trajectory2_RIGHT.end())
                .lineToLinearHeading(new Pose2d(-56,-7,Math.toRadians(180)))
                .build();
        Trajectory trajectory4_RIGHT = drive.trajectoryBuilder(trajectory3_RIGHT.end())
                .lineToLinearHeading(new Pose2d(-45,-7,Math.toRadians(180)))
                .build();
        Trajectory trajectory5_RIGHT = drive.trajectoryBuilder(trajectory4_RIGHT.end())
                .splineToConstantHeading(new Vector2d(15, -7), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(41, -30), Math.toRadians(180))
                .build();

        ElapsedTime waitTimer1 = new ElapsedTime();
        ElapsedTime waitTimer2 = new ElapsedTime();




        int pos = 0;
        waitForStart();



        if (isStopRequested()) return;


        currentState = State.WAIT_0;


        waitTimer1.reset();
        waitTimer2.reset();



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



            if(pos == 1) {

                switch (currentState) {

                    case WAIT_0:

                        if (waitTimer1.seconds() >= 1) {
                            currentState = State.TRAJECTORY_1;
                            drive.followTrajectoryAsync(trajectory1);
                        }
                        break;

                    case TRAJECTORY_1:

                        if (waitTimer2.seconds() >= 3.5) {
                            armStage = 1;
                        }
                        if (waitTimer2.seconds() >= 6) {
                            armStage = 2;
                        }
                        if (waitTimer2.seconds() >= 7.25) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.15);


                        }
                        if (waitTimer2.seconds() >= 8) {
                            waitTimer2.reset();
                            currentState = State.WAIT_1;


                        }

                        break;


                    case WAIT_1:

                        if (waitTimer1.seconds() >= 1) {
                            currentState = State.TRAJECTORY_2;
                            drive.followTrajectoryAsync(trajectory2);
                        }

                        break;

                    case TRAJECTORY_2:

                        if (waitTimer2.seconds() >= 1) {
                            armStage = 1;
                        }

                        if (!drive.isBusy()) {
                            currentState = State.WAIT_2;
                            waitTimer1.reset();
                        }
                        break;

                    case WAIT_2:
                        if (waitTimer1.seconds() >= 0.75) {
                            armStage = 4;
                            currentState = State.TRAJECTORY_3;
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
                        if (waitTimer1.seconds() >= .75) {
                            clawLeft.setPosition(0);
                            clawRight.setPosition(0);
                            currentState = State.TRAJECTORY_5;
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
                            drive.followTrajectoryAsync(trajectory4);
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
                            drive.followTrajectoryAsync(trajectory5);
                        }

                        break;

                    case TRAJECTORY_7:
                        if(waitTimer2.seconds() >= 1){
                            clawUD.setPosition(0.7);
                        }
                        if(waitTimer2.seconds() >= 2){
                            clawLeft.setPosition(0);
                            clawRight.setPosition(0);
                        }

                        if (!drive.isBusy()) {
                            currentState = State.WAIT_7;
                            waitTimer1.reset();
                        }

                        break;

                    case WAIT_7:
                        if (waitTimer1.seconds() >= 1) {
                            armStage = 3;
                        }
                        if (waitTimer1.seconds() >= 3.5) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.05);
                        }
                        if (waitTimer1.seconds() >= 4) {
                            clawLeft.setPosition(0);
                            clawRight.setPosition(0);
                        }
                        if (waitTimer1.seconds() >= 4.9) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.15);
                        }
                        if (waitTimer1.seconds() >= 5.4) {
                            clawLeft.setPosition(0);
                            clawRight.setPosition(0);
                        }


                        if (waitTimer1.seconds() >= 6) {
                            armStage = 1;
                        }
                        break;

                    case IDLE:
                        // Do nothing in IDLE
                        // currentState does not change once in IDLE
                        // This concludes the autonomous program


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

                        if (waitTimer2.seconds() >= 3.5) {
                            armStage = 1;
                        }
                        if (waitTimer2.seconds() >= 6) {
                            armStage = 2;
                        }
                        if (waitTimer2.seconds() >= 7.25) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.15);


                        }
                        if (waitTimer2.seconds() >= 8) {
                            waitTimer2.reset();
                            currentState = State.WAIT_1;


                        }

                        break;


                    case WAIT_1:

                        if (waitTimer1.seconds() >= 1) {
                            currentState = State.TRAJECTORY_2;
                            drive.followTrajectoryAsync(trajectory2_MID);
                        }

                        break;

                    case TRAJECTORY_2:

                        if (waitTimer2.seconds() >= 1) {
                            armStage = 1;
                        }

                        if (!drive.isBusy()) {
                            currentState = State.WAIT_2;
                            waitTimer1.reset();
                        }
                        break;

                    case WAIT_2:
                        if (waitTimer1.seconds() >= 0.75) {
                            armStage = 4;
                            currentState = State.TRAJECTORY_3;
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
                        if (waitTimer1.seconds() >= .75) {
                            clawLeft.setPosition(0);
                            clawRight.setPosition(0);
                            currentState = State.TRAJECTORY_5;
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
                            drive.followTrajectoryAsync(trajectory4_MID);
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
                            drive.followTrajectoryAsync(trajectory5_MID);
                        }

                        break;

                    case TRAJECTORY_7:
                        if(waitTimer2.seconds() >= 1){
                            clawUD.setPosition(0.7);
                        }
                        if(waitTimer2.seconds() >= 2){
                            clawLeft.setPosition(0);
                            clawRight.setPosition(0);
                        }

                        if (!drive.isBusy()) {
                            currentState = State.WAIT_7;
                            waitTimer1.reset();
                        }

                        break;

                    case WAIT_7:
                        if (waitTimer1.seconds() >= 1) {
                            armStage = 3;
                        }
                        if (waitTimer1.seconds() >= 3.5) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.05);
                        }
                        if (waitTimer1.seconds() >= 4) {
                            clawLeft.setPosition(0);
                            clawRight.setPosition(0);
                        }
                        if (waitTimer1.seconds() >= 4.9) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.15);
                        }
                        if (waitTimer1.seconds() >= 5.4) {
                            clawLeft.setPosition(0);
                            clawRight.setPosition(0);
                        }


                        if (waitTimer1.seconds() >= 6) {
                            armStage = 1;
                        }
                        break;
                    case IDLE:
                        // Do nothing in IDLE
                        // currentState does not change once in IDLE
                        // This concludes the autonomous program


                        break;
                }









            } else if (pos == 3){
                switch (currentState) {

                    case WAIT_0:

                        if (waitTimer1.seconds() >= 1) {
                            currentState = State.TRAJECTORY_1;
                            drive.followTrajectoryAsync(trajectory1_RIGHT);
                        }
                        break;

                    case TRAJECTORY_1:

                        if (waitTimer2.seconds() >= 3.5) {
                            armStage = 1;
                        }
                        if (waitTimer2.seconds() >= 6) {
                            armStage = 2;
                        }
                        if (waitTimer2.seconds() >= 7.25) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.15);


                        }
                        if (waitTimer2.seconds() >= 8) {
                            waitTimer2.reset();
                            currentState = State.WAIT_1;


                        }

                        break;


                    case WAIT_1:

                        if (waitTimer1.seconds() >= 1) {
                            currentState = State.TRAJECTORY_2;
                            drive.followTrajectoryAsync(trajectory2_RIGHT);
                        }

                        break;

                    case TRAJECTORY_2:

                        if (waitTimer2.seconds() >= 1) {
                            armStage = 1;
                        }

                        if (!drive.isBusy()) {
                            currentState = State.WAIT_2;
                            waitTimer1.reset();
                        }
                        break;

                    case WAIT_2:
                        if (waitTimer1.seconds() >= 0.75) {
                            armStage = 4;
                            currentState = State.TRAJECTORY_3;
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
                        if (waitTimer1.seconds() >= .75) {
                            clawLeft.setPosition(0);
                            clawRight.setPosition(0);
                            currentState = State.TRAJECTORY_5;
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
                            drive.followTrajectoryAsync(trajectory4_RIGHT);
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
                            drive.followTrajectoryAsync(trajectory5_RIGHT);
                        }

                        break;

                    case TRAJECTORY_7:
                        if(waitTimer2.seconds() >= 1){
                            clawUD.setPosition(0.7);
                        }
                        if(waitTimer2.seconds() >= 2){
                            clawLeft.setPosition(0);
                            clawRight.setPosition(0);
                        }

                        if (!drive.isBusy()) {
                            currentState = State.WAIT_7;
                            waitTimer1.reset();
                        }

                        break;

                    case WAIT_7:
                        if (waitTimer1.seconds() >= 1) {
                            armStage = 3;
                        }
                        if (waitTimer1.seconds() >= 3.5) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.05);
                        }
                        if (waitTimer1.seconds() >= 4) {
                            clawLeft.setPosition(0);
                            clawRight.setPosition(0);
                        }
                        if (waitTimer1.seconds() >= 4.9) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.15);
                        }
                        if (waitTimer1.seconds() >= 5.4) {
                            clawLeft.setPosition(0);
                            clawRight.setPosition(0);
                        }


                        if (waitTimer1.seconds() >= 6) {
                            armStage = 1;
                        }
                        break;
                    case IDLE:
                        // Do nothing in IDLE
                        // currentState does not change once in IDLE
                        // This concludes the autonomous program


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
//            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

                windMotor.setTargetPosition(-100);
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
                armDeployTarget = -4100;

                windMotor.setTargetPosition(0);
                windMotor.setPower(1);
                windMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//                clawLeft.setPosition(0);
//                clawRight.setPosition(0);

                clawUD.setPosition(0.98);
            }
            if(armStage == 3) {
                armDeployTarget = -3800;

                windMotor.setTargetPosition(-1800);
                windMotor.setPower(1);
                windMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                clawUD.setPosition(0.9);
            }
            if(armStage == 4) {
                armDeployTarget = -609 ;

                windMotor.setTargetPosition(0);
                windMotor.setPower(1);
                windMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                clawUD.setPosition(0.5);
            }


            // You can add other lift update logic here
        }

        // You can add other methods here, like setters for the armDeployTarget or to change the PID parameters
    }

}