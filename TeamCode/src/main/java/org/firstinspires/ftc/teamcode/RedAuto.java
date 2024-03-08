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
public class RedAuto extends LinearOpMode {

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
        TRAJECTORY_1_MID,   // First, follow a splineTo() trajectory
        TRAJECTORY_2_MID,   // Then, follow a lineTo() trajectory
        TRAJECTORY_3_MID,
        TRAJECTORY_1_R,   // First, follow a splineTo() trajectory
        TRAJECTORY_2_R,   // Then, follow a lineTo() trajectory
        TRAJECTORY_3_R,
        WAIT_0,         // Then, we follow another lineTo() trajectory
        WAIT_1,         // Then we're gonna wait a second
        WAIT_2,
        // Finally, we're gonna turn again
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(15, -63, Math.toRadians(90));

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

        //.lineToLinearHeading(new Pose2d(23, -36, Math.toRadians(90)))
//                                .lineToLinearHeading(new Pose2d(44, -43, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(44, -57, Math.toRadians(180)))

        //middle
//                                .lineToLinearHeading(new Pose2d(11, -36, Math.toRadians(90)))
//                                .lineToLinearHeading(new Pose2d(44, -35, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(44, -57, Math.toRadians(180)))

        //left
//                                .splineTo(new Vector2d(13, -40), Math.toRadians(135))
//                                .splineTo(new Vector2d(7, -36), Math.toRadians(135))
//                                .lineToLinearHeading(new Pose2d(44, -28, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(44, -57, Math.toRadians(180)))

        // Let's define our trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(15, -35), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(0, -35), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(49,-32,Math.toRadians(180)))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(44, -57, Math.toRadians(180)))
                .build();

        Trajectory trajectory1_MID = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, -30, Math.toRadians(90)))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2_MID = drive.trajectoryBuilder(trajectory1_MID.end())
                .lineToLinearHeading(new Pose2d(49,-36,Math.toRadians(180)))
                .build();

        Trajectory trajectory3_MID = drive.trajectoryBuilder(trajectory2_MID.end())
                .lineToLinearHeading(new Pose2d(44, -57, Math.toRadians(180)))
                .build();

        Trajectory trajectory1_RIGHT = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(27, -36, Math.toRadians(90)))
                .build();
//                                .splineTo(new Vector2d(13, 40), Math.toRadians(-135))
//                                .splineTo(new Vector2d(7, 36), Math.toRadians(-135))
//                                .lineToLinearHeading(new Pose2d(44, 28, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(44, 57, Math.toRadians(180)))
        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2_RIGHT = drive.trajectoryBuilder(trajectory1_RIGHT.end())
                .lineToLinearHeading(new Pose2d(49, -45, Math.toRadians(180)))
                .build();

        Trajectory trajectory3_RIGHT = drive.trajectoryBuilder(trajectory2_RIGHT.end())
                .lineToLinearHeading(new Pose2d(44, -57, Math.toRadians(180)))
                .build();

        // Define the angle to turn at

        //double turnAngle1 = Math.toRadians(-270);

        // Third trajectory
        // We have to define a new end pose because we can't just call trajectory2.end()
        // Since there was a point turn before that
        // So we just take the pose from trajectory2.end(), add the previous turn angle to it
//        Pose2d newLastPose = trajectory2.end().plus(new Pose2d(0, 0, turnAngle1));
//        Trajectory trajectory3 = drive.trajectoryBuilder(newLastPose)
//                .lineToConstantHeading(new Vector2d(-15, 0))
//                .build();

        // Define a 1.5 second wait time
        //double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Define the angle for turn 2
        //double turnAngle2 = Math.toRadians(720);

        // Our state machine logic
        // You can have multiple switch statements running together for multiple state machines
        // in parallel. This is the basic idea for subsystems and commands.

        // We essentially define the flow of the state machine through this switch statement



        int pos = 0;



//        while(pos == 0 && !isStopRequested()){
//            if (!rateLimit.hasExpired()) {
//                continue;
//            }
//            rateLimit.reset();
//            HuskyLens.Block[] blocks = huskyLens.blocks();
//            telemetry.addData("Block count", blocks.length);
//            for (int i = 0; i < blocks.length; i++) {
//                int colorXValue = blocks[i].x;
//                int colorYValue = blocks[i].y;
//
//                globalY = colorYValue;
//                globalX = colorXValue;
//
//
//                telemetry.addData("Block", blocks[i].toString());
//                telemetry.addData("color X", colorXValue );
//                telemetry.addData("color Y", colorYValue );
//
//
//            }
//
//            if(!((globalX > 0 && globalX < 320) && (globalY > 0 && globalY < 240))){
//                pos = 1;
//            }
//
//            if((globalX > 120 && globalX < 150) && (globalY > 120 && globalY < 150)){
//                pos = 2;//middle
//            }
//
//            if((globalX > 245 && globalX < 275) && (globalY > 145 && globalY < 175)){
//                pos = 3;//right
//            }
//
//            //261,161
//
//            telemetry.update();
//        }
//
//        pos = 2;


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



            if(pos == 1) {

                switch (currentState) {

                    case WAIT_0:

                        if (waitTimer1.seconds() >= 1) {
                            currentState = State.TRAJECTORY_1;
                            drive.followTrajectoryAsync(trajectory1);
                        }
                        break;

                    case TRAJECTORY_1:
                        // Check if the drive class isn't busy
                        // `isBusy() == true` while it's following the trajectory
                        // Once `isBusy() == false`, the trajectory follower signals that it is finished
                        // We move on to the next state
                        // Make sure we use the async follow function
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_1;
                            waitTimer1.reset();
                        }
                        break;

                    case WAIT_1:

                        if (waitTimer1.seconds() >= 1) {
                            armStage = 1;
                        }

                        if (waitTimer1.seconds() >= 2) {
                            currentState = State.TRAJECTORY_2;
                            drive.followTrajectoryAsync(trajectory2);
                        }
                        break;

                    case TRAJECTORY_2:
                        // Check if the drive class is busy following the trajectory
                        // Move on to the next state, TURN_1, once finished

                        if (!drive.isBusy()) {
                            currentState = State.WAIT_2;
                            waitTimer1.reset();
                        }
                        break;

                    case WAIT_2:

                        if (waitTimer1.seconds() >= 1) {
                            armStage = 2;
                        }
                        if (waitTimer1.seconds() >= 2.5) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.15);
                        }

                        if (waitTimer1.seconds() >= 3.5) {
                            armStage = 1;
                        }

                        if (waitTimer1.seconds() >= 4.5) {
                            currentState = State.TRAJECTORY_3;
                            drive.followTrajectoryAsync(trajectory3);
                        }


                        break;
                    case TRAJECTORY_3:
                        // Do nothing in IDLE
                        // currentState does not change once in IDLE
                        // This concludes the autonomous program
                        if (!drive.isBusy()) {
                            currentState = State.IDLE;
                            waitTimer1.reset();
                        }
//
//
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
                            currentState = State.TRAJECTORY_1_MID;
                            drive.followTrajectoryAsync(trajectory1_MID);
                        }
                        break;

                    case TRAJECTORY_1_MID:
                        // Check if the drive class isn't busy
                        // `isBusy() == true` while it's following the trajectory
                        // Once `isBusy() == false`, the trajectory follower signals that it is finished
                        // We move on to the next state
                        // Make sure we use the async follow function
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_1;
                            waitTimer1.reset();
                        }
                        break;

                    case WAIT_1:

                        if (waitTimer1.seconds() >= 1) {
                            armStage = 1;
                        }

                        if (waitTimer1.seconds() >= 2) {
                            currentState = State.TRAJECTORY_2_MID;
                            drive.followTrajectoryAsync(trajectory2_MID);
                        }
                        break;

                    case TRAJECTORY_2_MID:
                        // Check if the drive class is busy following the trajectory
                        // Move on to the next state, TURN_1, once finished

                        if (!drive.isBusy()) {
                            currentState = State.WAIT_2;
                            waitTimer1.reset();
                        }
                        break;

                    case WAIT_2:

                        if (waitTimer1.seconds() >= 1) {
                            armStage = 2;
                        }
                        if (waitTimer1.seconds() >= 2.5) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.15);
                        }

                        if (waitTimer1.seconds() >= 3.5) {
                            armStage = 1;
                        }

                        if (waitTimer1.seconds() >= 4.5) {
                            currentState = State.TRAJECTORY_3_MID;
                            drive.followTrajectoryAsync(trajectory3_MID);
                        }


                        break;
                    case TRAJECTORY_3_MID:
                        // Do nothing in IDLE
                        // currentState does not change once in IDLE
                        // This concludes the autonomous program
                        if (!drive.isBusy()) {
                            currentState = State.IDLE;
                            waitTimer1.reset();
                        }
//
//
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
                            currentState = State.TRAJECTORY_1_R;
                            drive.followTrajectoryAsync(trajectory1_RIGHT);
                        }
                        break;

                    case TRAJECTORY_1_R:
                        // Check if the drive class isn't busy
                        // `isBusy() == true` while it's following the trajectory
                        // Once `isBusy() == false`, the trajectory follower signals that it is finished
                        // We move on to the next state
                        // Make sure we use the async follow function
                        if (!drive.isBusy()) {
                            currentState = State.WAIT_1;
                            waitTimer1.reset();
                        }
                        break;

                    case WAIT_1:

                        if (waitTimer1.seconds() >= 1) {
                            armStage = 1;
                        }

                        if (waitTimer1.seconds() >= 2) {
                            currentState = State.TRAJECTORY_2_R;
                            drive.followTrajectoryAsync(trajectory2_RIGHT);
                        }
                        break;

                    case TRAJECTORY_2_R:
                        // Check if the drive class is busy following the trajectory
                        // Move on to the next state, TURN_1, once finished

                        if (!drive.isBusy()) {
                            currentState = State.WAIT_2;
                            waitTimer1.reset();
                        }
                        break;

                    case WAIT_2:

                        if (waitTimer1.seconds() >= 1) {
                            armStage = 2;
                        }
                        if (waitTimer1.seconds() >= 2.5) {
                            clawLeft.setPosition(0.15);
                            clawRight.setPosition(0.15);
                        }

                        if (waitTimer1.seconds() >= 3.5) {
                            armStage = 1;
                        }

                        if (waitTimer1.seconds() >= 4.5) {
                            currentState = State.TRAJECTORY_3_R;
                            drive.followTrajectoryAsync(trajectory3_RIGHT);
                        }


                        break;
                    case TRAJECTORY_3_R:
                        // Do nothing in IDLE
                        // currentState does not change once in IDLE
                        // This concludes the autonomous program
                        if (!drive.isBusy()) {
                            currentState = State.IDLE;
                            waitTimer1.reset();
                        }
//
//
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



                clawUD.setPosition(0.92);
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