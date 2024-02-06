//package org.firstinspires.ftc.teamcode.util;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//
///*
// * This is a simple routine to test translational drive capabilities.
// */
//@Config
//@Autonomous(group = "drive")
//public class BlueAuto extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Pose2d startPose = new Pose2d(0,0, Math.toRadians(90));
//        drive.setPoseEstimate(startPose);
//
//        Trajectory traj1 = drive.trajectorySequenceBuilder(startPose)
//
//                .build();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        drive.followTrajectory(trajectory);
//
//        Pose2d poseEstimate = drive.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.update();
//
//        while (!isStopRequested() && opModeIsActive()) ;
//    }
//}
