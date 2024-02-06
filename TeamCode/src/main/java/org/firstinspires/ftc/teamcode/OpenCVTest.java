//package org.firstinspires.ftc.teamcode;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
////@Disabled
//@Autonomous(name="Duck Detector", group="Auto")
//public class OpenCVTest extends LinearOpMode {
//    OpenCvCamera webcam;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        BluePipeLine detector = new BluePipeLine(telemetry);
//        webcam.setPipeline(detector);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//
//
//        });
//
//        waitForStart();
//
//        switch (detector.getLocation()) {
//            case LEFT:
//                //....
//                telemetry.addLine("left");
//                break;
//            case RIGHT:
//                //...
//                telemetry.addLine("right");
//                break;
//            case MIDDLE:
//                //...
//                telemetry.addLine("mid");
//                break;
//
//        }
//    }
//}
//
//
//
