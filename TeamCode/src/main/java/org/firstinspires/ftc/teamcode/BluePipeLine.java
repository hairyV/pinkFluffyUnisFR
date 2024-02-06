
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BluePipeLine extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat(); //Java color codec
    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT
    }
    private Location location = Location.RIGHT;

    static final Rect LEFT_ROI = new Rect (
            new Point (984, 780),
            new Point (703, 476));

    static final Rect MID_ROI = new Rect (
            new Point(1613, 645), //top left
            new Point(1145, 420)); // bottom right

    static final Rect RIGHT_ROI = new Rect (
            new Point (1531, 235),
            new Point (1388, 162));

    static double PERCENT_COLOR_THRESHOLD = 0.2;

    public BluePipeLine(Telemetry t) { telemetry = t;}

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); //converting rgb to hsv codec
        Scalar lowHSV = new Scalar(90, 100, 100); //darker reds
        Scalar highHSV = new Scalar(130, 255, 255); //lighter reds

//        //Scalar lowHSV = new Scalar(90, 100, 100);
//        //Scalar highHSV = new Scalar(130, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat); //HSV to Mat so Java can interpret

        //you can show this

        Mat left = mat.submat(LEFT_ROI);
        Mat middle = mat.submat(MID_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        //blur anything to the right of the "=" sign
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double midValue = Core.sumElems(middle).val[0] / MID_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;



        left.release();
        middle.release();
        right.release();

        telemetry.addData("left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("middle percentage", Math.round(midValue * 100) + "%");
        telemetry.addData("right percentage", Math.round(rightValue * 100) + "%");
        //telemetry.addData("right percentage", Math.round(rightValue * 100) + "%");
        //-----------------------------------------------------------------------

        boolean barcodeLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean barcodeMiddle = midValue > PERCENT_COLOR_THRESHOLD;
        boolean barcodeRight = rightValue > PERCENT_COLOR_THRESHOLD;

        /*

        Control Award image recognition
        Camera can only see the left and middle barcodes.

        case 1: If the left barcode is seen, then the element is in the middle
        case 2: if the middle barcode is seen, then the element is on the left
        case 3: If both the left and middle barcodes are seen, then the element is on the right. (not seen by camera)


        if (barcodeLeft) {
            //left
            location = Location.MIDDLE;
            telemetry.addData("Element Location", "middle");
        } else if (barcodeMiddle) {
            //middle
            location = Location.LEFT;
            telemetry.addData("Element Location", "left");
        } else if (barcodeLeft && barcodeMiddle){
            //right
            location = Location.RIGHT;
            telemetry.addData("Element Location", "right");
        }
        */
        if (barcodeLeft) {
            //left
            location = Location.LEFT;
            telemetry.addData("Element Location", "left");
        } else if (barcodeMiddle) {
            //middle
            location = Location.MIDDLE;
            telemetry.addData("Element Location", "middle");
        } else {
            //right
            location = Location.RIGHT;
            telemetry.addData("Element Location", "right");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        //                              R    G  B
        Scalar noDuckColor = new Scalar(255, 0, 0);
        Scalar duckColor = new Scalar (0 ,255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? duckColor:noDuckColor);
        Imgproc.rectangle(mat, MID_ROI, location == Location.MIDDLE? duckColor:noDuckColor);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? duckColor:noDuckColor);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}

