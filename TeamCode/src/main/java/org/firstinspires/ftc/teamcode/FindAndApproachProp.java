package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="FindAndApproachProp", group="Autonomous")
public class FindAndApproachProp extends LinearOpMode {

    private OpenCvCamera webcam;
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new ZoneDetectionPipeline());
        webcam.openCameraDevice();
        webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
    }


    private void initMotors() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "Left_front");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "Left_rear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Right_rear");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }


    @Override
    public void runOpMode() {

        initCamera();
        initMotors();

        waitForStart();
        while (opModeIsActive()) {
            String detectedZone = ZoneDetectionPipeline.zone;

            if (detectedZone.equals("middle")) {
                driveForward();
            }
            else if (detectedZone.equals("right")) {
                steerLeft();
            }
            else if (detectedZone.equals("left")) {
                steerRight();
            }
            else {
                stopDriving();
            }

            telemetry.addData("Detected Zone", detectedZone);
            telemetry.update();
        }
    }

    private void driveForward() {
        leftFrontDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);
    }

    private void steerLeft() {
        leftFrontDrive.setPower(0.3);
        rightFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.3);
        rightBackDrive.setPower(0.5);
    }

    private void steerRight() {
        leftFrontDrive.setPower(0.5);
        rightFrontDrive.setPower(0.3);
        leftBackDrive.setPower(0.5);
        rightBackDrive.setPower(0.3);
    }

    private void stopDriving() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }


    public static class ZoneDetectionPipeline extends org.openftc.easyopencv.OpenCvPipeline {

        static String zone = "not found";
        static final int CONSECUTIVE_FRAMES_THRESHOLD = 3;
        static int consecutiveFrames = 0;
        String lastZone = "not found";

        Mat hsvFrame = new Mat();
        Mat mask = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.GaussianBlur(input, input, new Size(5, 5), 0);

            Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerBlue = new Scalar(15, 20, 130);
            Scalar upperBlue = new Scalar(30, 60, 150);

            // Define the HSV range for the red color
            Scalar lowerRed = new Scalar(105, 89, 170);
            Scalar upperRed = new Scalar(129, 120, 215);

            Mat blueMask = new Mat();
            Mat redMask = new Mat();

            Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

            Core.inRange(hsvFrame, lowerRed, upperRed, redMask);

            Core.bitwise_or(blueMask, redMask, mask);
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

            Imgproc.erode(mask, mask, kernel);
            Imgproc.dilate(mask, mask, kernel);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            String detectedZone = "not found";

            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);

                if (rect.area() > 500) {
                    int frameWidth = input.width();
                    int zoneWidth = frameWidth / 3;
                    if (rect.x < zoneWidth) {
                        detectedZone = "left";
                    } else if (rect.x < 2 * zoneWidth) {
                        detectedZone = "middle";
                    } else {
                        detectedZone = "right";
                    }
                }

                contour.release();
            }

            hierarchy.release();

            if (detectedZone.equals(lastZone)) {
                consecutiveFrames++;
            } else {
                consecutiveFrames = 0;
            }

            if (consecutiveFrames > CONSECUTIVE_FRAMES_THRESHOLD) {
                zone = detectedZone;
            }

            lastZone = detectedZone;

            return input;
        }

        @Override
        public void onViewportTapped() {
            hsvFrame.release();
            mask.release();
        }
    }
}