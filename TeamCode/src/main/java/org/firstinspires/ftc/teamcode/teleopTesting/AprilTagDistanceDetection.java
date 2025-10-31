package org.firstinspires.ftc.teamcode.teleopTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.util.Size;
@Disabled
@TeleOp(name="AprilTag Distance Detection")
public class AprilTagDistanceDetection extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final int TARGET_TAG_ID = 20; // Blue goal
    private static final double TAG_SIZE = 8.125; // inches

    @Override
    public void runOpMode() {

        // Build the AprilTag library for DECODE
        AprilTagLibrary.Builder libraryBuilder = new AprilTagLibrary.Builder();
        libraryBuilder.addTag(new AprilTagMetadata(20, "Blue Goal", TAG_SIZE, DistanceUnit.INCH));
        libraryBuilder.addTag(new AprilTagMetadata(24, "Red Goal", TAG_SIZE, DistanceUnit.INCH));
        libraryBuilder.addTag(new AprilTagMetadata(21, "Obelisk 1", TAG_SIZE, DistanceUnit.INCH));
        libraryBuilder.addTag(new AprilTagMetadata(22, "Obelisk 2", TAG_SIZE, DistanceUnit.INCH));
        libraryBuilder.addTag(new AprilTagMetadata(23, "Obelisk 3", TAG_SIZE, DistanceUnit.INCH));
        AprilTagLibrary tagLibrary = libraryBuilder.build();

        // Initialize AprilTag processor WITH NEW CALIBRATION
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setLensIntrinsics(485.856, 485.856, 444.996, 236.201)  // NEW fx, fy, cx, cy
                .build();

        // Initialize vision portal with the SAME resolution used for calibration
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))  // MUST match calibration resolution
                .build();

        telemetry.addData("Status", "Initialized - NEW Calibration!");
        telemetry.addData("Target", "Looking for Tag " + TARGET_TAG_ID);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean targetFound = false;

            if (aprilTag.getDetections().size() > 0) {

                for (AprilTagDetection detection : aprilTag.getDetections()) {

                    if (detection.id == TARGET_TAG_ID && detection.metadata != null) {
                        targetFound = true;

                        telemetry.addLine("==== TARGET FOUND ====");
                        telemetry.addData("Tag ID", detection.id);
                        telemetry.addData("Distance", "%.2f inches", detection.ftcPose.y);
                        telemetry.addData("X offset", "%.2f inches", detection.ftcPose.x);
                        telemetry.addData("Z offset", "%.2f inches", detection.ftcPose.z);
                        telemetry.addData("Yaw", "%.1f degrees", detection.ftcPose.yaw);
                        telemetry.addLine();

                    } else if (detection.metadata != null) {
                        telemetry.addData("Other Tag", "ID %d at %.1f inches",
                                detection.id, detection.ftcPose.y);
                    }
                }

            }

            if (!targetFound) {
                telemetry.addData("Status", "Target tag " + TARGET_TAG_ID + " not visible");
            }

            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }
}