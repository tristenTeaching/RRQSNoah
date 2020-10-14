package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(group = "advanced")
public class TeleOpAlignWithPoint extends LinearOpMode {

    public static PIDCoefficients headingCoefficients = new PIDCoefficients(0.8, 0, 0);

    enum State {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }

    private State currentState = State.NORMAL_CONTROL;

    private PIDFController headingController = new PIDFController(headingCoefficients);

    private Vector2d targetPosition = new Vector2d(0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        headingController.setOutputBounds(-1, 1);
        headingController.setInputBounds(-Math.PI, Math.PI);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            Pose2d driveDirection = new Pose2d();

            switch (currentState) {
                case NORMAL_CONTROL:
                    if (gamepad1.a) {
                        currentState = State.ALIGN_TO_POINT;
                    }

                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    );
                    break;
                case ALIGN_TO_POINT:
                    if (gamepad1.b) {
                        currentState = State.NORMAL_CONTROL;
                    }

                    // Create a vector from the gamepad x/y inputs
                    // Then, rotate that vector by the inverse of that heading
                    // Converts control to field centric
                    Vector2d input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(-poseEstimate.getHeading());

                    Vector2d difference = targetPosition.minus(poseEstimate.vec());
                    // Obtain the target angle for feedback and derivative for feedforward
                    double theta = difference.angle();

                    // Not technically omega because its power and this is the derivative of atan2
                    double thetaFF = input.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                    headingController.setTargetPosition(theta);

                    double headingInput = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF)
                            * DriveConstants.TRACK_WIDTH;

                    driveDirection = new Pose2d(
                            input.getX(),
                            input.getY(),
                            headingInput
                    );

                    break;
            }

            drive.setWeightedDrivePower(driveDirection);

            headingController.update(poseEstimate.getHeading());
            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
