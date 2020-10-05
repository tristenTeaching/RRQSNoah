package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how to create a teleop using just the SampleMecanumDrive class without
 * the need for an external robot class. This will allow you to do some cool things like
 * incorporating live trajectory following in your teleop. Check out TeleOpAgumentedDriving.java for
 * an example of such behavior.
 *
 * This opmode is essentially just LocalizationTest.java with a few additions and comments.
 */
@Config
@Autonomous(group = "advanced")
public class TeleOpDrive extends LinearOpMode {
    // Create weights for velocity x, velocity y, and angular velocity kinematics
    // Mecanum kinematics works in such a way where the x/y direction may not exert torque
    // equally, thus requires a weighting to balance them out.
    // Follow [this link](https://www.chiefdelphi.com/t/paper-mecanum-and-omni-kinematic-and-force-analysis/106153)
    // for further details on the described behavior
    // Weights are arbitrarily set by the user to adjust the behavior to their liking
    // You may leave these at 1
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

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

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()) {
            // Translate gamepad inputs into velocity
            Pose2d baseVel = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            Pose2d vel;
            if (Math.abs(baseVel.getX()) + Math.abs(baseVel.getY()) + Math.abs(baseVel.getHeading()) > 1) {
                // re-normalize the powers according to the weights
                double denom = VX_WEIGHT * Math.abs(baseVel.getX())
                        + VY_WEIGHT * Math.abs(baseVel.getY())
                        + OMEGA_WEIGHT * Math.abs(baseVel.getHeading());
                vel = new Pose2d(
                        VX_WEIGHT * baseVel.getX(),
                        VY_WEIGHT * baseVel.getY(),
                        OMEGA_WEIGHT * baseVel.getHeading()
                ).div(denom);
            } else {
                vel = baseVel;
            }

            // Set drive power directly
            drive.setDrivePower(vel);

            // Update everything. Odometry. Etc.
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
