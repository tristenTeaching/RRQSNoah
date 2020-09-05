package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;

public class MeepMeepTesting {
    public static void main(String[] args) {
        DriveConstraints constraints = new DriveConstraints(
                60.0, 60.0, 0.0,
                Math.toRadians(270.0), Math.toRadians(270.0), 0.0
        );

        DriveConstraints slowConstraints = new DriveConstraints(
                10.0, 60.0, 0.0,
                Math.toRadians(270.0), Math.toRadians(270.0), 0.0
        );

        MeepMeep mm = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_SKYSTONE_LIGHT)
                .setTheme(new ColorSchemeRedLight())
                .setBackgroundAlpha(0.8f)
                .setConstraints(constraints)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
//                                .setReversed(true)
//                                .splineTo(new Vector2d(36, 36), Math.toRadians(0))
//                                .addDisplacementMarker(0.5, 15, () -> {})
//                                .addSpatialMarker(new Vector2d(36, 18), () -> {})
//                                .splineTo(new Vector2d(72, 0), Math.toRadians(0))
//                                .addDisplacementMarker(25, () -> {})
//                                .build()
//                )
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-40, -40, Math.toRadians(90)))
//                                .splineTo(new Vector2d(-20, -20), Math.toRadians(0))
//                                .turn(Math.toRadians(45))
//                                .waitSeconds(0.5)
//                                .turn(Math.toRadians(45))
//                                .addDisplacementMarker(() -> {})
//                                .setConstraints(slowConstraints)
//                                .splineTo(new Vector2d(0, -30), Math.toRadians(90))
//                                .setConstraints(constraints)
//                                .addDisplacementMarker(() -> {})
//                                .waitSeconds(1)
//                                .lineTo(new Vector2d(20, -30))
//                                .turn(Math.toRadians(180))
//                                .lineTo(new Vector2d(30, 20))
//                                .turn(Math.toRadians(-90))
//                                .addSpatialMarker(new Vector2d(30, -10), () -> {} )
//                                .build()
//                )
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(270)))
//                                .setReversed(true)
//                                .splineTo(new Vector2d(-24, -36), Math.toRadians(180))
//                                .setReversed(false)
//                                .addTemporalMarker(3, () -> {})
//                                .lineTo(new Vector2d(12.0, -36.0))
//                                .addDisplacementMarker(() -> {})
//                                .splineTo(new Vector2d(48.0, -36.0), Math.toRadians(90))
//                                .build()
//                )
//                .followTrajectorySequence(drive ->
//                                drive.trajectorySequenceBuilder(new Pose2d())
//                                        .splineTo(new Vector2d(20, 20), Math.toRadians(0))
//                                        .setReversed(true)
//                                        .splineTo(new Vector2d(0, 40), Math.toRadians(180), slowConstraints)
//                                        .addDisplacementMarker(() -> {})
//                                        .setReversed(false)
//                                        .splineTo(new Vector2d(20, 60), Math.toRadians(180))
//                                        .build()
//                )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d())
                                .splineTo(new Vector2d(20, 20), Math.toRadians(0))
                                .addDisplacementMarker(() -> {})
                                .splineTo(new Vector2d(40, 40), Math.toRadians(45), slowConstraints)
                                .addTemporalMarker(0.5, 0.5, () -> {})
                                .build())
                .start();
    }
}
