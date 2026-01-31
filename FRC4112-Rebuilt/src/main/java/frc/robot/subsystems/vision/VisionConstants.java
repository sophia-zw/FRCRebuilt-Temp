// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

//THESE ALL NEED TO BE CHANGED ***********************************************************************************************************
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeWelded);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "FRC4112_Cam_1"; // Back Left
    public static String camera1Name = "FRC4112_Cam_3"; // Front
    public static String camera2Name = "FRC4112_Cam_4"; // Back Right

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 = new Transform3d(
            Units.inchesToMeters(-11.448),
            Units.inchesToMeters(9.299),
            Units.inchesToMeters(8.758),
            new Rotation3d(
                    0.0,
                    -Radians.convertFrom(15, Degrees),
                    Math.PI + Radians.convertFrom(31.989, Degrees)));
    public static Transform3d robotToCamera1 = new Transform3d(
            Units.inchesToMeters(5.3),
            Units.inchesToMeters(-7.1),
            Units.inchesToMeters(38.75),
            new Rotation3d(
                    0.0,
                    0.0,
                    Radians.convertFrom(-5, Degrees)));
    public static Transform3d robotToCamera2 = new Transform3d(
            Units.inchesToMeters(-11.458),
            Units.inchesToMeters(-9.309),
            Units.inchesToMeters(8.760),
            new Rotation3d(
                    0.0,
                    -Radians.convertFrom(15, Degrees),
                    Math.PI - Radians.convertFrom(31.989, Degrees)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.15;
    public static double maxZError = 0.5;
    public static final int[] scoringTagIds = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
            0.5, // Camera 0
            1.0, // Camera 1
            0.5, // Camera 2
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}