// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.VisionPoseEstimator.VisionPoseEstimatorConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Frequency simNotifierFrequency = Hertz.of(200);
  public static final CANBus subsystemsCANBus = new CANBus("subsystems");
  public static final CANBus swerveCANBus = new CANBus("canivore");

  public static final boolean isProfiling = false;

  public static class Ports {
    public static final int driverController = 0;
  }

  public static class FieldConstants {
    public static final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);


    public static final Translation2d blueHub =
        new Translation2d(
            tagLayout.getTagPose(26).get().getX() + Units.inchesToMeters(47.0) / 2.0,
            tagLayout.getFieldWidth() / 2.0);
            
    public static final Translation2d redHub =
        blueHub.rotateAround(
            new Translation2d(tagLayout.getFieldLength() / 2.0, tagLayout.getFieldWidth() / 2.0),
            Rotation2d.k180deg);

    
    // update
    public static final Rectangle2d blueBumpZone = new Rectangle2d(Pose2d.kZero, 1, 1);
    public static final Rectangle2d redBumpZone = new Rectangle2d(Pose2d.kZero, 1, 1);

    public static final Rectangle2d blueAllianceZone = new Rectangle2d(Pose2d.kZero, 1, 1);
    public static final Rectangle2d redAllianceZone = new Rectangle2d(Pose2d.kZero, 1, 1);

    public static final Rectangle2d neutralZone = new Rectangle2d(Pose2d.kZero, 1, 1);

    // uncomment if using the test tag layout
    // public static final AprilTagFieldLayout tagLayout;

    // static {
    //   try {
    //     tagLayout =
    //         new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/test-tag-layout.json");
    //   } catch (Exception e) {
    //     throw new RuntimeException(e);
    //   }
    // }
  }

  public static class MotorConstants {
    public static final DCMotor krakenX60 =
        new DCMotor(12, 7.16, 374.4, 3, Units.rotationsPerMinuteToRadiansPerSecond(6065), 1);

    public static final DCMotor krakenX44 =
        new DCMotor(12, 4.11, 279.1, 3, Units.rotationsPerMinuteToRadiansPerSecond(7758), 1);
  }

  public static class VisionConstants {
    public static final double singleTagStdDevsScaler = 5;

    public static final double ambiguityThreshold = 0.2;

    public static final double xBoundMargin = 0.01;
    public static final double yBoundMargin = 0.01;
    public static final double zBoundMargin = 0.01;

    public static final VisionPoseEstimatorConstants leftArducam =
        new VisionPoseEstimatorConstants(
            "left-arducam",
            new Transform3d(
                new Translation3d(
                    -Units.inchesToMeters(9.8578),
                    Units.inchesToMeters(9.6913),
                    Units.inchesToMeters(20.2395)),
                new Rotation3d(0, 0, -Units.degreesToRadians(15))),
            0.1,
            3,
            7);

    public static final VisionPoseEstimatorConstants rightArducam =
        new VisionPoseEstimatorConstants(
            "right-arducam",
            new Transform3d(
                new Translation3d(
                    -Units.inchesToMeters(9.8578),
                    -Units.inchesToMeters(9.6913),
                    Units.inchesToMeters(20.2395)),
                new Rotation3d(0, 0, Units.degreesToRadians(15))),
            0.1,
            3,
            7);
  }

  public static class SwerveConstants {
    public static final Frequency odometryFrequency = Hertz.of(250);

    public static final Mass mass = Pounds.of(103);
    public static final MomentOfInertia moi = KilogramSquareMeters.of(8);

    public static final LinearVelocity driverTranslationalVelocity = MetersPerSecond.of(4);
    public static final AngularVelocity driverAngularVelocity = RadiansPerSecond.of(Math.PI);

    public static final LinearVelocity profileTranslationalVelocity = MetersPerSecond.of(1);
    public static final LinearAcceleration profileTranslationalAcceleration =
        MetersPerSecondPerSecond.of(2);

    public static final AngularVelocity profileAngularVelocity = RadiansPerSecond.of(Math.PI);
    public static final AngularAcceleration profileAngularAcceleration =
        RadiansPerSecondPerSecond.of(Math.PI * 2);

    public static final Per<LinearVelocityUnit, DistanceUnit> poseTranslationalkP =
        MetersPerSecond.per(Meter).ofNative(0);
    public static final Per<AngularVelocityUnit, AngleUnit> poseRotationkP =
        RadiansPerSecond.per(Radian).ofNative(0);

    public static final boolean ignorePoseTolerance = true;

    public static final Translation2d poseTranslationTolerance = new Translation2d(0.03, 0.03);
    public static final Rotation2d poseRotationTolerance = Rotation2d.fromDegrees(1);

    public static LinearVelocity translationalDeadband = MetersPerSecond.of(0.01);
    public static AngularVelocity rotationalDeadband = RadiansPerSecond.of(0.01);
  }

  public class ShooterConstants {
    public static final int shooterLeftFlywheelID = 1;
    public static final int shooterRightFlywheelID = 2;

    public static final Voltage flywheelkS = Volts.of(0.13262);
    public static final Per<VoltageUnit, AngularVelocityUnit> flywheelkV =
        Volts.per(RotationsPerSecond).ofNative(0.13334);
    public static final Per<VoltageUnit, AngularVelocityUnit> flywheelkP =
        Volts.per(RotationsPerSecond).ofNative(0.2);

    public static final double shooterGearRatio = 1.2;

    public static final double RPSTolerance = 5;

    // shooter rps, feed rps, roller rps
    public static final InterpolatingMatrixTreeMap<Double, N3, N1> hubPresets = new InterpolatingMatrixTreeMap<>();
    public static final InterpolatingDoubleTreeMap hubTOF = new InterpolatingDoubleTreeMap();

    static {
      hubPresets.put(1.89, vec3(38, 50, 40));
      hubPresets.put(2.665, vec3(43, 50, 40));
      hubPresets.put(3.768, vec3(47.75, 50, 40));
      hubPresets.put(4.574, vec3(51.8, 50, 40));
      hubPresets.put(5.252, vec3(52, 50, 40));

      hubTOF.put(1.89, 0.955);
      hubTOF.put(2.665, 1.08);
      hubTOF.put(3.768, 1.38);
      hubTOF.put(4.574, 1.53);
      hubTOF.put(5.252, 1.525);
    }

    public static final LinearVelocity horizontolProjectileVelocity = MetersPerSecond.of(2.722);
  }

  public class HopperConstants {
    public static int feedMotorID = 3;
    public static int rollerMotorID = 4;

    public static final Voltage feedKs = Volts.of(0.17);
    public static final Per<VoltageUnit, AngularVelocityUnit> feedKv =
        Volts.per(RotationsPerSecond).ofNative(0.235);
    public static final Per<VoltageUnit, AngularVelocityUnit> feedKp =
        Volts.per(RotationsPerSecond).ofNative(0.6);

    public static final Voltage rollerKs = Volts.of(0.21);
    public static final Per<VoltageUnit, AngularVelocityUnit> rollerKv =
        Volts.per(RotationsPerSecond).ofNative(0.28);
    public static final Per<VoltageUnit, AngularVelocityUnit> rollerKp =
        Volts.per(RotationsPerSecond).ofNative(0.6);

    public static final double feedGearRatio = 2.25;
    public static final double rollerGearRatio = 2;
  }

  public class IntakeConstants {
    public static final int feedMotorID = 5;
    public static final int pivotMotorID = 6;

    public static final Voltage feedKs = Volts.of(0.34);
    public static final Per<VoltageUnit, AngularVelocityUnit> feedKv =
        Volts.per(RotationsPerSecond).ofNative(0.205);
    public static final Per<VoltageUnit, AngularVelocityUnit> feedKp =
        Volts.per(RotationsPerSecond).ofNative(0.6);

    public static final Voltage pivotKg = Volts.of(0);
    public static final Voltage pivotKs = Volts.of(0);
    public static final Per<VoltageUnit, AngularVelocityUnit> pivotKv =
        Volts.per(RotationsPerSecond).ofNative(0);
    public static final Per<VoltageUnit, AngularVelocityUnit> pivotKp =
        Volts.per(RotationsPerSecond).ofNative(0);
    public static final Per<VoltageUnit, AngleUnit> pivotKa = Volts.per(Radian).ofNative(0);

    public static final double feedGearRatio = 2;
    public static final double pivotGearRatio = 20;
  }

  public class ClimberConstants {
    public static final int ClimberMotorID = 7;

    public static final Voltage climberKg = Volts.of(0);
    public static final Voltage climberKs = Volts.of(0);
    public static final Per<VoltageUnit, LinearVelocityUnit> climberKv =
        Volts.per(MetersPerSecond).ofNative(0);
    public static final Per<VoltageUnit, LinearVelocityUnit> climberKp =
        Volts.per(MetersPerSecond).ofNative(0);
    public static final Per<VoltageUnit, AngleUnit> climberKa = Volts.per(Radian).ofNative(0);

    public static final double climberGearRatio = 16;
  }

    public static Matrix<N3, N1> vec3(double a, double b, double c) {
        return new Matrix<N3, N1>(N3.instance, N1.instance, new double[] {a, b, c});
    }
}
