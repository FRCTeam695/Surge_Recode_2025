// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class PIDConstants1 {
    public static final double kP1 = 0.0001; //0.0001
    public static final double kI1 = 0;
    public static final double kD1 = 0;
    public static final double ff1 = 0.000156; //0.000156
    public static final double kMaxOutput1 = 1;
    public static final double kMinOutput1 = -1;
  }

  public static class PIDConstants2 {
    public static final double kP2 = 0;
    public static final double kI2 = 0;
    public static final double kD2 = 0;
    public static final double ff2 = 0;
    public static final double kMaxOutput2 = 1;
    public static final double kMinOutput2 = -1;
  }
}
