package team3176.robot.constants;

/** File for storing all hardware IDs to prevent double assignments */
public class Hardwaremap {
  /*
   * Superstructer CIDs & CBNs
   */
  public static final int elevatorLeader_CID = 30;
  public static final int elevatorFollower_CID = 5;
  public static final int shooterWheelUpper_CID = 7;
  public static final int shooterWheelLower_CID = 6;
  public static final int shooterTransfer_CID = 50;
  public static final int shooterPivot_CID = 4;
  public static final int intakeRoller_CID = 9;
  public static final int intakePivot_CID = 8;
  public static final int armEncoder_CID = 2;
  public static int PDH_CID = 1;

  public static final String elevatorLeaderMotor_CBN = "rio";
  public static final String elevatorFollowerMotor_CBN = "rio";
  public static final String shooterWheelUpper_CBN = "rio";
  public static final String shooterWheelLower_CBN = "rio";
  public static final String shooterWheelLower_CBN2 = "rio";
  public static final String shooterPivot_CBN = "rio";
  public static final String intakeRoller_CBN = "rio";
  public static final String intakePivot_CBN = "rio";
  public static String PDH_CBN = "rio";

  public static final int elevatorLeaderLimitSwitch_DIO = 9;
  public static final int elevatorFollowerLimitSwitch_DIO = 8;
  public static final int intakeRollerLinebreak_DIO = 0;
  public static final int intakePivotLinebreak_DIO = 1;
  public static final int intakeUpperLimitSwitch_DIO = 2;
  public static final int intakeLowerLimitSwitch_DIO = 3;
  public static final int shooterPivotLower_DIO = 4;

  public static final int blinkin_pwm_port = 9;

  /*
   * Drivetrain CIDs
   */
  // statics constants for swerve pods
  public static final String SWERVEPOD_CBN = "rio";
  public static final SwervePodHardwareID POD001 =
      new SwervePodHardwareID(1, 10, SWERVEPOD_CBN, 12, SWERVEPOD_CBN, -172.135);
  public static final SwervePodHardwareID POD002 =
      new SwervePodHardwareID(2, 20, SWERVEPOD_CBN, 22, SWERVEPOD_CBN, -225.186);
  public static final SwervePodHardwareID POD003 =
      new SwervePodHardwareID(3, 30, SWERVEPOD_CBN, 32, SWERVEPOD_CBN, -130);
  public static final SwervePodHardwareID POD004 =
      new SwervePodHardwareID(4, 40, SWERVEPOD_CBN, 42, SWERVEPOD_CBN, 140.463); // 120.5
  public static final SwervePodHardwareID POD005 =
      new SwervePodHardwareID(5, 13, SWERVEPOD_CBN, 14, SWERVEPOD_CBN, -30.525);
  public static final SwervePodHardwareID POD006 =
      new SwervePodHardwareID(6, 23, SWERVEPOD_CBN, 24, SWERVEPOD_CBN, 107);
  public static final SwervePodHardwareID POD007 =
      new SwervePodHardwareID(7, 33, SWERVEPOD_CBN, 34, SWERVEPOD_CBN, 125.508);
  public static final SwervePodHardwareID POD008 =
      new SwervePodHardwareID(8, 43, SWERVEPOD_CBN, 44, SWERVEPOD_CBN, -79);
  public static final SwervePodHardwareID POD009 =
      new SwervePodHardwareID(9, 15, SWERVEPOD_CBN, 16, SWERVEPOD_CBN, -138);

  public static final SwervePodHardwareID FR = POD009;
  public static final SwervePodHardwareID FL = POD008;
  public static final SwervePodHardwareID BL = POD006;
  public static final SwervePodHardwareID BR = POD003;

  public static final int STEER_FR_CID = 11;
  public static final int STEER_FL_CID = 21;
  public static final int STEER_BL_CID = 31;
  public static final int STEER_BR_CID = 41;
  public static final String STEER_FR_CBN = "rio";
  public static final String STEER_FL_CBN = "rio";
  public static final String STEER_BL_CBN = "rio";
  public static final String STEER_BR_CBN = "rio";
}
