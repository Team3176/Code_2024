package team3176.robot.constants;

/** File for storing all hardware IDs to prevent double assignments */
public class Hardwaremap {
  /*
   * Superstructer CIDs
   */
  public static int arm_CID = 3;
  public static int claw_CID = 6;
  public static int intake_CID = 7;
  public static int armEncoder_CID = 2;
  // public static int armEncoder_CID = 9;
  public static int PDH_CID = 1;

  /*
   * Drivetrain CIDs
   */
   // statics constants for swerve pods
   public static final SwervePodHardwareID POD001 = new SwervePodHardwareID(1,10, 12, -172.135);
   public static final SwervePodHardwareID POD002 = new SwervePodHardwareID(2,20, 22, -225.186);
   public static final SwervePodHardwareID POD003 = new SwervePodHardwareID(3,30, 32, -130);
   public static final SwervePodHardwareID POD004 = new SwervePodHardwareID(4,40, 42, 140.463); // 120.5
   public static final SwervePodHardwareID POD005 = new SwervePodHardwareID(5,13, 14, -30.525);
   public static final SwervePodHardwareID POD006 = new SwervePodHardwareID(6,23, 24, 107);
   public static final SwervePodHardwareID POD007 = new SwervePodHardwareID(7,33, 34, 125.508);
   public static final SwervePodHardwareID POD008 = new SwervePodHardwareID(8,43, 44, -79);
   public static final SwervePodHardwareID POD009 = new SwervePodHardwareID(9,15, 16, -138);
 
   public static final SwervePodHardwareID FR = POD009;
   public static final SwervePodHardwareID FL = POD008;
   public static final SwervePodHardwareID BL = POD006;
   public static final SwervePodHardwareID BR = POD003;

   public static final int STEER_FR_CID = 11;
   public static final int STEER_FL_CID = 21;
   public static final int STEER_BL_CID = 31;
   public static final int STEER_BR_CID = 41;
}
