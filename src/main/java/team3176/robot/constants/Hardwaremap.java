package team3176.robot.constants;

/**
 * File for storing all hardware IDs to prevent double assignments
 */
public class Hardwaremap {
    /*
     * Superstructer CIDs
     */
    public static int arm_CID = 3;
    public static int claw_CID = 6;
    public static int intake_CID = 7;
    public static int armEncoder_CID = 2;
    //public static int armEncoder_CID = 9;
    public static int PDH_CID = 1;

    /*
     * Drivetrain CIDs
     */
    public static final SwervePodHardwareID pod001 = 
    new SwervePodHardwareID(  10,  12,  0.216);
    public static final SwervePodHardwareID pod002 = 
    new SwervePodHardwareID(  20,  22,  -103.48);
    public static final SwervePodHardwareID pod003 = 
    new SwervePodHardwareID(  30,  32,  135.689);
    public static final SwervePodHardwareID pod004 = 
    new SwervePodHardwareID( 40,  42,  -89.945);
    public static final SwervePodHardwareID pod005 = 
    new SwervePodHardwareID(  13,  14,  105.586);
    public static final SwervePodHardwareID pod006 = 
    new SwervePodHardwareID(  23,  24,  135.547);
    public static final SwervePodHardwareID pod007 = 
    new SwervePodHardwareID(  33,  34,  -35.16);
    public static final SwervePodHardwareID pod008 = 
    new SwervePodHardwareID(  43,  44,  77.557);

    public static final int STEER_FR_CID = 11;
    public static final int STEER_FL_CID = 21;
    public static final int STEER_BL_CID = 31;
    public static final int STEER_BR_CID = 41;
}
