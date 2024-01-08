package team3176.robot.constants;

import team3176.robot.constants.SwervePodHardwareID;
public class DrivetrainHardwareMap {
    //statics constants for swerve pods 
    public static final SwervePodHardwareID POD001 = 
    new SwervePodHardwareID(  10,  12,  -172.135);
    public static final SwervePodHardwareID POD002 = 
    new SwervePodHardwareID(  20,  22,  -225.186);
    public static final SwervePodHardwareID POD003 = 
    new SwervePodHardwareID(  30,  32, -40);
    public static final SwervePodHardwareID POD004 = 
    new SwervePodHardwareID( 40,  42,  140.463);  //120.5
    public static final SwervePodHardwareID POD005 = 
    new SwervePodHardwareID(  13,  14,  -30.525);
    public static final SwervePodHardwareID POD006 = 
    new SwervePodHardwareID(  23,  24,  120.556);
    public static final SwervePodHardwareID POD007 = 
    new SwervePodHardwareID(  33,  34,  125.508);
    public static final SwervePodHardwareID POD008 = 
    new SwervePodHardwareID(  43,  44,  -173);
    public static final SwervePodHardwareID POD009 = 
    new SwervePodHardwareID(  15,  16,  -358.330);
   

    public static final SwervePodHardwareID FR = POD009;
    public static final SwervePodHardwareID FL = POD008;
    public static final SwervePodHardwareID BL = POD006;
    public static final SwervePodHardwareID BR = POD003;
    
    // public static final int THRUST_FR_CID = FR.THRUST_CID;
    // public static final int THRUST_FL_CID = FL.THRUST_CID;
    // public static final int THRUST_BL_CID = BL.THRUST_CID;
    // public static final int THRUST_BR_CID = BR.THRUST_CID;

    
    public static final int[] STEER_CANCODER_CID = 
    //{12, 22, 32, 42};
    {FR.CANCODER_CID, FL.CANCODER_CID, BL.CANCODER_CID, BR.CANCODER_CID}; 
    

    //The swerve pod offset is measured when the swerve pod is in the front right position and the wheel gear is facing the right
    // to counteract the offset caused by the mounting in different positions
    public static final double[] 
    AZIMUTH_ABS_ENCODER_OFFSET_POSITION = 
    { FR.OFFSET+180, FL.OFFSET+90, BL.OFFSET, BR.OFFSET-90}; //TODO: I think these offsets are wrong. I would double check the pods by setting them each as if they were FR zero and remeasuring the offsets
    

    
     //CAN IDs static to the frame
     public static final int STEER_FR_CID = 11;
     public static final int STEER_FL_CID = 21;
     public static final int STEER_BL_CID = 31;
     public static final int STEER_BR_CID = 41;
     
}

