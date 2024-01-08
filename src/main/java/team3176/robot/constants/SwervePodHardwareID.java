// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package team3176.robot.constants;

public class SwervePodHardwareID {
    public int SERIAL;
    public int THRUST_CID;
    public int CANCODER_CID;
    public double OFFSET;
    SwervePodHardwareID(){}
    SwervePodHardwareID(int thrust_cid, int cancoder_cid, double offset) {
        this.THRUST_CID = thrust_cid;
        this.CANCODER_CID = cancoder_cid;
        this.OFFSET = offset;
    }
    public static boolean check_duplicates(SwervePodHardwareID a, SwervePodHardwareID b){
        boolean are_duplicates = (a.CANCODER_CID == b.CANCODER_CID)| (a.THRUST_CID == b.THRUST_CID);
        return are_duplicates;
    }
    public static boolean check_duplicates_all(SwervePodHardwareID a, SwervePodHardwareID b,SwervePodHardwareID c, SwervePodHardwareID d){
        return check_duplicates(a,b) | check_duplicates(a,c)| check_duplicates(a,d)| check_duplicates(b,c) | check_duplicates(b,d) | check_duplicates(c,d);
    }
}