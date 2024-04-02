package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class VisionSubsystem extends SubsystemBase {
    final private NetworkTableEntry ty;
    final private NetworkTableEntry tx; 
    final private NetworkTableEntry tv;
    //final private NetworkTableEntry tl;
    final private NetworkTableEntry txNote;
    final private NetworkTableEntry tyNote; 
    private final NetworkTableEntry taNote;
    private final NetworkTableEntry tclassNote;
    final private NetworkTableEntry tvNote;
    
    final private NetworkTable limelight2Table = NetworkTableInstance.getDefault().getTable("limelight-two");
    final private NetworkTable limelight3Table = NetworkTableInstance.getDefault().getTable("limelight-three");

    //Field for april tag detection
    private final Field2d m_field = new Field2d();
    

    public VisionSubsystem(){
        ty = limelight3Table.getEntry("ty");
        tx = limelight3Table.getEntry("tx");
        
        //TODO: what is tl and why is it not configured properly?
        //tl = limelightTable.getEntry("ty");
        tv = limelight3Table.getEntry("tv");
        
        txNote = limelight2Table.getEntry("tx");
        tyNote = limelight2Table.getEntry("ty");
        taNote = limelight2Table.getEntry("ta");
        tclassNote = limelight2Table.getEntry("tclass");
        tvNote = limelight2Table.getEntry("tv");

        //setPipeline(VisionConstants.kDefaultPipeline);

        // Shuffleboard.getTab("Vision").addInteger("Pipeline", () -> getPipeline());
        // Shuffleboard.getTab("Vision").addDouble("Distance", () -> getReflectiveTapeDistance());
        // Shuffleboard.getTab("Vision").add(m_field);
    }

    //tv = valid targets
    //tx horizontal offset from crosshair to target
    //ty vertical offset from crosshair to target
    //ta = target area 0% to 100%
    
    public void setPipeline(int pipeline, int pipelineNote){
        limelight3Table.getEntry("pipeline").setNumber(pipeline);
        limelight2Table.getEntry("pipeline").setNumber(pipelineNote);
    }

    public int getPipeline(){
        return ((Double)limelight3Table.getEntry("pipeline").getNumber(-1)).intValue();  

    }

    /**
     * Returns the horizontal offset from the crosshair to the target. Returns 0 if the target can't be found.
     */
    public double getX(){
        return tx.getDouble(0.0);
    }
    
    /**
     * Returns the vertical offset from the crosshair to the target. Returns 0 if the target can't be found.
     */
    public double getY(){
        return ty.getDouble(0.0);
    }
    
    // public double getTL(){
    //     return tl.getDouble(0.0);
    // }
    
    /**
     * Returns the number of valid targets detected by the limelight. Returns 0 if no targets are found.
     */
    public int getTV(){
        return (int)tv.getDouble(0);
    }

    public double getNoteTX()
    {
        return txNote.getDouble(0.0);
    }
    public double getNoteTY()
    {
        return tyNote.getDouble(0.0);
    }
    public double getNoteTA()
    {
        return taNote.getDouble(0.0);
    }
    public String getNoteTClass()
    {
        return tclassNote.getString("Nothing");
    }
    public double getNoteTV()
    {
        return tvNote.getDouble(0.0);
    }
    /**
     * Returns the distance between the robot and the reflective tape goal. Returns 0 if no targets can be found.
     */
    public double getReflectiveTapeDistance(){
        //Check for no targets
        if(getTV() == 0){
            return 0;
        }
        
        final double targetOffsetAngle_Vertical = getNoteTX();

        // distance from the target to the floor
        final double goalHeightInches;

        if(targetOffsetAngle_Vertical > 0){
            goalHeightInches = VisionConstants.kTopReflectiveTapeHeight;
        }
        else{
            goalHeightInches = VisionConstants.kBottomReflectiveTapeHeight;
        }

        final double angleToGoalDegrees = VisionConstants.kLimelightMountAngle + targetOffsetAngle_Vertical;
        final double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        //calculate distance
        return (goalHeightInches - VisionConstants.kLimelightLensHeight) / Math.tan(angleToGoalRadians);
    }
     public double getNoteDistance(){
        //Check for no targets
        if(getTV() == 0){
            return 0;
        }
        if(getNoteTClass() == "note")
        {
            //calculate distance
            return getNoteTX();
        }
        else{
            return 0;
        }
    }

    /**
     * Uses whatever april tag is in front of it to estimate the robot's position on the field. 
     * Returns null if no april tag is in view.
     * @return The position of the robot, or null.
     */
    public Pose2d getRobotPosition(){
        
        if(getPipeline() == VisionConstants.kAprilTagPipeline) {
            /* Notes for Gabe: a Pose2d object contains a robot's x position, y position, 
             * and rotation. I need your help with taking the limelight values from network tables 
             * and converting them to a robot pose object. Once you do that I'll figure out how
             * to use this object for correcting the odometry.
             */
            //TODO: implement this function
            return new Pose2d(0, 0, new Rotation2d(0));
        }
        return null;
    }


    @Override
    public void periodic(){
        m_field.setRobotPose(getRobotPosition());
    }
}