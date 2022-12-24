package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    final double heightCamera = 0; //height of limelight
    final double heightTarget = 0; //height of target
    double cameraAngle = 0; //angle of the camera

    public Limelight(){}

    @Override 
    public void periodic(){
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }
    public double tx() {
        return tx.getDouble(0.0);
    }

    public double ty(){
        return ty.getDouble(0.0);
    }

    public double ta(){
        return ta.getDouble(0.0);
    }
    public boolean tv(){
        return tv.getDouble(0.0) == 1.0; 
    }

    public double getDistance(){
        //d = (h2-h1) / tan(a1+a2)
        cameraAngle = Math.toRadians(cameraAngle);
        double pitch = Math.toRadians(ty()); //value ty aka pitch value given from limelight
        return (heightTarget-heightCamera)/Math.tan(cameraAngle+pitch); //imagine you use degree inside tan xd
    }
}