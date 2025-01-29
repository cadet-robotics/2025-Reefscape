// package frc.robot.subsystems;

// /*
//  * Pipeline tuning:
//  *
//  * Threads : 3
//  * Decimate : Defualt
//  * Blur : Defualt
//  * Refine Edges: Defualt
//  * Max error bits : start with 0
//  * Decision Margin Cutoff : start aorund 30, increase for less false possitives
//  * Pose Estimation Iterations : start around 50, Increase to reduce noise ( risk wrong pose )
// */

// /*
//  * Camera Calibration:
//  *
//  * Print a checkerboard pattern
//  * Meassure to verify size
//  * Capture pictures are each resolution
//  * Use a tag to measue 3d accuracy
//  *
// */

// // Docs found at https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/pipelines
// // https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/programming
// // https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib

// import frc.robot.lib.custom.CCommand;
// import frc.robot.lib.custom.CSubsystem;

// import frc.robot.lib.Limelight.LimelightHelpers;

// import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.PS4Controller;
// import edu.wpi.first.wpilibj.PS4Controller.Button;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// public class LimelightSubsystem extends CSubsystem {

//     //private final LimelightHelpers m_limelight = new LimelightHelpers();

//     public LimelightSubsystem() {}

//     @Override
//     public void periodic() {

//         NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
//         NetworkTableEntry tx = table.getEntry("tx");
//         NetworkTableEntry ty = table.getEntry("ty");
//         NetworkTableEntry ta = table.getEntry("ta");

//         //read values periodically
//         double x = tx.getDouble(0.0);
//         double y = ty.getDouble(0.0);
//         double area = ta.getDouble(0.0);

//         //post to smart dashboard periodically
//         SmartDashboard.putNumber("LimelightX", x);
//         SmartDashboard.putNumber("LimelightY", y);
//         SmartDashboard.putNumber("LimelightArea", area);

//     }

//     public double getTX() {
//         return LimelightHelpers.getTX("");
//     }
//     public double getTY() {
//         return LimelightHelpers.getTY("");
//     }
//     public double getTA() {
//         return LimelightHelpers.getTA("");
//     }
//     public boolean hasTarget() {
//         return LimelightHelpers.getTV("");
//     }
//     public double getTXNC() {
//         return LimelightHelpers.getTXNC("");
//     }
//     public double getTYNC() {
//         return LimelightHelpers.getTYNC("");
//     }

// }