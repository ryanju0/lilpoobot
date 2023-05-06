package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class ConeAlign extends CommandBase {
    private DriveSubsystem m_drive;
    private Limelight m_limelight;

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController thetaController;

    private double xGoal = 0.42;

    public ConeAlign(){
        this.m_drive = m_drive;
        this.m_limelight = m_limelight;

        xController = new ProfiledPIDController(0.7, 0, 0, new TrapezoidProfile.Constraints(0.7, 0.2));
        yController = new ProfiledPIDController(1.1, 0, 0, new TrapezoidProfile.Constraints(3, 3));
        thetaController = new ProfiledPIDController(1.5, 0, 0, new TrapezoidProfile.Constraints(5, 10));

        xController.setGoal(xGoal);
        xController.setTolerance(0,0);

        yController.setGoal(0);
        yController.setTolerance(0,0);

        thetaController.setGoal(0);
        thetaController.setTolerance(0,0);
    }
    @Override
    public void initialize() {
      m_limelight.alignPipeline();
      m_limelight.setLED(true);
    }
    @Override
    public void execute() {
      if(m_limelight.isTargetVisible()) {
        m_drive.drive(
          xController.calculate(m_limelight.getXOffset()), 
          yController.calculate(m_limelight.getYOffset()), 
          thetaController.calculate(m_drive.getHeading()), 
          true,
          false);
      }
    }
    @Override
    public void end(boolean interrupted) {
        m_limelight.setLED(false);
        m_drive.drive(0, 0, 0, true, false);
    }
    @Override
    public boolean isFinished() {
      return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }
}