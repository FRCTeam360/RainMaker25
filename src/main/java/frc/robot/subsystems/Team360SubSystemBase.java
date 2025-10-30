package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Team360SubSystemBase extends SubsystemBase {
  protected String getLogPreFix() {
    return getName() + ":";
  }
}
