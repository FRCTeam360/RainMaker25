package frc;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Code: Logs when a command begins and ends then gets the name of the command
 * Public: Logging when a command given to the robot begins and when it ends or is interupted
 */
public class CommandLogger {
  public static Command logCommand(Command command) {
    return command.beforeStarting(() -> logCommand(command, true))
        .finallyDo(() -> logCommand(command, false));
  }

  private static void logCommand(Command command, boolean isRunning) {
    Logger.recordOutput("CommandRunning " + command.getName(), isRunning);
  }
}
