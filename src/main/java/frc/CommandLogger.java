package frc;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Code: Logs when a command begins and ends then gets the name of the command
 * Public: Logging when a command given to the robot begins and when it ends or is interupted
 */
public class CommandLogger {
  public static Command logCommand(Command command, String commandName) {
    return command.beforeStarting(() -> logCommand(commandName, true))
        .finallyDo(() -> logCommand(commandName, false));
  }

  private static void logCommand(String commandName, boolean isRunning) {
    Logger.recordOutput("CommandRunning " + commandName, isRunning);
  }
}
