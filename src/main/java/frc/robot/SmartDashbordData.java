package frc.robot;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.Timer;

public class SmartDashbordData {

    public static double remainingTime;

    public static boolean isHubActive() {

  Optional<Alliance> alliance = DriverStation.getAlliance();
  // If we have no alliance, we cannot be enabled, therefore no hub.
  if (alliance.isEmpty()) {
    //IGNORE
    return false;
  }
  // Hub is always enabled in autonomous.
  if (DriverStation.isAutonomousEnabled()) {
    //IGNORE
    return true;
  }
  // At this point, if we're not teleop enabled, there is no hub.
  if (!DriverStation.isTeleopEnabled()) {
    //IGNORE
    return false;
  }

  // We're teleop enabled, compute.
  double matchTime = DriverStation.getMatchTime();
  String gameData = DriverStation.getGameSpecificMessage();
  remainingTime = matchTime;
  // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
  if (gameData.isEmpty()) {
    return true;
  }
  boolean redInactiveFirst = false;
  switch (gameData.charAt(0)) {
    case 'R' -> redInactiveFirst = true;
    case 'B' -> redInactiveFirst = false;
    default -> {
      // If we have invalid game data, assume hub is active.
      return true;
    }
  }

  // Shift was is active for blue if red won auto, or red if blue won auto.
  boolean shift1Active = switch (alliance.get()) {
    case Red -> !redInactiveFirst;//Red lose auto
    case Blue -> redInactiveFirst;//Red win auto
  };

  if (matchTime > 130) {
    // Transition shift, hub is active.
    remainingTime = matchTime - 130;
    return true;
  } else if (matchTime > 105) {
    // Shift 1
    remainingTime = matchTime - 105;
    return shift1Active;
  } else if (matchTime > 80) {
    // Shift 2
    remainingTime = matchTime - 80;
    return !shift1Active;
  } else if (matchTime > 55) {
    // Shift 3
    remainingTime = matchTime - 55;
    return shift1Active;
  } else if (matchTime > 30) {
    // Shift 4
    remainingTime = matchTime - 30;
    return !shift1Active;
  } else {
    // End game, hub always active.
    remainingTime = matchTime;
    return true;
  }

  
    }

    public static boolean didWinAuto(){
        String gameData;
gameData = DriverStation.getGameSpecificMessage();
Optional<Alliance> alliance = DriverStation.getAlliance();
if(gameData.length() > 0)
{
  switch (gameData.charAt(0))
  {
    case 'B' :
      //Blue case code
      if(alliance.isPresent()) {
          if(alliance.get() == Alliance.Red){

            return false;
          }
          else{
            return true;
          }
      }
      break;
    case 'R' :
      //Red case code
      if(alliance.isPresent()) {
          if(alliance.get() == Alliance.Red){
            return true;
          }
          else{
            return false;
          }
      }
      break;
    default :
      //This is corrupt data
      break;
  }
} else {
  //Code for no data received yet
  return false;
}

  return false;
    }

}

