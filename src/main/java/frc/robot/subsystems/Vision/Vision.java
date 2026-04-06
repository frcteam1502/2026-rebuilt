package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.Vision.PhotonCameraCfg.FIELD_TAG_LAYOUT;
import static frc.robot.subsystems.Vision.PhotonCameraCfg.angularStdDevBaseline;
import static frc.robot.subsystems.Vision.PhotonCameraCfg.angularStdDevMegatag2Factor;
import static frc.robot.subsystems.Vision.PhotonCameraCfg.cameraStdDevFactors;
import static frc.robot.subsystems.Vision.PhotonCameraCfg.linearStdDevBaseline;
import static frc.robot.subsystems.Vision.PhotonCameraCfg.linearStdDevMegatag2Factor;
import static frc.robot.subsystems.Vision.PhotonCameraCfg.maxAmbiguity;
import static frc.robot.subsystems.Vision.PhotonCameraCfg.maxZError;
import frc.robot.subsystems.Vision.VisionIO.PoseObservationType;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIO.VisionIOInputs[] inputs;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIO.VisionIOInputs[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIO.VisionIOInputs();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
    }

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > FIELD_TAG_LAYOUT.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > FIELD_TAG_LAYOUT.getFieldWidth();

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);
  }

}
