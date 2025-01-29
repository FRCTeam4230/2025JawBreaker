package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.utils.FieldConstants;
import java.util.ArrayList;
import java.util.List;

/**
 * Utility class for vision processing that provides different vision modes, measurement validation,
 * and data structures for vision measurements. Supports both MegaTag1 (MT1) and MegaTag2 (MT2)
 * vision systems.
 */
public class VisionUtil {
  // Configuration constants
  private static volatile boolean BEFORE_MATCH = true; // Controls MT1-only usage before match
  public static final double FIELD_MARGIN =
      0.5; // Meters beyond field boundaries to accept measurements
  public static final double Z_MARGIN = 0.5; // Meters above/below field to accept measurements
  public static final double MT2_SPIN_MAX = 40.0; // Maximum rotation speed for MT2 measurements
  public static final double MIN_TAG_AREA = 0.05; // Minimum tag area to be accepted

  // Vision measurement constants for MA mode
  private static final double MA_VISION_STD_DEV_XY = 0.333; // Base XY standard deviation
  private static final double MA_VISION_STD_DEV_THETA = 5.0; // Base theta standard deviation
  public static final double MA_AMBIGUITY =
      0.4; // Maximum allowed ambiguity for single-tag measurements

  /**
   * Enum defining different vision processing modes with unique validation and measurement
   * calculation implementations.
   */
  public enum VisionMode {
    /**
     * Mode that rejects all vision measurements. Useful for testing or when vision should be
     * temporarily disabled.
     */
    NONE {
      @Override
      public VisionMeasurement getVisionMeasurement(PoseEstimate mt) {
        return new VisionMeasurement(mt, VecBuilder.fill(0.0, 0.0, 0.0));
      }

      @Override
      public boolean acceptVisionMeasurement(PoseObservation mt) {
        return false;
      }
    },

    /**
     * Standard vision mode that calculates measurement uncertainties based on distance. Uses a
     * simple model where standard deviations increase quadratically with distance and decrease
     * linearly with the number of tags detected.
     */
    MA {
      @Override
      public VisionMeasurement getVisionMeasurement(PoseEstimate mt) {
        double xyStdDev = calculateXYStdDev(mt);
        // MT2 measurements don't provide reliable rotation data
        double thetaStdDev = mt.isMegaTag2() ? Double.MAX_VALUE : calculateThetaStdDev(mt);
        return new VisionMeasurement(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
      }

      /**
       * Calculates the standard deviation for X and Y measurements.
       *
       * @param mt The pose estimate containing tag detection information
       * @return Standard deviation scaled by distance squared and tag count
       */
      private double calculateXYStdDev(PoseEstimate mt) {
        return MA_VISION_STD_DEV_XY * Math.pow(mt.avgTagDist(), 2.0) / mt.tagCount();
      }

      /**
       * Calculates the standard deviation for rotation measurements.
       *
       * @param mt The pose estimate containing tag detection information
       * @return Standard deviation scaled by distance squared and tag count
       */
      private double calculateThetaStdDev(PoseEstimate mt) {
        return MA_VISION_STD_DEV_THETA * Math.pow(mt.avgTagDist(), 2.0) / mt.tagCount();
      }
    },

    /** Implements generic version POOF (Cheesy Poof 254) algorithm . */
    POOF {
      @Override
      public VisionMeasurement getVisionMeasurement(PoseEstimate mt) {
        // Reject measurements outside the field boundaries
        if (invalidPose(mt.pose())) {
          return new VisionMeasurement(
              mt, VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE));
        }
        return calculatePoofMeasurement(mt);
      }

      /**
       * Calculates vision measurements using the POOF algorithm.
       *
       * @param mt The pose estimate to process
       * @return A VisionMeasurement with calculated standard deviations
       */
      private VisionMeasurement calculatePoofMeasurement(PoseEstimate mt) {
        var stdDevs = calculateStandardDeviations(mt);
        var xyStdDev = stdDevs.xyStdDev();
        var thetaStdDev = stdDevs.thetaStdDev();

        // MT2 measurements don't provide reliable rotation data
        if (mt.isMegaTag2()) {
          thetaStdDev = Double.MAX_VALUE;
        }

        return new VisionMeasurement(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
      }
    };

    /**
     * Record holding vision standard deviation values for both position and rotation.
     *
     * @param xyStdDev Standard deviation for X and Y measurements
     * @param thetaStdDev Standard deviation for rotation measurements
     */
    private record VisionDevs(double xyStdDev, double thetaStdDev) {}

    /**
     * Calculates standard deviations based on the number of tags detected.
     *
     * @param mt The pose estimate containing tag detection information
     * @return VisionDevs containing calculated standard deviations
     */
    private static VisionDevs calculateStandardDeviations(PoseEstimate mt) {
      if (mt.tagCount() >= 2) {
        return calculateMultipleTagStdDevs(mt);
      } else if (mt.tagCount() == 1) {
        return calculateSingleTagStdDevs(mt);
      }
      // No tags detected - return maximum uncertainty
      return new VisionDevs(Double.MAX_VALUE, Double.MAX_VALUE);
    }

    /**
     * Calculates standard deviations when multiple tags are detected. Uses tag area to determine
     * measurement confidence.
     *
     * @param mt The pose estimate containing tag detection information
     * @return VisionDevs with appropriate standard deviations
     */
    private static VisionDevs calculateMultipleTagStdDevs(PoseEstimate mt) {
      boolean hasLargeTagArea = mt.avgTagArea() > 0.1;
      // Higher confidence (smaller std devs) when tags appear larger in the image
      return hasLargeTagArea
          ? new VisionDevs(0.2, Units.degreesToRadians(6.0))
          : new VisionDevs(1.2, Units.degreesToRadians(12.0));
    }

    /**
     * Calculates standard deviations when only one tag is detected. Uses both tag area and distance
     * from expected pose to determine confidence.
     *
     * @param mt The pose estimate containing tag detection information
     * @return VisionDevs with appropriate standard deviations
     */
    private static VisionDevs calculateSingleTagStdDevs(PoseEstimate mt) {
      // Calculate how far the measured pose is from the expected pose
      double poseDifference =
          mt.robotPose().getTranslation().getDistance(mt.pose().getTranslation().toTranslation2d());

      // Define confidence thresholds based on pose difference and tag area
      boolean isCloseToExpectedPose = poseDifference < 0.5;
      boolean isVeryCloseToExpectedPose = poseDifference < 0.3;
      boolean hasLargeTagArea = mt.avgTagArea() > 0.1;

      // Assign standard deviations based on confidence levels
      if (mt.avgTagArea() > 0.8 && isCloseToExpectedPose) {
        // Highest confidence case
        return new VisionDevs(0.5, Units.degreesToRadians(12.0));
      } else if (hasLargeTagArea && isVeryCloseToExpectedPose) {
        // Medium confidence case
        return new VisionDevs(1.0, Units.degreesToRadians(25.0));
      }
      // Low confidence case
      return new VisionDevs(2.0, Units.degreesToRadians(50.0));
    }

    /**
     * Creates a vision measurement with calculated standard deviations.
     *
     * @param mt The pose estimate to process
     * @return A vision measurement with appropriate standard deviations
     */
    public abstract VisionMeasurement getVisionMeasurement(PoseEstimate mt);

    /**
     * Default implementation for validating vision measurements. Checks all standard validation
     * criteria. This implementation is shared between MA and POOF modes.
     *
     * @param mt The pose observation to validate
     * @return True if the measurement should be accepted, false otherwise
     */
    public boolean acceptVisionMeasurement(PoseObservation mt) {
      PoseEstimate poseEst = mt.poseEstimate();
      return !invalidPose(poseEst.pose())
          && !invalidMT2Time(poseEst)
          && !invalidRotationVelocity(poseEst)
          && !invalidTagArea(poseEst)
          && !invalidAmbiguity(poseEst);
    }
  }

  /** Record containing a pose estimate and its associated standard deviations. */
  public record VisionMeasurement(PoseEstimate poseEstimate, Vector<N3> visionMeasurementStdDevs) {}

  /**
   * Record containing collections of vision measurements and poses, categorized by acceptance
   * status.
   */
  public record VisionData(
      List<VisionMeasurement> measurements,
      List<Pose3d> tagPoses,
      List<Pose3d> acceptedTagPoses,
      List<Pose3d> rejectedTagPoses,
      List<Pose3d> robotPoses,
      List<Pose3d> acceptedPoses,
      List<Pose3d> rejectedPoses) {

    /** Creates an empty VisionData object. */
    public static VisionData empty() {
      return new VisionData(
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>());
    }

    /** Merges two lists of the same type. */
    private static <T> List<T> mergeLists(List<T> list1, List<T> list2) {
      ArrayList<T> merged = new ArrayList<>(list1);
      merged.addAll(list2);
      return merged;
    }

    /**
     * Combines this VisionData with another, merging all corresponding lists.
     *
     * @param other The VisionData to merge with
     * @return A new VisionData containing all elements from both objects
     */
    public VisionData merge(VisionData other) {
      return new VisionData(
          mergeLists(measurements, other.measurements),
          mergeLists(tagPoses, other.tagPoses),
          mergeLists(acceptedTagPoses, other.acceptedTagPoses),
          mergeLists(rejectedTagPoses, other.rejectedTagPoses),
          mergeLists(robotPoses, other.robotPoses),
          mergeLists(acceptedPoses, other.acceptedPoses),
          mergeLists(rejectedPoses, other.rejectedPoses));
    }
  }

  /**
   * Validation helper method to check if MT2 measurements are being used before match start.
   *
   * @param mt The pose estimate to validate
   * @return True if the measurement is invalid due to timing constraints
   */
  private static boolean invalidMT2Time(PoseEstimate mt) {
    return mt.isMegaTag2() && beforeMatch();
  }

  /**
   * Validation helper method to check if rotation velocity is within acceptable limits.
   *
   * @param mt The pose estimate to validate
   * @return True if the rotation velocity exceeds the maximum allowed speed
   */
  private static boolean invalidRotationVelocity(PoseEstimate mt) {
    return Math.abs(mt.yawVelocityRadPerSec()) > MT2_SPIN_MAX;
  }

  /**
   * Validation helper method to check if ambiguity is acceptable for single-tag measurements.
   *
   * @param mt The pose estimate to validate
   * @return True if the ambiguity is too high for a single-tag measurement
   */
  private static boolean invalidAmbiguity(PoseEstimate mt) {
    return mt.tagCount() == 1 && mt.ambiguity() > MA_AMBIGUITY;
  }

  /**
   * Validation helper method to check if the pose is within the valid field boundaries.
   *
   * @param robotPose The 3D pose to validate
   * @return True if the pose is outside the valid field boundaries
   */
  private static boolean invalidPose(Pose3d robotPose) {
    return robotPose.getX() < -FIELD_MARGIN
        || robotPose.getX() > FieldConstants.fieldLength + FIELD_MARGIN
        || robotPose.getY() < -FIELD_MARGIN
        || robotPose.getY() > FieldConstants.fieldWidth + FIELD_MARGIN
        || robotPose.getZ() < -Z_MARGIN
        || robotPose.getZ() > Z_MARGIN;
  }

  /**
   * Validation helper method to check if the average tag area is sufficient.
   *
   * @param mt The pose estimate to validate
   * @return True if the average tag area is below the minimum threshold
   */
  private static boolean invalidTagArea(PoseEstimate mt) {
    return mt.avgTagArea() < MIN_TAG_AREA;
  }

  /**
   * Checks if the robot is in the pre-match phase where only MT1 should be used. Updates the
   * BEFORE_MATCH flag when the robot becomes enabled.
   *
   * @return True if the robot is in pre-match phase
   */
  public static boolean beforeMatch() {
    if (DriverStation.isEnabled()) {
      BEFORE_MATCH = false;
    }
    return BEFORE_MATCH;
  }
}
