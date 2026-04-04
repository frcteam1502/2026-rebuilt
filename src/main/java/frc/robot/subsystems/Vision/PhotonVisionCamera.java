/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.LinkedList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionCamera {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private double lastTimestamp;
    private Optional<EstimatedRobotPose> estimatedGlobalPose;

    public PhotonVisionCamera(String cameraName, Transform3d robotToCam) {
        camera = new PhotonCamera(cameraName);

        photonEstimator = new PhotonPoseEstimator(PhotonCameraCfg.FIELD_TAG_LAYOUT, 
												  PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
												  robotToCam);
        
		photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @param referencePose A {@link Pose2d} of the current pose of the robot to determine what is the closest AprilTag
     *      from the list of observed AprilTags
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> processCamera(Pose2d referencePose){
        //Clear Estimated Pose in case no valid pose is found
        estimatedGlobalPose = Optional.empty();

        //Read all results from the camera.
        PhotonPipelineResult pipelineResult = camera.getLatestResult();

        //Pass in the reference pose for the robot
        photonEstimator.setReferencePose(referencePose);
        //Return if no new results were received
        if(pipelineResult.getTimestampSeconds() == lastTimestamp) return estimatedGlobalPose;

        //Find the targets too inaccurate to be used. 
        LinkedList<PhotonTrackedTarget> toRemove = new LinkedList<PhotonTrackedTarget>();
        for(int i=0; i<pipelineResult.targets.size(); i++){
            var result = pipelineResult.targets.get(i);
            int tagID = result.fiducialId;
            Pose2d tagPose = PhotonCameraCfg.FIELD_TAG_LAYOUT.getTagPose(tagID).get().toPose2d();
            double distanceToTarget = PhotonUtils.getDistanceToPose(referencePose, tagPose);
            if((result.getPoseAmbiguity() > PhotonCameraCfg.MINIMUM_TARGET_AMBIGUITY)||
               (distanceToTarget > PhotonCameraCfg.DISTANCE_THRESHOLD_M)){
                toRemove.add(result);
            }
        }
        //Remove all the ambiguous targets
        pipelineResult.targets.removeAll(toRemove);

        //Return if the list of targets is non-existant or invalid
        if(!pipelineResult.hasTargets()) return estimatedGlobalPose;

        var calculatedPose = photonEstimator.update(pipelineResult);
        
        if(calculatedPose.isPresent()){
            //Make sure the estimated pose is on the field
            if((calculatedPose.get().estimatedPose.getX() >= 0.0) && 
               (calculatedPose.get().estimatedPose.getX() <= PhotonCameraCfg.FIELD_TAG_LAYOUT.getFieldLength()) &&
               (calculatedPose.get().estimatedPose.getY() >= 0.0) && 
               (calculatedPose.get().estimatedPose.getY() <= PhotonCameraCfg.FIELD_TAG_LAYOUT.getFieldWidth())){
                    //Estimated pose is on the field!
                    estimatedGlobalPose = calculatedPose;
                    lastTimestamp = calculatedPose.get().timestampSeconds;
                    return estimatedGlobalPose;
               }
        };
        return estimatedGlobalPose;
    }
}
