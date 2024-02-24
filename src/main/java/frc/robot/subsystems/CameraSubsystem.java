package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class CameraSubsystem extends SubsystemBase{

    private final VisionSystemSim simVisionSystem = new VisionSystemSim("PhotonVision");
    private final PhotonCamera photonCamera= new PhotonCamera("Arducam_OV9281_USB_Camera");
    private PhotonCameraSim photonCameraSim = new PhotonCameraSim(photonCamera);
    private final SimCameraProperties cameraProp = new SimCameraProperties();
    
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final Transform3d robotToCam = new Transform3d(new Translation3d(-0.047625, -0.3048, 0.51435), new Rotation3d(0, -0.453786, 2.89725));

    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, robotToCam);

    private final SwerveSubsystem swerveDrive;
    private EstimatedRobotPose latestEstimatedRobotPose;

    public CameraSubsystem(SwerveSubsystem swerveDrive){
        this.swerveDrive = swerveDrive;

        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(75));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        //cameraProp.setCalibError(0, 0);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        photonCameraSim = new PhotonCameraSim(photonCamera, cameraProp);
        photonCameraSim.enableProcessedStream(true);
        photonCameraSim.enableRawStream(false);
        photonCameraSim.enableDrawWireframe(false);

        photonCamera.setDriverMode(false);

        simVisionSystem.addCamera(photonCameraSim, robotToCam);
        simVisionSystem.addAprilTags(aprilTagFieldLayout);
    }
    
    public void periodicCall()
    {
        
        Optional<EstimatedRobotPose> currentEstimatedRobotPose = photonPoseEstimator.update();

        if (currentEstimatedRobotPose.isPresent()) {
        swerveDrive.addFakeVisionReading(
            currentEstimatedRobotPose.get().estimatedPose.toPose2d());

        }
    }

    public void simulationPeriodicCall()
    {
        simVisionSystem.update(swerveDrive.getPose());
    }
}