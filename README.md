# SwerveBase-CTRE
[![CI](https://github.com/Team334/SwerveBase-CTRE/actions/workflows/main.yml/badge.svg)](https://github.com/Team334/SwerveBase-CTRE/actions/workflows/main.yml)

A base project for future robots that has CTRE generated swerve drive code and Photon Vision AprilTag processing.

# Features
- Swerve drive code using CTRE's swerve generator.
- Device logging with SignalLogger and data logging with Epilogue and DogLog.
- Device Fault Logging as telemetry for at-home testing. These faults are also logged with DogLog for post-match review.
- Pre-match self-check with custom self-check commands.
- A custom `VisionPoseEstimator` class that reads from a Photon Vision camera and updates the swerve pose estimator with filtered and disambiguated AprilTag vision measurements.
- Automated wheel radius characterization routine (based on 6328's wheel characterization).
- Choreo support.

# How This Will Be Used
- At the start of each season, this project should be re-imported in the season's wpilib release for any additional updates.
- After that, since this project is a template on github, the actual season's robot code repo can be generated from this template:
![github templates](https://docs.github.com/assets/cb-76823/mw-1440/images/help/repository/use-this-template-button.webp)
