# Apollo 🚀 2025-26 FTC Robot Code 

![Build Status](https://github.com/Yanai-Yakuel/ApolloLib-FTC/actions/workflows/build.yml/badge.svg)

**Lead Developer & Maintainer**: [Yanai-Yakuel](https://github.com/Yanai-Yakuel)

Welcome to the official repository for the **Apollo** team's 2025-2026 FTC competition robot. This project features a completely refactored modular architecture designed for high performance, reliability, and ease of use.

##  Project Highlights

### ApolloLib: Modular Robot Framework
The core of our robot is powered by **ApolloLib**, a custom-built Java library that abstracts hardware complexity into reusable subsystems.

*   **Centralized Control**: Managed by the [ApolloRobot](file:///c:/Users/blabl/Apollo_2025-26_FTC_Code/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apollo/ApolloRobot.java) class for unified initialization.
*   **Advanced Drive System**: Built on Roadrunner, featuring integrated Limelight vision for automated alignment and precise localization.
*   **State-Machine Shooter**: A reliable 4-stage state machine (`IDLE`, `SPOOLING`, `READY`, `SHOOTING`) ensuring perfect shots every time.
*   **Intuitive Intake**: Simplified control for high-speed collection and internal transfer.

###  Competition History
We maintain our historical integrity by preserving the specific codebases used in previous competitions. These are organized for easy reference:

*   **[Competition 1](file:///c:/Users/blabl/Apollo_2025-26_FTC_Code/TeamCode/src/main/java/comp1_code_folder/firstinspires/ftc/teamcode/)**: Legacy code from our first event.
*   **[Competition 2](file:///c:/Users/blabl/Apollo_2025-26_FTC_Code/TeamCode/src/main/java/comp2_code_folder/teamcode/)**: Refined strategies and updated mechanics. At this competition, our team was **Alliance 4 Captain** and received the **2nd Place Inspire Award**!

All historical code has been updated to support standard `.java` extensions and modern package structures.


##  How to Use it?

To use **ApolloLib** in your own FTC project, follow these steps:

1. **Extract the Library**: Copy the `org/firstinspires/ftc/teamcode/apollo` directory into your project's `TeamCode/src/main/java/` folder.
2. **Configure Your Hardware**: 
   - Open [ApolloRobot.java](file:///c:/Users/blabl/Apollo_2025-26_FTC_Code/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/apollo/ApolloRobot.java).
   - Update the hardware names (e.g., `"drive_left"`, `"shooter_motor"`) to match your robot's configuration.
3. **Initialize in Your OpMode**:
   ```java
   ApolloRobot robot = new ApolloRobot(hardwareMap);
   
   @Override
   public void runOpMode() {
       robot.init();
       waitForStart();
       while (opModeIsActive()) {
           robot.drive.drive(gamepad1); // Simple!
           if (gamepad1.a) robot.shooter.startShooting();
       }
   }



##  Technical Stack

*   **Language**: Java 8
*   **Framework**: FTC SDK v11.0+
*   **Navigation**: [Roadrunner](https://learnroadrunner.com/) (Mecanum/Tank drive support)
*   **Vision**: [Limelight 3A](https://limelightvision.io/)
*   **Dashboard**: FTC Dashboard for real-time telemetry and PID tuning.

---

##  Getting Started

1.  **Clone the Repo**:
    ```bash
    git clone https://github.com/Yanai-Yakuel/ApolloLib-FTC.git
    ```
2.  **Open in Android Studio**:
    - Import the project folder.
    - Perform a **Gradle Sync**.
3.  **Configure Robot**:
    - Ensure your robot configuration names match the strings in `ApolloRobot.java`.
4.  **Run**:
    - Select `ApolloCompetitionTeleOp` from the Driver Station to start driving!


##  Official SDK Documentation
For general information about the FTC SDK, please refer to the [FTC Docs](https://ftc-docs.firstinspires.org/).
