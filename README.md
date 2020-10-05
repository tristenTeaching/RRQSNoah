# Road Runner Advanced Examples

You should be fairly familiar with Road Runner and the quickstart before taking a look at these samples.

The main quickstart repo can be found [here](https://github.com/acmerobotics/road-runner).

## Installation

For more detailed instructions on getting Road Runner setup in your own project, see the [Road Runner README](https://github.com/acmerobotics/road-runner#core).

1. Download or clone this repo with `git clone https://github.com/acmerobotics/road-runner-quickstart`.

1. Open the project in Android Studio and build `TeamCode` like any other `ftc_app` project.

## Samples:

1. Passing pose data between opmodes

    Files:
    - [teamcode/drive/advanced/TeleOpJustLocalizer.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpJustLocalizer.java)
        
    - [teamcode/drive/advanced/AutoTransferPose.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/AutoTransferPose.java)
        
    - [teamcode/drive/advanced/PoseStorage.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/PoseStorage.java)

    If you wish to read your localizer's pose during teleop, you need to know where your initial
    pose is. If an initial pose is not set, the program will assume you start at x: 0, y: 0, and
    heading: 0, which is most likely not what you want. So, to know where you are teleop, you must
    know where you ended in auto. This sample explains how to do so via a static class,
    `PoseStorage`, that allows data to persist between opmodes. AutoTransferPose will write its
    pose estimate to `PoseStorage.currentPose`. Because this is a static field, it will persist
    between opmodes. Thus, a teleop afterwards is able to read from it and know where the autonomous
    opmode ended.

2. Road Runner in teleop - just the localizer
    
    File:
    - [teamcode/drive/advanced/TeleOpJustLocalizer.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpJustLocalizer.java)

    Example code demonstrating how one would read from their localizer in teleop. Utilizes a static
    class to pass data between opmodes. This sample reads from the `PoseStorage` static field to set
    an initial starting pose. An autonomous opmode should have written its last known pose to
    the `PoseStorage.currentPose` field. 

3. Road Runner in teleop - incorporating the drive class
   
   File:
       - [teamcode/drive/advanced/TeleOpDrive.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpDrive.java)

    Example code demonstrating how one would incorporate the `SampleMecanumDrive` class into their
    teleop, without the need for a separate robot class. Instead, this sample utilizes the drive
    class's kinematics and `setDrivePower()` function. This sample is essentially just a modified
    `LocalizationTest.java` with pose extraction from `PoseStorage` and additional comments.
    
4. Async following with FSM orchestration

   File:
       - [teamcode/drive/advanced/AsyncFollowingFSM.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/AsyncFollowingFSM.java)

    Example opmode describing how one utilizes async following in conjunction with finite state
    machines to orchestrate variable, multi-step movements. This allows for complex autonomous
    programs.

5. Breaking from a live trajectory

6. Automatic driving in teleop