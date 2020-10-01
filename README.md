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
        - [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpJustLocalizer.java](teamcode/drive/advanced/TeleOpJustLocalizer.java)
        - [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/AutoTransferPose.java](teamcode/drive/advanced/AutoTransferPose.java)
        - [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/PoseStorage.java](teamcode/drive/advanced/PoseStorage.java)

    If you wish to read your localizer's pose during teleop, you need to know where you start.
    This is because the localizer only measures relative position.
    So, to know where you are in teleop, you have to know where you ended in auto.
    This sample explains how to do so via a static class that allows data to persist between opmodes.

2. Road Runner in teleop - just the localizer
    
    File:
    [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpJustLocalizer.java](teamcode/drive/advanced/TeleOpJustLocalizer.java)

    Example code demonstrating how one would read from their localizer in teleop. Utilizes the
    passing data between opmodes example for the start pose.

3. Road Runner in teleop - incorporating the drive class
    
4. Async following with FSM orchestration

5. Breaking from a live trajectory