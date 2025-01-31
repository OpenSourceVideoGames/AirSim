# Build Colosseum on macOS
  
**THIS IS NOT CURRENTLY SUPPORTED WITH COLOSSEUM AND MAY NOT WORK.**

Only macOS **Sequoia (15)** has currently been tested. Theoretically, Colosseum should work on all macOS versions.

We've two options - you can either build inside docker containers or your host machine.

## Docker

Please see instructions [here](docker_ubuntu.md)

## Host machine

### Pre-build Setup

#### Download Unreal Engine

1. [Download](https://www.unrealengine.com/download) the Epic Games Launcher. While the Unreal Engine is open source and free to download, registration is still required.
2. Run the Epic Games Launcher, open the `Library` tab on the left pane.
   Click on the `Add Versions` which should show the option to download **Unreal 5.4** as shown below. If you have multiple versions of Unreal installed then **make sure 5.4 is set to `current`** by clicking down arrow next to the Launch button for the version.

   **Note**: If you have UE 4.16 or older projects, please see the [upgrade guide](unreal_upgrade.md) to upgrade your projects.
   **Note**: Your project must be on a case-insensitive file system.

### Build Colosseum

- Clone Colosseum and build it:

```bash
# go to the folder where you clone GitHub projects
git clone https://github.com/OpenSourceVideoGames/AirSim.git
cd AirSim
```

By default Colosseum uses clang 16 to build for compatibility with UE 5.4. The setup script will install the right version of cmake, llvm, and eigen.

[Brew](https://brew.sh/) is required for building.

```bash
./setup.sh
./build.sh
# use ./build.sh --debug to build in debug mode
```

### Build Unreal Environment

Finally, you will need an Unreal project that hosts the environment for your vehicles. Colosseum comes with a built-in "BlocksV2 Environment" which you can use, or you can create your own. Please see [setting up Unreal Environment](unreal_proj.md) if you'd like to setup your own environment.

## How to Use AirSim

- Browse to `AirSim/Unreal/Environments/BlocksV2`.
- Run `./GenerateProjectFiles.sh <UE_PATH>` from the terminal, where `UE_PATH` is the path to the Unreal installation folder. (By default, this is `/Users/Shared/Epic\ Games/UE_5.4/`) The script creates an XCode workspace by the name BlocksV2 (Mac).xcworkspace.
- Open the XCode workspace, and press the Build and run button in the top left.
- After Unreal Editor loads, press Play button.

   **Note**: It will show error about nil unexpected - you can remove this part of code. (need help to pass flag to disable it by default)
   **Note**: After Build and Run it do not find BlocksV2.uproject - you can open editor from BlocksV2.uproject, but unfortunately you need to build it again :(
   **Note**: You can monitor errors during build here ~/Library/Application\ Support/Epic/UnrealBuildTool/Log.txt

See [Using APIs](apis.md) and [settings.json](settings.md) for various options available for Colosseum usage.

!!! tip
Go to 'Edit->Editor Preferences', in the 'Search' box type 'CPU' and ensure that the 'Use Less CPU when in Background' is unchecked.

### [Optional] Setup Remote Control (Multirotor Only)

A remote control is required, if you want to fly manually - see the [remote control setup](remote_control.md) for more details.

Alternatively, you can use [APIs](apis.md) for programmatic control or use the so-called [Computer Vision mode](image_apis.md) to move around using the keyboard.
 
