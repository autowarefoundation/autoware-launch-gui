# Autoware Launch GUI

Autoware Launch GUI is a Tauri / NextJS application designed to simplify the process of Launching Autoware Processes. Instead of relying on command prompts, users can now utilize a user-friendly graphical interface.

## Table of Contents

- [Dependencies](#dependencies)
  - [Installing Rust](#installing-rust)
  - [Installing Node.js](#installing-nodejs)
- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Contributing](#contributing)
- [Author](#author)
- [License](#license)

## Dependencies

To run or develop the Autoware Build GUI, you'll need to have both Rust and Node.js installed on your system.

### Installing Rust

1. To install Rust, run the following command:

   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs/ | sh
   ```

2. Follow the on-screen instructions to complete the installation.

3. Once installed, you can verify your Rust installation with:

   ```bash
   rustc --version
   ```
   *rustc 1.72.0 (5680fa18f 2023-08-23) is the one i'm currently on while developping this.*

### Installing Node.js

1. You can download and install Node.js from the [official website](https://nodejs.org/).

2. Alternatively, if you're using a package manager like `apt` for Ubuntu/Debian, you can install Node.js with:

   - For Ubuntu/Debian:

     ```bash
     sudo apt update
     sudo apt install nodejs
     ```

3. Verify your Node.js installation with:

   ```bash
   node --version
   ```
   *v16.20.0 is the one i'm currently on while developping this.*

4. It's also recommended to install `pnpm` as it's not included:

   ```bash
   npm install -g pnpm
   ```

Once Rust and Node.js are set up, you can proceed with the [Installation](#installation) steps mentioned above.

## Installation

### Using the .deb File

For most users, the easiest way to get started is by downloading and installing the provided `.deb` file.

### For Developers

If you're interested in developing additional features or want to run the project from source:

1. Clone the repository:

   ```bash
   git clone https://github.com/leo-drive/autoware-launch-gui.git
   ```

2. Navigate to the project directory:

   ```bash
   cd autoware-launch-gui
   ```

3. Install the required packages:

   ```bash
   pnpm i
   ```

4. Run the development version of the app:

   ```bash
   pnpm tauri dev
   ```

## Usage

For a comprehensive guide on how to use the Autoware Launch GUI, please refer to our demo video. In essence, the process involves:

## **Demo Video**
[![Autoware Launch GUI Demo](https://github-production-user-asset-6210df.s3.amazonaws.com/36904941/273592037-9bb7c83d-eb79-4991-a43c-16d1c5b2673d.png)](https://www.youtube.com/watch?v=iQEEct-pwpg&ab_channel=KhalilSelyan)

1. Launching the app.
2. Setting the path to the Autoware folder.
3. Selecting the desired launch file located in the `src/launcher/autoware_launch/autoware_launch/launch` folder.
4. Adding the necessary parameters, and any extra parameters you'd like to include in your launch command.
5. Launch Autoware

- Separately, you can also edit config param yaml files located in the `src/launcher/autoware_launch/autoware_launch/config` folder.

## Features

- [x] Launch Autoware with Custom Parameters depending on the selected launch file
- [x] Kill Autoware Processes
- [x] Edit Config Params YAML Files
- [x] View Current CPU/Memory Usage

## Contributing

We welcome contributions from the community! If you're interested in enhancing the Autoware Build GUI, please follow the installation instructions for developers and feel free to submit pull requests.

## Author

**Khalil Selyan** - Creator and maintainer of Autoware Build GUI.

- **GitHub**: <https://github.com/khalilselyan>
- **LinkedIn**: <https://linkedin.com/in/khalilselyan>
- **Email**: <khalil@leodrive.ai>

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
