# Fast\_Pathtracer

## Overview

Fast\_Pathtracer is a high-performance path tracer built for rendering realistic images using ray tracing techniques. It is inspired by the principles outlined in the "Ray Tracing in a Weekend" series and aims to deliver speed and visual fidelity.

This project requires several dependencies and specific build instructions to ensure proper functionality. The core focus of this project is to implement an **acceleration structure** to optimize the ray tracing process and significantly reduce rendering times. Follow the steps below to set up your development environment and successfully compile the project.

---

## Prerequisites

To build and run Fast\_Pathtracer, you will need the following tools and software installed on your system:

### 1. **ImageMagick**

ImageMagick is used to handle image conversion tasks. It allows the `magick` or `convert` commands to be run globally from the command line.

#### Installation Instructions (Windows)

1. **Using ****`winget`**

   - Open a terminal (Command Prompt or PowerShell).
   - Run the following command:
     ```powershell
     winget install ImageMagick
     ```
   - Verify the installation by running:
     ```cmd
     magick -version
     ```

2. **Manual Installation**

   - Download the installer from [ImageMagick Official Site](https://imagemagick.org/script/download.php).
   - Run the installer and ensure the option to "Add application directory to your system path" is checked.
   - Verify the installation by running the following command in a terminal:
     ```cmd
     magick -version
     ```

---

### 2. **CMake**

CMake is required to configure and generate the Visual Studio project files.

#### Installation Instructions (Windows)

1. Download the CMake installer from the [CMake Official Website](https://cmake.org/download/).
2. Run the installer and ensure that the option to add CMake to the system `PATH` is selected.
3. Verify the installation by running the following command in a terminal:
   ```cmd
   cmake --version
   ```

---

### 3. **Visual Studio 2022**

You will need the latest version of Visual Studio 2022 with C++ 17 support.

#### Installation Instructions (Windows)

1. Download the Visual Studio 2022 installer from [Visual Studio Official Website](https://visualstudio.microsoft.com/).
2. Select the **Desktop development with C++** workload during installation.
3. Ensure the following options are selected:
   - **C++ 17 support**
   - **MSVC v143 - VS 2022 C++ x64/x86 build tools**
   - **CMake Tools for Windows**
4. After the installation, verify that Visual Studio 2022 can be launched properly.

---

## Build Instructions

### 1. **Clone the Repository**

Start by cloning the Fast\_Pathtracer repository to your local system.

```cmd
git clone <repository_url>
cd Fast_Pathtracer
```

### 2. **Run CMake**

1. Open a terminal (Command Prompt, PowerShell, or Developer Command Prompt for Visual Studio).
2. Run the following command from the root directory of the Fast\_Pathtracer repository:
   ```cmd
   cmake -B build
   ```
   This will create a `build` directory in the root of the project and generate Visual Studio project files.

### 3. **Open the Project in Visual Studio 2022**

1. Open Visual Studio 2022.
2. Click on **File** > **Open** > **Folder**.
3. Select the `build` folder that was created by the CMake command.
4. Wait for Visual Studio to load the project files.

---

### 4. **Select the Executable to Run**

1. In Visual Studio 2022, locate the **Solution Explorer** (View > Solution Explorer if it’s not visible).
2. Identify the executable for the path tracer, which should be labeled something like:
   ```
   ray_tracing_the_next_week.exe
   ```
   This executable is the target for the compilation.

---

### 5. **Build the Project**

1. In Visual Studio, set the build configuration to **Release** for optimal performance.
   - Click on the dropdown at the top of the Visual Studio window (it will say "Debug" by default) and change it to **Release**.
2. Click **Build** > **Build Solution** or press `Ctrl + Shift + B`.
3. Wait for the build process to complete. If there are errors, make sure all dependencies are installed correctly and that you are using a supported compiler (like MSVC 2022).

---

### 6. **Run the Project**

1. Right-click the **ray\_tracing\_the\_next\_week.exe** target in the Solution Explorer.
2. Select **Set as Startup Project**.
3. Press **F5** or click the green play button to run the project.

---

## Performance Tips

1. **Multi-Core Compilation**

   - Visual Studio can compile using multiple cores. This speeds up the build process.
   - To enable multi-core compilation, go to **Tools** > **Options** > **Projects and Solutions** > **Build and Run**.
   - Set the "maximum number of parallel project builds" to the number of CPU cores available on your machine.

2. **Release Build**

   - Always use the **Release** build configuration for optimal performance.
   - The **Debug** configuration includes extra symbols for debugging, which slows down execution.

---

## Usage

Once the build is complete, you can run the `ray_tracing_the_next_week.exe` file directly from Visual Studio or from the command line. This executable will produce rendered images using ray tracing.

---

## Acceleration Structure

One of the key focuses of this project is to implement an **acceleration structure** to improve the performance of the ray tracing process. Acceleration structures, such as **Bounding Volume Hierarchies (BVH)** or **kd-trees**, help reduce the number of ray-primitive intersection tests, resulting in faster rendering times.


---

## Troubleshooting

**Problem:** "ImageMagick not found" or "magick command not recognized"

- **Solution:** Make sure ImageMagick is installed and its path is added to the system's `PATH` environment variable. Restart the terminal or system if necessary.

**Problem:** CMake errors "command not found"

- **Solution:** Ensure CMake is installed and its path is added to the system’s `PATH` variable.

**Problem:** "Unable to find Visual Studio" when running CMake

- **Solution:** Make sure Visual Studio 2022 is installed and that the **Desktop development with C++** workload is selected.

