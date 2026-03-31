# 2D Rocket Simulation

A real-time 2D rocket physics simulation using SFML for graphics and Eigen for mathematics.

## Features (Planned)
- Real-time physics simulation with gravity
- Controllable rocket with rotatable engine
- Adjustable thrust levels
- Visualization using SFML
- Linear algebra calculations with Eigen

## Current Status

### ✅ Completed
- CMake build system configured
- MinGW toolchain integrated
- Project structure created
- Basic build/compile workflow verified

### ⏳ In Progress
- Installing SFML 2.6+
- Installing Eigen 3.4+

### ❌ TODO
- Rocket physics engine implementation
- Render system with SFML
- User input controls
- Simulation visualization

## Quick Start

### Prerequisites
- CMake 3.16+ 
- g++ / GCC 14.2+
- mingw32-make

All are already installed and working!

### Building

```bash
cd build
mingw32-make
./RocketSim.exe
```

### Installing Required Libraries

#### Method 1: vcpkg (Recommended)
```bash
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
.\vcpkg install sfml:x64-windows-static eigen3:x64-windows
```

Then rebuild:
```bash
cd ..\2D_RocketSim\build
rm CMakeCache.txt
cmake .. -G "Unix Makefiles" \
  -DCMAKE_MAKE_PROGRAM="mingw32-make" \
  -DCMAKE_CXX_COMPILER="g++" \
  -DCMAKE_C_COMPILER="gcc" \
  -DCMAKE_TOOLCHAIN_FILE="<vcpkg_path>/scripts/buildsystems/vcpkg.cmake"
mingw32-make clean
mingw32-make
```

#### Method 2: Download Binaries

**SFML:**
1. Download: http://www.sfml-dev.org/download/sfml/2.6.0/ (GCC 13.1.0 MinGW (SEH) - 64-bit)
2. Extract to: `C:\Libraries\SFML`
3. Update CMake:
   ```bash
   cmake .. -DSFML_DIR="C:\Libraries\SFML\lib\cmake\SFML"
   ```

**Eigen:**
1. Download: https://gitlab.com/libeigen/eigen/-/releases
2. Extract to: `C:\Libraries\Eigen`
3. Update CMake:
   ```bash
   cmake .. -DEigen3_DIR="C:\Libraries\Eigen\share\eigen3\cmake"
   ```

#### Method 3: Automated Script
Run the provided setup script:
```cmd
install_libraries.bat
```

## Project Structure

```
2D_RocketSim/
├── CMakeLists.txt           # Build configuration
├── README.md                # This file
├── SETUP.md                 # Detailed setup instructions
├── install_libraries.bat    # Library installation helper
├── include/                 # Header files
│   └── rocket.h             # (To be created)
├── src/
│   ├── main.cpp             # Application entry point
│   ├── rocket.cpp           # (To be created) Rocket physics
│   └── simulation.cpp       # (To be created) Physics engine
└── build/                   # Build output directory
    ├── Makefile
    ├── CMakeFiles/
    └── RocketSim.exe        # Compiled executable
```

## Architecture

### Planned Components

**Rocket Physics Engine**
- State vector: position, velocity, angle, angular velocity
- Forces: gravity, engine thrust
- Integration: Runge-Kutta 4th order

**Rendering System**
- SFML Graphics for visualization
- Real-time position and rotation updates
- UI for thrust and angle controls

**Input System**
- Keyboard controls for thrust adjustment
- Mouse/keyboard for engine angle control
- Real-time parameter adjustment

## Development Notes

- C++ Standard: C++17
- Build System: CMake 3.16+
- Compiler: MinGW GCC 14.2.0
- Math Library: Eigen 3.4+
- Graphics: SFML 2.6+

## Build Commands

```bash
# Configure
cd build
cmake .. -G "Unix Makefiles" -DCMAKE_MAKE_PROGRAM="mingw32-make" -DCMAKE_CXX_COMPILER="g++" -DCMAKE_C_COMPILER="gcc"

# Build
mingw32-make

# Clean rebuild
mingw32-make clean
mingw32-make

# Run
./RocketSim.exe
```

## Next Steps

1. **Install SFML & Eigen** - Use one of the methods above
2. **Verify Installation** - Rebuild and test
3. **Implement Physics Engine** - Create rocket simulation logic
4. **Add Graphics** - Render rocket and trajectory
5. **Controller Input** - Implement user controls

## Troubleshooting

### CMake cannot find SFML/Eigen
- Ensure libraries are installed to expected locations
- Provide explicit paths to CMake: `-DSFML_DIR=...`
- Check `CMake Error.log` in build directory

### Linker errors
- Verify libraries were built for the same architecture (x64)
- Check library paths are correct
- Ensure SFML dependencies (dependencies like libfreetype, etc.) are available

### Runtime errors
- Check that SFML DLLs are in PATH or build directory
- Verify graphics are properly initialized

## Resources

- [SFML Documentation](https://www.sfml-dev.org/documentation/2.6/)
- [Eigen Documentation](https://eigen.tuxfamily.org/dox/)
- [CMake Documentation](https://cmake.org/documentation/)
- [MinGW Guide](https://www.mingw-w64.org/)

## License

MIT License - See LICENSE file (to be created)
