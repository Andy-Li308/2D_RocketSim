# 2D Rocket Simulation

A real-time 2D rocket landing simulation with full-state feedback LQR control, using SFML for visualization and Eigen for linear algebra. The simulation demonstrates how different LQR gain profiles affect rocket landing behavior.

## Overview

- **Physics:** Simulates a 2D rigid-body rocket with 6-DOF (x, y, θ and their derivatives), gravity, thrust, and gimbaled engine.
- **Control:** Uses a Linear Quadratic Regulator (LQR) for full-state feedback. Two profiles are available:
  - **Slow:** Gentle, low-effort control.
  - **Fast:** Aggressive, rapid stabilization.
- **Visualization:** Real-time graphics with SFML, showing rocket state and control actions.

## How It Works

1. **LQR Gain Generation:**  
   Run the Python script to generate LQR gain matrices for both profiles:
   ```
   python scripts/compute_lqr_gains.py
   ```
   This produces `lqr_gains_slow.json` and `lqr_gains_fast.json`.

2. **Build the Project:**  
   - Prerequisites: CMake 3.16+, GCC 14+, SFML 3.0+, Eigen 5.0+.
   - Configure and build:
     ```
     cd build
     cmake ..
     cmake --build .
     ```

3. **Run the Simulation:**  
   From the `build` directory:
   ```
   ./RocketSim.exe slow   # Gentle controller
   ./RocketSim.exe fast   # Aggressive controller
   ```
   The controller profile determines which gain file is loaded.

## Project Structure

```
2D_RocketSim/
├── CMakeLists.txt
├── include/
│   ├── Controller.h
│   ├── Rocket.h
│   └── Visualization.h
├── lqr_gains_fast.json
├── lqr_gains_slow.json
├── scripts/
│   └── compute_lqr_gains.py
├── src/
│   ├── Controller.cpp
│   ├── main.cpp
│   ├── Rocket.cpp
│   └── Visualization.cpp
└── build/
```

- **src/**: C++ source files for simulation, control, and rendering.
- **include/**: C++ headers.
- **scripts/**: Python script for generating LQR gains.
- **lqr_gains_*.json**: Controller gain files (auto-generated).
- **build/**: Build output directory.

## Notes

- Only `lqr_gains_slow.json` and `lqr_gains_fast.json` are used.  
- The simulation window displays the rocket, ground, and control actions in real time.
- Adjust LQR weights in `scripts/compute_lqr_gains.py` to experiment with controller behavior.
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
