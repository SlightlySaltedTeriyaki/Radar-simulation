# RadarSim 📡

A high-performance C++ simulation pipeline for real-time radar signal processing and target tracking.

This project implements a complete, lightweight digital signal processing (DSP) and tracking chain designed for low latency, zero heap allocation in the hot path, and modularity.

---

## 🏗️ Architecture & Data Flow

The simulation processes radar sweeps sequentially. Each sweep represents a single vector of range bins, flowing through the following pipeline:

```mermaid
graph TD
    A[SignalGenerator] -- Sweep --::std::array--> B[RingBuffer]
    B -- Sweep --> C[SignalProcessor]
    C -- CA-CFAR Detections --::std::vector--> D[TargetTracker]
    D -- Updates Tracks --> E[Active Tracks Table]
    
    style A fill:#1a73e8,stroke:#fff,stroke-width:2px,color:#fff
    style B fill:#f29900,stroke:#fff,stroke-width:2px,color:#fff
    style C fill:#188038,stroke:#fff,stroke-width:2px,color:#fff
    style D fill:#d93025,stroke:#fff,stroke-width:2px,color:#fff
    style E fill:#80868b,stroke:#fff,stroke-width:2px,color:#fff
```

1. **`SignalGenerator`**: Generates Gaussian-noise-corrupted sweeps containing simulated target signatures.
2. **`RingBuffer`**: A static, lock-free template queue that decouples production and consumption without invoking heap allocation.
3. **`SignalProcessor`**: Implements a **Constant False Alarm Rate (CA-CFAR)** algorithm to dynamically compute the local noise floor and threshold targets.
4. **`TargetTracker`**: Associates new detections with existing tracks using a weighted Exponential Moving Average (EMA) and handles track lifecycles (creation, confirmation, and dropouts).

---

## 🛠️ Key Components

### 1. Zero-Allocation `RingBuffer`
A circular buffer template with a compile-time fixed size $N$. It uses bitwise logic for optimal modulo indexing:
* Enforces at compile-time that capacity $N$ is a power of two to prevent runtime boundary errors.
* Avoids dynamic memory allocations to prevent CPU cache misses and heap fragmentation.

### 2. CA-CFAR Target Detection
The Constant False Alarm Rate processor dynamically scales the detection threshold based on the surrounding noise environment:
* **Guard Cells**: Ignores immediate neighbors of the Cell Under Test (CUT) to avoid target energy spilling into the noise estimation.
* **Reference Cells**: Computes the mean power level of background noise from further bins.

### 3. EMA Target Association & Tracking
The tracking component filters out momentary noise spikes:
* Uses an Exponential Moving Average ($\alpha = 0.3$) to smooth target range coordinates.
* Tracks are deleted dynamically once they exceed a defined frame miss threshold.

---

## 🚀 Getting Started

### Prerequisites
* **CMake** (v3.16 or higher)
* **C++17 Compatible Compiler** (MSVC, GCC, or Clang)

### Building the Project
Clone the repository and build using CMake in **Release** mode to enable maximum compiler optimization:

```bash
# Configure the build directory
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Compile the target binaries
cmake --build build --config Release
```

### Running the Simulator
Run the main performance simulation benchmark:

```bash
# On Windows
.\build\Release\radarsim.exe

# On Linux/macOS
./build/Release/radarsim
```

### Running Unit Tests
Validate pipeline components and assertions:

```bash
# On Windows
.\build\Release\tests.exe

# On Linux/macOS
./build/Release/tests
```

---

## 📊 Example Benchmark Metrics

Running the release executable executes **10,000 sweeps** and prints out raw performance telemetry:

```text
Sweepy:      10000
Cas celkem:  2946972 us
Throughput:  3393 sweepy/s
Latence:     294.7 us/sweep
Aktivni tracky: 636
```

---

## 📈 Future Optimization Roadmap

The project is structured to encourage experimentation with high-performance C++ and digital signal processing. Here are the planned optimization areas:

### 🛡️ DSP Correctness: Envelope Detection
Adding an envelope detection pre-processing step (`std::abs(signal)`) to ensure the noise average sum remains positive, reducing the false alarm rate and narrowing active tracks down to true positive signatures.

### ⚡ Algorithmic Upgrade: $O(1)$ Sliding Window CFAR
Transitioning the nested search in the CA-CFAR loop to a rolling sliding window sum. This drops computational complexity from $O(N \cdot M)$ to $O(N)$ per sweep, yielding a $\sim 10\times$ throughput speedup.

### ⛓️ Concurrent Pipeline & Thread Safety
Making the `RingBuffer` thread-safe (via lock-free atomics or mutex-guarded condition variables) and running the `SignalGenerator` in a dedicated producer thread while the consumer thread processes the CFAR & tracking algorithms in parallel.
