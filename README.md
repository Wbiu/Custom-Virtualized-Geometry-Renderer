# Custom-Virtualized-Geometry-Renderer 💻
A custom Virtualized Geometry LOD Rendering Engine : cluster-based real-time mesh simplification and LOD streaming system inspired by UE5's Nanite. This engine dynamically switches between multiple Levels of Detail using a virtualized geometry hierarchy, supporting view-dependent streaming and cluster-based simplification.

## ✨ Features
- ♻️ **Dynamic LOD transitions** via a **graph** to reduce cluster overdraw  
- 🧩 **Cluster-based mesh partitioning** for efficient culling and updates  
- 🚀 **View-dependent streaming** with bounding volume–based LOD switching  
- 🍭 **Debug views:** render **clusters** or **triangles (primitives)**
---
## 🔧 Technical Highlights
- 🛠️ Built from the ground up
- 📦 Minimal external dependencies: [GLFW](https://www.glfw.org/) and [Dear ImGui](https://github.com/ocornut/imgui)
- ⏭️ Custom SIMD (SSE/AVX/FMA) math for vector ops
- 🌲 **BVH** (Bounding Volume Hierarchy) for cluster spatial organization
- ✂️ **QEM** (Quadric Error Metrics) for geometry simplification
- 🎯 Per-cluster frustum culling
- 📥 View-frustum based streaming of cluster geometry
- 🧵 **Multi-threaded** mesh processing; rendering on its own thread
- 🌋 **Vulkan RHI** backend
- ⚡️ Ready to support **ray tracing** via BVH
---
## 🎮 Ideal For
- Researchers exploring real-time mesh simplification  
- Engine developers experimenting with virtualized geometry  
- Anyone inspired by **Nanite** and interested in similar techniques (here using **Vulkan**)
---
## 🎥 Demo
A packaged demo app is available under **Releases**. Example meshes are included.  
**Note:** not every mesh will work—see the project note below for details.
❗️🛡️ On some system **Windows Defender**  will be alerted. So to run click on **More Info** -> **Run anyway**.

### Triangle View
![primitive_view](https://github.com/user-attachments/assets/a9ed94d2-7118-47bd-86d3-0f5ba690bb0b)

### Cluster View
![cluster_view](https://github.com/user-attachments/assets/84c4f374-0396-4df2-bed2-5ee1261dde27)

---
## ⚠️🚧⚠️ Important Note to Project
This project was developed as part of my degree and is **not** intended to be a production-ready, bulletproof architecture.  
Turning it into one would require re-architecting core data structures (current ones use more memory than ideal) and improving robustness for a wider range of mesh topologies. Some meshes—especially non-manifold or highly irregular ones—may fail to process or render correctly.

## ✅ Requirements
**OS**
- Windows 10 or 11, 64-bit
 
**GPU & Drivers**
- A Vulkan-capable discrete or integrated GPU
- Vulkan **1.4** driver recommended (no SDK required to run)

**CPU**
- x86_64 CPU
- **AVX2 + FMA recommended** (this build uses SIMD paths for speed)
- SSE2-only CPUs may work with a non-SIMD build (slower). If you need that, build with SIMD disabled.



