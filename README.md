# Digital Phase-Locked Loop (DPLL)

This repository contains a **simulation-verified Digital Phase-Locked Loop (DPLL)** implementation.  
The design demonstrates stable lock behavior in simulation (lock drift resolved).  

⚠️ **Important Notice**  
- This release is **for simulation and evaluation purposes only**.  
- It is **not synthesizable**.  
- The included **CORDIC core is provided as-is and should be treated as a simulation-only model**.  
- A synthesizable and commercial-grade version is under development.

---

## 📌 Features
- Digital Phase Detector (PD)  
- Loop Filter  
- Numerically Controlled Oscillator (NCO)  
- CORDIC-based sine/cosine generation (**simulation-only**)  
- Verified lock acquisition and stability in simulation  

---

## 📊 Current Status
- ✅ Simulation-verified in HDL simulator (e.g., ModelSim, Icarus Verilog, GHDL)  
- ✅ Lock drift issue resolved  
- ❌ Not yet synthesizable  
- 🚧 Roadmap includes synthesizable FPGA/ASIC-ready release  

---

## 📂 Repository Structure
src/ → DPLL source files (top-level + modules)
tb/ → Testbenches
sim_results/ → Simulation results (waveforms, plots)
docs/ → Block diagrams, notes

Copy code
.
