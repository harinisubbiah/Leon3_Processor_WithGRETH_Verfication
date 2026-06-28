# Development of Memory Interface Bridge for AMBA-Based Ethernet MAC

Functional Verification, ASIC Synthesis and Formal Verification of an AMBA Memory Interface Bridge integrated with the LEON3 Processor and GRETH Ethernet MAC.

---

## Project Overview

This project was completed as part of an ASIC Design Internship and focuses on the verification and implementation of a custom bridge module connecting:

- LEON3 Processor
- GRETH Ethernet MAC
- AMBA AHB Bus
- APB Bus
- External SRAM

The work includes:

- Functional verification using VHDL testbenches
- Bare-metal C software verification
- Code coverage analysis
- RTL linting
- ASIC synthesis
- Formal equivalence checking

---

## Project Features

- LEON3 + GRETH verification
- Memory Interface Bridge
- APB Bridge Verification
- AHB Burst Verification
- SRAM Memory Model
- Protocol-compliant AHB Master
- Synopsys VCS Simulation
- Synopsys Design Compiler
- Synopsys Formality
- Synopsys LEDA

---

## Verification Flow for Bridge Module

<img width="919" height="710" alt="image" src="https://github.com/user-attachments/assets/08842e87-ebb5-4874-8caa-5413e6a3d903" />

---

## Testbenches

### 1. Simple_Test.vhd

Verifies:

- Processor Read
- Processor Write
- AHB Read
- AHB Write
- Reset Behaviour

---

### 2. Burst_Final.vhd

Verifies:

- SINGLE
- INCR
- WRAP4
- INCR4
- WRAP8
- INCR8
- WRAP16
- INCR16

Additional Tests:

- BUSY Cycles
- Early Burst Termination
- Persistent SRAM Model

---

## Software Verification

The GRETH MAC was verified using a bare-metal C application.

Implemented tests include:

- Register Access
- PHY MDIO Read
- PHY Loopback
- Single Frame Loopback
- Multi Frame Stress Test

---

## Tools Used

| Tool | Purpose |
|-------|----------|
| Synopsys VCS | RTL Simulation |
| Synopsys LEDA | Lint Analysis |
| Synopsys Design Compiler | Logic Synthesis |
| Synopsys Formality | Equivalence Checking |
| SPARC Gaisler GCC | Cross Compilation |

---

## Major Results
- 21/21 GRETH software assertions passed
- Verified all AMBA burst types
- Identified and corrected multiple RTL defects
- Successful lint analysis
- Successful synthesis
- Successful RTL-to-gate formal equivalence
---

## Future Improvements

- AHB Error Response (HRESP) Verification
- Address Range Protection
- Gate-Level Simulation
- Multi-Queue GRETH Verification
- UVM-based Verification Environment

---

## Author

**Harini S**

Memory Interface Bridge Verification Project

2026
