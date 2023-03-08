# MTC ORI Collaboration 2022

## Beam Assembly Benchmark

### 1. Installation

1. Install SPARC

   - Clone SPARC to ASP translator repo (https://github.com/iensen/sparc/)
   - Follow instructions in SPARC user manual (https://github.com/iensen/sparc/blob/master/User_Manual/Sparc_Manual.pdf) to install JRE and clingo
   - Set environment variable
     ```bash
     echo 'export SPARC_PATH=path/to/sparc/folder' >> ~/.bashrc
     ```

2. Download this Repo and configure
   - Clone this repo
     ```bash
     git clone https://github.com/M-A-Robson/MTC_ORI_Collab.git
     ```
   - Set PLANNER_PATH environment variable
     ```bash
     echo 'export PLANNER_PATH=path/to/repo/folder' >> ~/.bashrc
     ```
   - Install python modules (may vary depening on your python install)
     ```bash
     cd MTC_ORI_Collab
     python3 setup.py build
     sudo python3 setup.py install
     ```

### 2. Usage

1. Run example planner with
   ```bash
   cd MTC_ORI_Collab
   python3 sparc_planning/src/main.py
   ```
