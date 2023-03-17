# MTC ORI Collaboration 2022

## Beam Assembly Benchmark

### 1. Installation

Tested on ubuntu 22.04.02 LTS

1. Install Dependencies

java runtime environment
   - install via apt
   ```bash
   sudo apt install default-jre
   ```

clingo
   - Add potassco stable ppa to apt and install clingo
   ```bash
   sudo add-apt-repository  ppa:potassco/stable
   sudo apt update
   sudo apt install clingo
   ```

 SPARC
   - Clone SPARC to ASP translator repo (https://github.com/iensen/sparc/)
   - Set environment variable
     ```bash
     echo 'export SPARC_PATH=path/to/sparc/folder' >> ~/.bashrc
     ```

Python distutils
   - if not included with your python build can be installed using apt, we use python3.10 but this should also work for other versions.
   ```bash
   sudo apt install python3.10-distutils
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
   - Change to repo base directory
     ```bash
     cd MTC_ORI_Collab
     ```
   - Install python dependencies
     ```bash
     python3 -m pip install -r requirements.txt
     ```
   - Install python modules (may vary depening on your python install)
     ```bash
     python3 setup.py build
     sudo python3 setup.py install
     ```

### 2. Usage

1. Run example planner with
   ```bash
   cd MTC_ORI_Collab
   python3 sparc_planning/src/main.py
   ```
