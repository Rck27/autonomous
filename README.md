# autonomous

# Autonomous Drone: Object Detection and Drop

## 🧩 Requirements
- [Gazebo Harmonic](https://gazebosim.org/home)  
- tambahi ken
---

## ✅ What Works
- Figure-8 mission  
- Downward-facing camera  
- World with runway, drop-point objects, and flight path visualization  
- MAVProxy set and connected  

---

## 🧠 To-Do List
- [ ] Add camera bridge (using ZMQ or other methods)  
- [ ] Integrate object detection code  
- [ ] Implement object drop mechanism  
- [ ] Tune PID for better flight performance  

---

## 🚀 How to Run

### -1. Pre requirements

install gz and  ardupilot sitl with distrobox

ardupilot-env -> for gz and ardupilot sitl

vision-env -> for vision and mission related task


example command to create distrobox env
```bash
distrobox create --name ardupilot-env --image ubuntu:22.04 --home /mnt/data/home-distrobox/ardupilot-env --init --additional-packages "git python3-pip nano systemd libpam-systemd"

```

### 1. Run SITL
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=127.0.0.1:14550 --out=127.0.0.1:14551
```

### 2. Run Gazebo
```bash
gz sim -v4 -r models/world.sdf
```

### 3. Run QGC or other ground control
set the port into 14551

### 4. Run the main.py


