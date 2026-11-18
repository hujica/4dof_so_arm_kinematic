# SO-ARM101: 4-DOF Velocity Kinematics & Control Console

**SO-ARM101** 4-DOF Robotic Manipulator အတွက် Python ဖြင့် ရေးသားထားသော Velocity Kinematics Control, 3D Visualization GUI နှင့် ROS 2 Bridge စနစ် ဖြစ်ပါသည်။ 

Feetech **STS3215** Serial Bus Servo များဖြင့် တိုက်ရိုက်ချိတ်ဆက် မောင်းနှင်နိုင်သကဲ့သို့ စက်ပစ္စည်း (Hardware) မရှိပါကလည်း **Simulation (Mock) Mode** ဖြင့် အလိုအလျောက် စမ်းသပ်အသုံးပြုနိုင်ပါသည်။

---

## အဓိက လုပ်ဆောင်ချက်များ (Key Features)

* **Velocity-Level Control (Resolved-Rate):** Jacobian Inverse (Damped Least Squares - DLS) နည်းပညာကို အသုံးပြု၍ Singularity အနီးတွင်ပင် ချောမွေ့စွာ Cartesian (X, Y, Z) ရွေ့လျားနိုင်ခြင်း။
* **Forward & Inverse Kinematics:** 4-DOF စက်ရုပ်လက်တံ၏ DH Parameters အပေါ် အခြေခံထားသော တွက်ချက်မှုများနှင့် Elbow-down IK စနစ်။
* **Interactive 3D GUI Console (Tkinter):**
  * Mouse ဖြင့် ဆွဲလှည့်ကြည့်ရှုနိုင်သော Real-time 3D Kinematic Visualizer နှင့် End-effector လမ်းကြောင်းပြ (Trail) စနစ်။
  * Cartesian Jog Pad (X, Y, Z ခလုတ်များ) နှင့် Joint Target Sliders များ။
  * စက္ကန့် ၁၀ စာ Joint Velocity ဂရပ်ဖ် (Real-time Velocity Plot)။
* **Auto Simulation Fallback:** `/dev/ttyACM0` တွင် Servo မတွေ့ပါက Simulation Mode သို့ အလိုအလျောက် ပြောင်းလဲပေးခြင်း။
* **ROS 2 Integration:** `/joint_states` (`sensor_msgs/msg/JointState`) topic သို့ Joint angles နှင့် velocities များကို အချိန်နှင့်တပြေးညီ ထုတ်လွှင့်ပေးခြင်း။
* **Safety Mechanisms:** 
  * Joint တစ်ခုချင်းစီအတွက် $\pm 60^\circ$ ကန့်သတ်ချက် (Safety Window)။
  * Emergency Stop (`Esc` ခလုတ် သို့မဟုတ် GUI ခလုတ်) နှင့် Servo ဆက်သွယ်မှု ပြတ်တောက်ပါက Auto E-Stop စနစ်။

---

## ဖိုင်ဖွဲ့စည်းပုံ (Project Structure)

```text
├── main.py                      # ပရိုဂရမ် စတင်ရန် Entry point (ROS 2 + Controller + GUI)
├── controller/
│   ├── __init__.py
│   └── arm_controller.py        # 50Hz Control Loop, Resolved-rate & Joint move logic
├── hardware/
│   └── sts3215_driver.py        # Feetech STS3215 Serial Driver နှင့် Simulation fallback
├── kinematics/
│   └── arm_kinematics.py        # Forward/Inverse Kinematics, DH Table, Jacobian & DLS
├── gui/
│   └── main_gui.py              # Tkinter 3D Visualizer, Velocity Plot နှင့် Control UI
└── ROS/
    └── ros2_bridge.py           # ROS 2 /joint_states publisher node
```

---

## လိုအပ်ချက်များ (Prerequisites & Dependencies)

* **Python 3.8+**
* **ROS 2** (Humble / Iron / Jazzy စသည်ဖြင့် `rclpy` နှင့် `sensor_msgs` ပါဝင်ရမည်)
* **Python Libraries:**
  * `numpy`
  * `tkinter` (Linux တွင် `sudo apt install python3-tk` ဖြင့် ထည့်သွင်းနိုင်သည်)
  * `scservo_sdk` (Feetech STS3215 Servo များ ချိတ်ဆက်အသုံးပြုရန်)

---

## တပ်ဆင်ခြင်းနှင့် အသုံးပြုပုံ (Installation & Usage)

### ၁။ လိုအပ်သော Package များ ထည့်သွင်းခြင်း
```bash
pip install numpy feetech-servo-sdk
sudo apt update && sudo apt install python3-tk
```

### ၂။ Hardware ချိတ်ဆက်ခြင်း (Optional)
* STS3215 Servo များ (ID `1` မှ `4` အထိ) ကို USB-to-TTL/Serial adapter မှတစ်ဆင့် ကွန်ပျူတာသို့ ချိတ်ဆက်ပါ။
* Serial Port ခွင့်ပြုချက် (Permission) ပေးပါ-
  ```bash
  sudo chmod 666 /dev/ttyACM0
  ```
* *(မှတ်ချက်- Port အမည် ပြောင်းလဲလိုပါက `main.py` အတွင်းရှိ `SERIAL_PORT = "/dev/ttyACM0"` နှင့် `BAUDRATE = 1000000` ကို ပြင်ဆင်နိုင်ပါသည်။)*

### ၃။ ပရိုဂရမ် စတင်မောင်းနှင်ခြင်း
ROS 2 environment ကို source လုပ်ပြီး `main.py` ကို run ပါ-
```bash
source /opt/ros/<your_ros2_distro>/setup.bash
python3 main.py
```

---

## Kinematic Parameters & Configuration

### DH Parameters (SO-ARM101)
| Joint ($i$) | $a_i$ ($\text{m}$) | $\alpha_i$ ($\text{rad}$) | $d_i$ ($\text{m}$) | ဖော်ပြချက် |
| :---: | :---: | :---: | :---: | :--- |
| **1** | `0.042` | $+\pi/2$ | `0.030` | Base Yaw |
| **2** | `0.120` | $0$ | $0$ | Shoulder Pitch |
| **3** | `0.164` | $0$ | $0$ | Elbow Pitch ($\text{offset}: -90^\circ$) |
| **4** | `0.000` | $0$ | $0$ | Wrist Pitch (Orientation-only) |

### Servo လားရာ ချိန်ညှိခြင်း (Direction Calibration)
Servo တပ်ဆင်ပုံ ပြောင်းပြန်ဖြစ်နေပါက `hardware/sts3215_driver.py` အတွင်းရှိ `servo_directions` တွင် သက်ဆိုင်ရာ Joint ID ၏ တန်ဖိုးကို `1` သို့မဟုတ် `-1` သို့ ပြောင်းလဲနိုင်ပါသည်-
```python
self.servo_directions = {1: -1, 2: -1, 3: -1, 4: 1}
```

---

## အသုံးပြုပုံ လမ်းညွှန် (Controls)

* **Cartesian Jog:** GUI ရှိ `+X / -X`, `+Y / -Y`, `+Z / -Z` ခလုတ်များကို ဖိထားခြင်းဖြင့် End-effector ကို မျဉ်းဖြောင့်အတိုင်း ရွှေ့နိုင်သည်။
* **HOME:** စက်ရုပ်လက်တံကို မူလ L-shape ($[0^\circ, 0^\circ, 0^\circ, 0^\circ]$ / $1024\text{ steps}$) အနေအထားသို့ ပြန်ပို့ပေးသည်။
* **Velocity Input:** Linear tip speed ($5 - 150\text{ mm/s}$) နှင့် Max joint speed ($5 - 120^\circ/\text{s}$) တို့ကို စိတ်ကြိုက် ချိန်ညှိနိုင်သည်။
* **Emergency Stop:** `Esc` ခလုတ်ကို နှိပ်ခြင်း သို့မဟုတ် အနီရောင် **EMERGENCY STOP** ခလုတ်ကို နှိပ်ခြင်းဖြင့် Servo torque များကို ချက်ချင်း ဖြတ်တောက်နိုင်သည်။ ပြန်လည်အသုံးပြုရန် **RESET E-STOP** ကို နှိပ်ပါ။