 #### This project was developed during the Lerobot Global Hackaton`for developing a pick and place task for S0-101 robot using SMOLVLA


https://github.com/user-attachments/assets/29bad428-616f-4c3c-82f6-d50691905df2



---

````markdown
# LeRobot: Setup, Calibration, and Fine-Tuning Guide

Welcome to the LeRobot setup guide! This README will walk you through the full process:

1. [Assemble Your Robot](#assemble-your-robot)
2. [Calibrate the Robot](#calibrate-the-robot)
3. [Fine-Tune SmolVLA](#fine-tune-smolvla)

---

## 🛠 Assemble Your Robot

### 1. Bill of Materials and 3D Printing
- Refer to the [Installation Guide](#) for a complete bill of materials.
- Links to source parts and STL files are included.
- First time 3D printing? Look for printer advice in the guide or seek 3D printing services.

### 2. Install LeRobot

To get started, install LeRobot:

```bash
pip install -e ".[feetech]"
````

Follow the [Installation Guide](#) for detailed steps.

---

### 3. Step-by-Step Assembly

#### Motors Overview

| Axis                | Motor | Gear Ratio |
| ------------------- | ----- | ---------- |
| Base / Shoulder Pan | 1     | 1 / 191    |
| Shoulder Lift       | 2     | 1 / 345    |
| Elbow Flex          | 3     | 1 / 191    |
| Wrist Flex          | 4     | 1 / 147    |
| Wrist Roll          | 5     | 1 / 147    |
| Gripper             | 6     | 1 / 147    |

#### Prepare Printed Parts

* Remove support material using a small screwdriver or precision tools.

#### Joint Assembly Instructions

**Joint 1**

* Insert motor 1 into base and secure with 4x M2x6mm screws.
* Slide and fasten motor holder, install horns with M3x6mm screw.
* Attach and secure the shoulder with 4x M3x6mm screws (top + bottom).
* Add the shoulder motor holder.

**Joint 2**

* Insert motor 2 from the top, secure with 4x M2x6mm screws.
* Install horns with M3x6mm screw.
* Attach upper arm with 4x M3x6mm screws on each side.

**Joint 3**

* Insert motor 3 and secure with 4x M2x6mm screws.
* Install horns with M3x6mm screw.
* Connect forearm with 4x M3x6mm screws on each side.

**Joint 4**

* Slide in motor holder and motor 4.
* Fasten with 4x M2x6mm screws.
* Attach horn with M3x6mm screw.

**Joint 5**

* Insert motor 5 into wrist holder and fasten with 2x M2x6mm screws.
* Install one horn with M3x6mm screw.
* Secure wrist to motor 4 using 4x M3x6mm screws on both sides.

**Gripper**

* Attach gripper to motor horn with 4x M3x6mm screws.
* Insert motor 6 and secure with 2x M2x6mm screws.
* Attach horn with M3x6mm screw.
* Install gripper claws with 4x M3x6mm screws on each side.

---

### 4. Configure Motors

#### a. Find USB Port

Run:

```bash
python -m lerobot.find_port
```

Follow instructions to identify the port for each controller (leader/follower).

#### b. Set Motor IDs and Baudrates

Run for **Follower**:

```bash
python -m lerobot.setup_motors \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodemXXXX  # <- Use actual port found above
```

Run for **Leader**:

```bash
python -m lerobot.setup_motors \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodemYYYY  # <- Use actual port
```

> ⚠️ Connect only one motor at a time as prompted. Repeat the procedure for each joint motor.

---

## 🧭 Calibrate the Robot

### Why Calibrate?

Calibration ensures the leader and follower arms share joint alignment, critical for teleoperation and learning-based control.

### Follower Calibration

```bash
python -m lerobot.calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodemXXXX \
  --robot.id=my_awesome_follower_arm
```

### Leader Calibration

```bash
python -m lerobot.calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodemYYYY \
  --teleop.id=my_awesome_leader_arm
```

> Follow on-screen instructions and reference the calibration video for joint positioning and full-range movement.

---

## 🎯 Fine-Tune SmolVLA

SmolVLA is Hugging Face’s lightweight robotics foundation model. Fine-tune it on your dataset for task-specific performance.

### 1. Set Up Your Environment

Install SmolVLA dependencies:

```bash
pip install -e ".[smolvla]"
```

---

### 2. Record Your Dataset

We recommend \~50 episodes of your task. Follow the `Recording a Dataset` guide. Example: for a cube pick-place task, vary cube positions across episodes.

> Reference Dataset: [SVLA SO100 PickPlace](#)

---

### 3. Fine-Tune the Model

```bash
cd lerobot && python -m lerobot.scripts.train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=${HF_USER}/mydataset \
  --batch_size=64 \
  --steps=20000 \
  --output_dir=outputs/train/my_smolvla \
  --job_name=my_smolvla_training \
  --policy.device=cuda \
  --wandb.enable=true
```

> 🧠 Tuning tip: Adjust batch size and steps based on performance and GPU capacity. Use `--help` for all available options.

---

### 4. Evaluate Finetuned Model

Once logged in to the Hugging Face Hub, run:

```bash
python -m lerobot.record \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=my_blue_follower_arm \
  --robot.cameras="{ front: {type: opencv, index_or_path: 8, width: 640, height: 480, fps: 30}}" \
  --dataset.single_task="Grasp a lego block and put it in the bin." \
  --dataset.repo_id=${HF_USER}/eval_DATASET_NAME_test \
  --dataset.episode_time_s=50 \
  --dataset.num_episodes=10 \
  --policy.path=HF_USER/FINETUNE_MODEL_NAME
```

Optional: Add `--teleop.*` arguments to use leader arm for teleoperation during evaluation.

---

## 📌 Notes

* Ensure motors are uniquely identified and properly wired.
* Calibration is critical for motion fidelity.
* Dataset size and diversity matter significantly for model generalization.
* Always verify USB port mappings before running scripts.

---

## 🧩 Additional Resources

* [Installation Guide](#)
* [Calibration Video](#)
* [SmolVLA Paper & Datasets](#)
* [Hugging Face Model Hub](https://huggingface.co)

---

Happy building! 🤖


```
