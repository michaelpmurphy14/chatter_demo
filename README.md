# chatter_demo

A minimal ROS Melodic **talker/listener** demo, extended with Docker support for the listener, plus a roadmap of next steps for camera, vision, and cloud integration.

---

## 🔧 Prerequisites

- **Jetson Nano** running Ubuntu 18.04 + ROS Melodic  
- **Desktop or VM** with Docker (for the listener container)  
- Network connectivity between devices (e.g. `192.168.68.58:11311`)

---

## ⚙️ Quickstart

### 1. Clone & build on the Jetson Nano

```bash
# on the Nano
cd ~/ros_ws/src
git clone https://github.com/yourusername/chatter_demo.git
cd ~/ros_ws
catkin_make                # or 'catkin build'
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.68.58:11311
export ROS_IP=192.168.68.58
```

### 2. Run the talker

#### Open Tab A on the Nano:

```bash
roscore
```

#### Open Tab B on the Nano:

```bash
source ~/ros_ws/devel/setup.bash
rosrun chatter_demo talker.py
```
#### You’ll see:

```bash
[INFO] … Publishing: hello world 0
[INFO] … Publishing: hello world 1
…
```
### 3. Run the listener via Docker on your desktop

```bash
# on your desktop, from the repo root:
docker build -t chatter_demo-listener .

# then:
docker run --rm --network host chatter_demo-listener
```
You’ll see:

```bash
[INFO] … I heard: hello world 0
[INFO] … I heard: hello world 1
…
```

## 🚀 Next Steps Roadmap
- [x] Arducam Integration

  * [x] Install & test driver (v4l2-ctl, OpenCV)
  
  * [x] Add a ROS camera node publishing to /camera/image_raw
  
- [ ] Basic Vision Node

  * [ ] New package vision_demo
  
  * [ ] Subscribe to /camera/image_raw, apply OpenCV processing (e.g. grayscale, edges)
  
  * [ ] Publish results or status topic

- [ ] AWS IoT Core Bridge

  * [ ] Write a ROS node using AWS IoT Python/C++ SDK
  
  * [ ] Subscribe to vision/status topic, publish JSON over MQTT
  
  * [ ] Provision certificates on the Nano securely
  
- [ ] Containerize Full Pipeline
  
  * [ ] Multi‐stage Docker on Nano: camera + vision + MQTT bridge
  
  * [ ] Extend desktop listener container to visualize or log data
  
  * [ ] Add GitHub Actions to build & test Docker images
  
- [ ] Security & Compliance (CMMC 2.0)

  * [ ] Review certificate storage & firewall rules
  
  * [ ] Document architecture & data flows in SECURITY.md
  
  * [ ] Apply least‐privilege IAM roles for AWS access
  
- [ ] Performance Tuning

  * [ ] Profile with tegrastats on Nano
  
  * [ ] Optimize vision node; consider TensorRT or DeepStream for ML inference
  
- [ ] Polish & Showcase

  * [ ] Finalize README.md with full architecture diagram & examples


### 📄 License

    Specify your license here, e.g. MIT


        [ ] Write a blog post or slide deck for interviews

 
