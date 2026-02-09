### **Chapter 1: Physical AI**

#### **1.1 Physical AI: Definition and Distinction from Digital AI**

* **Key Concepts**:

  * **Physical AI**: Physical AI refers to artificial intelligence systems that integrate AI algorithms with the ability to interact physically with the world through sensors, actuators, and robotic components. Unlike traditional AI, which is purely computational, physical AI has the ability to perceive and act in the real world, mimicking human or animal-like behaviors.
  * **Digital AI**: Digital AI operates within digital systems, processing data and making decisions based on algorithms without physical interaction with the environment. This can include software-based systems like **ChatGPT**, **image recognition** models, or other machine learning models that process input data and provide output without physical embodiment.
  * **Distinction**:

    * Digital AI is confined to virtual or computational environments and is often used for tasks like data analysis, natural language processing, and pattern recognition.
    * Physical AI, on the other hand, includes **robots** or devices that can interact physically with their environment, such as humanoid robots, autonomous vehicles, and smart machines that use sensors and actuators to perceive and interact with the real world.

* **Key Facts**:

  * **NVIDIA** frames **Physical AI** as the **embodiment of AI** into real-world systems, where it is not only about making decisions but also about interacting with and influencing the physical environment.
  * **Google DeepMind** advocates for the need to integrate AI with physical systems, allowing for AI to interact and learn from the world, as opposed to existing solely in digital environments.
  * Physical AI systems combine **machine learning models** with **robotic hardware** to create more autonomous and adaptive systems.

* **References**:

  * **NVIDIA**: "Physical AI: AI Meets Robotics" (NVIDIA Blog)
  * **Google DeepMind**: "Embodied Intelligence: Bridging Digital AI and Physical Systems" (DeepMind Research)

---

#### **1.2 Embodied Intelligence Theory**

* **Key Concepts**:

  * **Embodied Intelligence**: This theory, first introduced by **Rodney Brooks**, asserts that intelligence is not merely computational but is distributed throughout the body. Instead of relying on abstract reasoning, embodied intelligence suggests that cognition arises from interactions between the body and the environment. Robots with embodied intelligence learn by acting and perceiving in the real world.
  * **Morphological Computation**: The concept of **morphological computation** suggests that the structure of the robot's body plays a crucial role in how it processes information. This means that the design and geometry of the robot's body help it interact more efficiently with the environment and perform tasks.

* **Key Facts**:

  * **Rodney Brooks** (MIT) famously argued that true **intelligence** arises through **physical interactions** with the world. In his view, intelligence is not something that occurs purely in the brain, but emerges as the robot moves and interacts with its environment.
  * **Morphological Computation** implies that robots don't only rely on sensors or algorithms but can use their body structure as a means of processing data.
  * This idea is foundational in the development of **humanoid robots** and **autonomous machines** where physical capabilities are integral to cognitive processes.

* **References**:

  * Brooks, R. (1991). "Intelligence Without Representation". *Artificial Intelligence*.
  * **Morphological Computation**: Goossens, S. A. H. M., et al. (2014). "Morphological Computation: A New Approach to Robotics". *IEEE Robotics and Automation Magazine*.

---

#### **1.3 Perception-Action Loop Fundamentals**

* **Key Concepts**:

  * **Perception-Action Loop**: This fundamental concept in robotics and AI refers to the feedback loop between **perception** and **action**. Robots continuously gather sensory data (e.g., visual, tactile, auditory), process this information, and make decisions about how to act on it. These actions, in turn, affect the environment, which generates new sensory data, completing the loop.
  * **Closed-Loop Systems**: In a closed-loop system, sensory inputs are processed and lead to actions, which are then assessed through feedback. This loop allows robots to adapt and respond in real-time to environmental changes.

* **Key Facts**:

  * The **Perception-Action Loop** is crucial for autonomous robots. It enables robots to learn from the environment, make decisions, and refine their actions based on feedback.
  * This concept is implemented in robots that need to interact with the real world, such as **humanoid robots**, **autonomous vehicles**, and **smart machines**.

---

#### **1.4 Humanoid Robotics Market Data**

* **Key Concepts**:

  * **Humanoid Robotics Market**: Humanoid robots are becoming increasingly important in sectors such as manufacturing, healthcare, and personal assistance. Market projections suggest that the **humanoid robotics industry** will reach a **$154B** valuation by **2035**.
  * **Key Players**: Companies like **Tesla**, **Boston Dynamics**, **Unitree Robotics**, and **Figure AI** are making significant advancements in humanoid robotics. These companies are developing robots that are capable of performing tasks that require human-like dexterity, agility, and decision-making.

* **Key Facts**:

  * According to **Goldman Sachs**, the humanoid robotics market is projected to experience massive growth, especially in **labor-saving automation**.
  * Key players like **Tesla's Optimus**, **Boston Dynamics Atlas**, and **Unitree G1/H1** are pushing the boundaries of what humanoid robots can do, with applications ranging from **industrial work** to **elderly care**.

* **References**:

  * **Goldman Sachs Report (2021)**: "The Future of Robotics: The Humanoid Robotics Market".
  * **Tesla Optimus Updates**: Tesla's official announcement on the humanoid robot project.

---

#### **1.5 Tesla Optimus, Figure 01/02, Unitree G1/H1, Boston Dynamics Atlas — Current Status**

* **Key Concepts**:

  * **Tesla Optimus**: Tesla is developing a humanoid robot designed for repetitive industrial tasks and potentially household chores. It is still in the prototype phase but could revolutionize the robotics industry by providing robots that can adapt to a variety of tasks.
  * **Figure 01/02**: **Figure AI** is working on humanoid robots designed to perform tasks like caregiving, service work, and even personal assistance. These robots are meant to interact with people and help in everyday tasks.
  * **Unitree Robotics**: **Unitree’s G1/H1** quadruped robots are advancing rapidly in terms of mobility, and while not strictly humanoid, they share many key design elements, such as complex movement and autonomous navigation.
  * **Boston Dynamics Atlas**: A leader in humanoid robot development, Atlas is known for its agility and complex movements, including backflips, running, and obstacle navigation.

* **Key Facts**:

  * **Tesla Optimus** is a humanoid robot designed to improve productivity by handling repetitive tasks in factories. Tesla aims to mass-produce Optimus for deployment in various sectors.
  * **Boston Dynamics Atlas** is a highly advanced humanoid robot with exceptional movement capabilities and is used for research purposes.

* **References**:

  * **Tesla Optimus**: "Tesla’s Humanoid Robot for the Future" (Tesla, 2022).
  * **Boston Dynamics Atlas**: "Boston Dynamics Atlas: The New Age of Robots" (Boston Dynamics, 2022).

---

#### **1.6 Sensor Technologies: LIDAR Principles, Depth Cameras (Intel RealSense), IMU Mechanics, Force/Torque Sensors**

* **Key Concepts**:

  * **LIDAR** (Light Detection and Ranging): LIDAR sensors measure distance by emitting laser pulses and measuring the time it takes for them to return. These sensors are used for mapping and navigation.
  * **Depth Cameras (Intel RealSense)**: Depth cameras, such as **Intel RealSense**, provide 3D vision by measuring depth using infrared light. These cameras are crucial for robots to understand their surroundings in three dimensions.
  * **IMUs (Inertial Measurement Units)**: IMUs measure linear acceleration, angular velocity, and orientation, which are essential for maintaining balance and precise movement.
  * **Force/Torque Sensors**: These sensors detect the forces and torques applied to a robot's body or limbs, crucial for tasks like manipulation and walking, where feedback is needed to maintain stability.

* **Key Facts**:

  * **LIDAR** sensors are widely used for **mapping** and **obstacle detection** in both autonomous vehicles and humanoid robots.
  * **Intel RealSense** cameras are used for **3D depth sensing**, allowing robots to perceive their environment in real time.
  * **IMUs** provide feedback on orientation and balance, allowing robots to adjust their posture or movement based on environmental factors.
  * **Force/Torque Sensors** are integral to enabling robots to interact delicately with objects, such as picking up fragile items.

* **References**:

  * **Intel RealSense**: Intel's official documentation on **RealSense Cameras**.
  * **LIDAR Technology**: "LIDAR for Robotics" by **Waymo** (2022).
  * **Sensors in Robotics**: "The Role of Sensors in Robotics" by **IEEE Robotics** (2021).

---

### **Chapter 2: ROS 2 Architecture**

#### **2.1 ROS 2 Humble Architecture and DDS Middleware**

* **Key Concepts**:

  * **ROS 2 Humble**: ROS 2 Humble Hawksbill is a long-term support (LTS) release of ROS 2, which focuses on improving real-time capabilities, security, and multi-platform support (Ubuntu, Windows, macOS). The architecture builds on DDS (Data Distribution Service) for communication between nodes.
  * **DDS (Data Distribution Service)**: DDS is the middleware protocol that handles communication between nodes in ROS 2. It allows **peer-to-peer communication** without requiring a central server, making the system more scalable and fault-tolerant. DDS is designed for **real-time**, **high-performance**, and **secure** communication with **quality-of-service (QoS)** controls that allow fine-tuning of communication properties such as reliability and message delivery rates.

* **Key Facts**:

  * ROS 2's **peer-to-peer communication** model using DDS makes the system **scalable** and **fault-tolerant**.
  * **Quality of Service (QoS)** settings in DDS help control how messages are exchanged (e.g., reliable, best-effort, etc.), which is critical for robotics applications that require predictable and high-performance behavior.
  * **Real-time communication** is a central feature of DDS, making ROS 2 suitable for applications requiring timely data exchange.

* **Code Example**:
  Here's a simple example of how a ROS 2 **publisher** and **subscriber** interact using DDS middleware. The publisher will send messages, and the subscriber will listen to those messages asynchronously.

  **Publisher (Python)**:

  ```python
  import rclpy
  from rclpy.node import Node
  from std_msgs.msg import String

  class MinimalPublisher(Node):
      def __init__(self):
          super().__init__('minimal_publisher')
          self.publisher_ = self.create_publisher(String, 'topic', 10)
          self.timer = self.create_timer(1.0, self.timer_callback)

      def timer_callback(self):
          msg = String()
          msg.data = 'Hello, ROS 2!'
          self.publisher_.publish(msg)
          self.get_logger().info(f'Publishing: "{msg.data}"')

  def main():
      rclpy.init()
      node = MinimalPublisher()
      rclpy.spin(node)
      rclpy.shutdown()

  if __name__ == '__main__':
      main()
  ```

  **Subscriber (Python)**:

  ```python
  import rclpy
  from rclpy.node import Node
  from std_msgs.msg import String

  class MinimalSubscriber(Node):
      def __init__(self):
          super().__init__('minimal_subscriber')
          self.subscription = self.create_subscription(
              String,
              'topic',
              self.listener_callback,
              10
          )

      def listener_callback(self, msg):
          self.get_logger().info(f'Received: "{msg.data}"')

  def main():
      rclpy.init()
      node = MinimalSubscriber()
      rclpy.spin(node)
      rclpy.shutdown()

  if __name__ == '__main__':
      main()
  ```

* **References**:

  * **ROS 2 Official Documentation**: [ROS 2 Humble Overview](https://docs.ros.org/en/humble/index.html)
  * **DDS for ROS 2**: [Understanding DDS Middleware](https://design.ros2.org/articles/ros_on_dds.html)

---

#### **2.2 Node Lifecycle and rclpy API Patterns**

* **Key Concepts**:

  * **Node Lifecycle**: The **node lifecycle** in ROS 2 is a mechanism that allows nodes to transition between various states (e.g., **inactive**, **active**, **shutting down**). This explicit lifecycle management enables better resource allocation, predictable system behavior, and smoother integration into large-scale robotic systems.

    * **Configuring**: Node is in the setup state, allocating resources but not yet running.
    * **Inactive**: Node is not processing, but can be activated.
    * **Active**: Node is actively performing its tasks and handling data.
    * **Shutting Down**: Node is deactivating and releasing resources.
  * **rclpy API**: The **rclpy** library is the Python client library that allows users to interact with the ROS 2 system. It provides essential classes and methods for creating nodes, publishers, subscribers, and services. Nodes created using `rclpy` follow the standard ROS 2 node lifecycle.

* **Key Facts**:

  * ROS 2’s **node lifecycle** ensures nodes are more predictable and easier to manage, especially in robotic systems with complex multi-node configurations.
  * **rclpy API** follows a simple structure to create and manage ROS 2 nodes, making it accessible for Python developers to build robotic systems.

* **Code Example**:
  A basic example of a ROS 2 node that logs messages when initialized and manages its lifecycle:

  ```python
  import rclpy
  from rclpy.node import Node

  class MyNode(Node):
      def __init__(self):
          super().__init__('my_node')
          self.get_logger().info('Node initialized!')

      def lifecycle_callback(self):
          self.get_logger().info('Lifecycle callback triggered!')

  def main():
      rclpy.init()
      node = MyNode()
      rclpy.spin(node)
      rclpy.shutdown()

  if __name__ == '__main__':
      main()
  ```

* **References**:

  * **ROS 2 Node Lifecycle**: [Node Lifecycle Management](https://design.ros2.org/articles/node_lifecycle.html)
  * **rclpy Documentation**: [rclpy API](https://docs.ros2.org/en/humble/api/rclpy/)

---

#### **2.3 Topic/Publisher/Subscriber Patterns with Twist, LaserScan, Image Messages**

* **Key Concepts**:

  * **Topic Communication**: ROS 2 uses a **publish-subscribe** model to exchange messages between nodes. A publisher sends messages to a topic, and subscribers listen to the topic and receive messages asynchronously. This allows nodes to communicate without direct coupling.
  * **Message Types**:

    * **Twist**: Represents velocity commands for controlling robot motion. It is commonly used in mobile robots.
    * **LaserScan**: Represents data from LIDAR sensors, useful for obstacle detection and mapping.
    * **Image**: Represents data from a camera sensor, often used in vision-based navigation tasks.

* **Key Facts**:

  * **Twist** messages are used for controlling robot velocities in linear and angular directions.
  * **LaserScan** messages provide data from LIDAR sensors, typically used for distance measurement.
  * **Image** messages carry camera data, which is often processed in tasks like object detection, SLAM (Simultaneous Localization and Mapping), or vision-based control.

* **Code Example**:
  A basic example demonstrating the **Twist** message for controlling robot velocity and a **LaserScan** subscriber.

  **Publisher Example** (using **Twist** to control robot velocity):

  ```python
  import rclpy
  from rclpy.node import Node
  from geometry_msgs.msg import Twist

  class VelocityPublisher(Node):
      def __init__(self):
          super().__init__('velocity_publisher')
          self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
          self.timer = self.create_timer(0.5, self.publish_velocity)

      def publish_velocity(self):
          msg = Twist()
          msg.linear.x = 0.5  # Forward velocity
          msg.angular.z = 0.1  # Rotate in place
          self.publisher.publish(msg)
          self.get_logger().info(f'Publishing: "{msg}"')

  def main():
      rclpy.init()
      node = VelocityPublisher()
      rclpy.spin(node)
      rclpy.shutdown()

  if __name__ == '__main__':
      main()
  ```

  **Subscriber Example** (subscribing to **LaserScan** messages):

  ```python
  import rclpy
  from rclpy.node import Node
  from sensor_msgs.msg import LaserScan

  class LaserScanSubscriber(Node):
      def __init__(self):
          super().__init__('laser_scan_subscriber')
          self.subscription = self.create_subscription(
              LaserScan,
              'scan',
              self.listener_callback,
              10
          )

      def listener_callback(self, msg):
          self.get_logger().info(f'Received LaserScan data with {len(msg.ranges)} range values')

  def main():
      rclpy.init()
      node = LaserScanSubscriber()
      rclpy.spin(node)
      rclpy.shutdown()

  if __name__ == '__main__':
      main()
  ```

* **References**:

  * **ROS 2 Publisher-Subscriber Tutorial**: [Writing a Simple Publisher and Subscriber](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

---

#### **2.4 Service/Action Patterns with Example Service Definitions**

* **Key Concepts**:

  * **Services**: ROS 2 services allow synchronous request-response communication between nodes. A client sends a request to a server, which processes the request and sends back a response.
  * **Actions**: ROS 2 actions are for long-running tasks where feedback is required. Actions allow clients to send a goal to a server, receive feedback, and check the status of the task.

* **Key Facts**:

  * Services use **`.srv`** files for defining request and response types.
  * Actions use **`.action`** files, enabling feedback and preemption of long-running tasks.

* **Code Example**:
  A simple service example that adds two integers together.

  **Service Example** (Service definition with `AddTwoInts`):

  ```python
  from example_interfaces.srv import AddTwoInts
  import rclpy
  from rclpy.node import Node

  class AddTwoIntsClient(Node):
      def __init__(self):
          super().__init__('add_two_ints_client')
          self.client = self.create_client(AddTwoInts, 'add_two_ints')
          while not self.client.wait_for_service(timeout_sec=1.0):
              self.get_logger().info('Service not available, waiting again...')
          self.req = AddTwoInts.Request()
          self.req.a = 5
          self.req.b = 3
          self.future = self.client.call_async(self.req)
          self.future.add_done_callback(self.callback)
      
      def callback(self, future):
          response = future.result()
          self.get_logger().info(f'Result: {response.sum}')

  def main():
      rclpy.init()
      node = AddTwoIntsClient()
      rclpy.spin(node)
      rclpy.shutdown()

  if __name__ == '__main__':
      main()
  ```

* **References**:

  * **ROS 2 Services and Actions**: [ROS 2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Py-Service-And-Client.html)

---

#### **2.5 Computation Graph Concepts and Visualization**

* **Key Concepts**:

  * **Computation Graph**: The ROS 2 computation graph shows how different nodes are connected, how data flows through topics, services, and actions, and how nodes interact.
  * **rqt_graph**: `rqt_graph` is a tool in ROS 2 for visualizing the computation graph. It shows the active nodes, their topics, services, and the relationships between them.

* **References**:

  * **ROS 2 Computation Graph**: [Visualizing the Computation Graph](https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Nodes.html)
  * **rqt_graph Tool**: [rqt_graph Documentation](https://docs.ros2.org/en/humble/api/rqt_graph/)

---

### **Chapter 3: Building with ROS 2**

#### **3.1 ROS 2 Package Structure (setup.py, package.xml, colcon build)**

* **Key Concepts**:

  * **ROS 2 Package Structure**: ROS 2 packages are the fundamental building blocks for creating robotic systems. A ROS 2 package can include various components such as source code, message definitions, services, configuration files, and launch files.
  * **setup.py**: This Python script is used for defining the installation dependencies of a Python-based package. It includes metadata about the package such as its name, version, required dependencies, and installation instructions.
  * **package.xml**: This XML file contains the metadata for a ROS 2 package, including its dependencies, maintainers, version, and other package details. It is used by ROS 2 to handle package dependencies and ensure compatibility.
  * **colcon build**: **colcon** is the official build tool for ROS 2, replacing **catkin** used in ROS 1. It is used to build ROS 2 packages, resolving any inter-package dependencies and compiling the code.

* **Key Facts**:

  * **ROS 2 package structure** encourages modular development. Common directories include:

    * `src/`: Source code.
    * `msg/`: Message definitions.
    * `srv/`: Service definitions.
    * `launch/`: Launch files to start multiple nodes.
    * `config/`: Configuration files.
  * **colcon build** is used to compile the packages. It supports multiple build types such as `ament_cmake` (C++) and `ament_python` (Python).

* **Code Example**:

  * **setup.py** (for Python packages):

    ```python
    from setuptools import setup

    package_name = 'my_ros2_package'
    setup(
        name=package_name,
        version='0.0.1',
        packages=[package_name],
        install_requires=['setuptools'],
        zip_safe=True,
    )
    ```

  * **package.xml** (for ROS 2 package):

    ```xml
    <package format="2">
        <name>my_ros2_package</name>
        <version>0.0.1</version>
        <description>A ROS 2 package example</description>
        <maintainer email="maintainer@example.com">Maintainer</maintainer>
        <license>Apache-2.0</license>
        <depend>rclpy</depend>
        <buildtool_depend>ament_python</buildtool_depend>
    </package>
    ```

* **References**:

  * **Creating a ROS 2 Package**: [ROS 2 Package Structure](https://docs.ros.org/en/humble/Tutorials/Creating-A-Package.html)
  * **Colcon Build Documentation**: [Colcon Tool](https://docs.ros2.org/en/humble/Colcon.html)

---

#### **3.2 rclpy Patterns for Bridging Python AI Code with ROS 2**

* **Key Concepts**:

  * **rclpy**: `rclpy` is the Python client library that interacts with ROS 2, enabling Python-based nodes, publishers, subscribers, and services.
  * **Bridging Python AI Code**: ROS 2 can integrate with machine learning and AI models written in Python, allowing for seamless communication between AI algorithms and robotic systems. Python AI models can be used to process sensor data, make decisions, and send control commands to robotic actuators.

* **Key Facts**:

  * **Publisher/Subscriber Model**: In ROS 2, the publisher/subscriber pattern is used to send and receive data asynchronously between nodes. This allows Python-based AI models to publish their output to topics and subscribe to sensory data for input.
  * **Service/Action Patterns**: AI models can also interact with other ROS 2 nodes using services (synchronous requests) or actions (long-running tasks with feedback).

* **Code Example**:
  Example of integrating a basic **AI model** with ROS 2 using `rclpy`. Here, a simple **linear regression** model predicts robot motion based on input features (e.g., velocity), and the result is published as a `Twist` message to control the robot’s movement.

  **AI Publisher (Python)**:

  ```python
  import rclpy
  from rclpy.node import Node
  from geometry_msgs.msg import Twist
  from sklearn.linear_model import LinearRegression
  import numpy as np

  class AIPublisher(Node):
      def __init__(self):
          super().__init__('ai_publisher')
          self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
          self.timer = self.create_timer(1.0, self.publish_ai_data)
          self.model = LinearRegression()  # Simple AI model for prediction
          # Example: Training data for motion control (simple example)
          X = np.array([[0], [1], [2], [3], [4]])  # Time steps
          y = np.array([0, 0.5, 1, 1.5, 2])  # Speed/velocity
          self.model.fit(X, y)  # Train the model

      def publish_ai_data(self):
          msg = Twist()
          # Example AI prediction (simple linear regression model)
          predicted_velocity = self.model.predict([[5]])[0]  # Predict velocity for time step 5
          msg.linear.x = predicted_velocity
          self.publisher.publish(msg)
          self.get_logger().info(f'Publishing AI-controlled velocity: {predicted_velocity}')

  def main():
      rclpy.init()
      node = AIPublisher()
      rclpy.spin(node)
      rclpy.shutdown()

  if __name__ == '__main__':
      main()
  ```

* **References**:

  * **rclpy API**: [rclpy Documentation](https://docs.ros2.org/en/humble/api/rclpy/)
  * **Integrating Python AI with ROS 2**: [ROS 2 and AI](https://docs.ros2.org/en/humble/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

---

#### **3.3 Launch File Python API (LaunchDescription, Node Actions)**

* **Key Concepts**:

  * **Launch Files**: ROS 2 uses Python-based launch files for managing nodes and configurations. These launch files enable users to configure parameters, manage node execution, and spawn multiple nodes in a coordinated manner.
  * **LaunchDescription**: A class in Python used to organize and configure nodes for execution. Each node is launched as a **Node Action**, which defines its package, executable, and parameters.
  * **Node Actions**: Node actions represent the running of a ROS 2 node and can include configuration options like parameters, node names, and remapping of topics.

* **Key Facts**:

  * ROS 2 launch files written in Python provide greater flexibility than XML-based launch files in ROS 1. They can dynamically add nodes and handle complex configurations.
  * **Node Actions** in the launch file specify what executable to run, along with any parameters needed for that node to function.

* **Code Example**:
  A simple ROS 2 launch file example to launch multiple nodes with different parameters:

  **Python Launch File**:

  ```python
  import launch
  from launch import LaunchDescription
  from launch_ros.actions import Node

  def generate_launch_description():
      return LaunchDescription([
          Node(
              package='my_ros2_package',
              executable='my_node_1',
              name='node_1_name',
              output='screen',
              parameters=[{'robot_name': 'robot_1'}]
          ),
          Node(
              package='my_ros2_package',
              executable='my_node_2',
              name='node_2_name',
              output='screen',
              parameters=[{'robot_name': 'robot_2'}]
          )
      ])
  ```

* **References**:

  * **Creating Launch Files**: [ROS 2 Launch Files](https://docs.ros2.org/en/humble/Tutorials/Creating-Launch-Files.html)
  * **LaunchDescription and Node Actions**: [Launch API Documentation](https://docs.ros2.org/en/humble/api/launch/)

---

#### **3.4 URDF XML Structure for Humanoid Robots (Links, Joints, Kinematics)**

* **Key Concepts**:

  * **URDF**: URDF (Unified Robot Description Format) is an XML format used to describe the structure of a robot, including its links (rigid bodies) and joints (connections between those links). It is used for visualization, simulation, and motion planning.
  * **Links and Joints**: Links represent parts of the robot (e.g., arms, legs, body), and joints define how these parts move relative to each other. Common joint types include **revolute** (rotating), **prismatic** (sliding), and **fixed** (no movement).
  * **Kinematics**: The study of motion without considering forces. In ROS 2, kinematic solvers are used to compute the necessary joint movements to achieve a desired position or orientation (inverse kinematics).

* **Key Facts**:

  * URDF is fundamental for creating **robot simulations** (e.g., **Gazebo**), **visualization** (e.g., **RViz**), and **motion planning**.
  * ROS 2 uses **MoveIt** to plan and execute robot motions, integrating with the URDF files to solve inverse kinematics (IK).

* **Code Example**:
  Example of a simple URDF file representing a humanoid robot with basic links and joints:

  **URDF Example for Humanoid Robot**:

  ```xml
  <?xml version="1.0"?>
  <robot name="humanoid_robot">
      <link name="base_link">
          <visual>
              <geometry>
                  <box size="0.2 0.2 0.2"/>
              </geometry>
          </visual>
      </link>
      <link name="arm_link">
          <visual>
              <geometry>
                  <cylinder radius="0.05" length="0.4"/>
              </geometry>
          </visual>
      </link>
      <joint name="base_to_arm" type="revolute">
          <parent link="base_link"/>
          <child link="arm_link"/>
          <axis xyz="0 0 1"/>
          <limit effort="10" velocity="1" lower="0" upper="1.5"/>
      </joint>
  </robot>
  ```

  This URDF file defines a **simple humanoid robot** with two links (base and arm) and a revolute joint connecting them.

* **References**:

  * **URDF Documentation**: [URDF in ROS 2](https://docs.ros2.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
  * **MoveIt and ROS 2**: [MoveIt for ROS 2](https://moveit.ros.org/)

---