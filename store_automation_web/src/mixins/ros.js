import ROSLIB from 'roslib';
import 'roslib/build/roslib';

export default {
  data () {
    return {
      ros: null,
      ros_connected: false,
      manual_control_enabled: true,
      cmd_vel_publisher: undefined,
      arm_operation_publisher: undefined,
      lin_vel: 0.0,
      ang_vel: 0.0,
      vel_coeff: 0.5,
      is_arm_operation_in_process: false,
      last_arm_operation_result: false
    }
  },
  mounted() {
    let ws_address = 'ws://localhost:9090'
    const ros = new window.ROSLIB.Ros({
      url: ws_address,
    });

    this.connected = false;
    const connect = () => {
      ros.on("connection", () => {
          this.ros_connected = true;
          console.log("ROS Connected!");
      });
      ros.on("error", (error) => {
          console.log("Error connecting to ROS bridge websocket server: ", error);
      });
      ros.on("close", () => {
          this.ros_connected = false;
          console.log("Connection to ROS bridge websocket server closed.");
      });
    }
    const disconnect = () => {
        ros.close();
    }
    connect();

    // ----------------------
    var image_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/storebot/compressed_image',
      messageType : 'sensor_msgs/msg/CompressedImage'
    });

    image_listener.subscribe(function(message) {
      document.body.style.backgroundImage = "url('data:image/jpeg;base64," + message.data + "')";
    });

    this.cmd_vel_publisher = new ROSLIB.Topic({
      ros : ros,
      name : '/cmd_vel',
      messageType : 'geometry_msgs/Twist'
    });

    setInterval(this.publish_cmd_vel, 100);

    // -----------------------

    this.arm_operation_publisher = new ROSLIB.Topic({
      ros : ros,
      name : '/call_arm_operation',
      messageType : 'std_msgs/String'
    });

    var arm_operation_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/arm_operation_result',
      messageType : 'std_msgs/msg/String'
    });

    arm_operation_listener.subscribe((message) => {
      console.log('Arm operation result: ', message);
      this.is_arm_operation_in_process = false;
      this.last_arm_operation_result = message.data;
    });
  },
  methods: {
    publish_cmd_vel() {
      if (this.ros_connected && this.manual_control_enabled) {
        var twist = new ROSLIB.Message({
          linear : {
            x : this.lin_vel,
            y : 0.0,
            z : 0.0
          },
          angular : {
            x : 0.0,
            y : 0.0,
            z : this.ang_vel
          }
        });
        this.cmd_vel_publisher.publish(twist);
      }
    },
    publish_arm_operation(operation) {
      if (this.ros_connected && !this.is_arm_operation_in_process) {
        var op = new ROSLIB.Message({
          data: operation
        })

        this.arm_operation_publisher.publish(op);
        this.is_arm_operation_in_process = true;
      }
    }
  },
  watch: {
    "control_buttons.up": function(val) {
      if (val) {
        this.lin_vel = 1.0 * this.vel_coeff
      } else {
        this.lin_vel = 0.0
      }
    },
    "control_buttons.down": function(val) {
      if (val) {
        this.lin_vel = -1.0 * this.vel_coeff
      } else {
        this.lin_vel = 0.0
      }
    },
    "control_buttons.left": function(val) {
      if (val) {
        this.ang_vel = 1.0 * this.vel_coeff
      } else {
        this.ang_vel = 0.0
      }
    },
    "control_buttons.right": function(val) {
      if (val) {
        this.ang_vel = -1.0 * this.vel_coeff
      } else {
        this.ang_vel = 0.0
      }
    },
    "control_buttons.load_left": function(val) {
      if (val) {
        this.publish_arm_operation('load_lft')
      }
    },
    "control_buttons.unload_left": function(val) {
      if (val) {
        this.publish_arm_operation('unload_left')
      }
    },
    "control_buttons.standby": function(val) {
      if (val) {
        this.publish_arm_operation('standby')
      }
    },
    "control_buttons.unload_right": function(val) {
      if (val) {
        this.publish_arm_operation('unload_right')
      }
    },
    "control_buttons.load_right": function(val) {
      if (val) {
        this.publish_arm_operation('load_right')
      }
    },
  }
}