import ROSLIB from 'roslib';
import 'roslib/build/roslib';

export default {
  data () {
    return {
      arm_operations: ['load_left', 'unload_left', 'standby', 'unload_right', 'load_right'],
      nav_pose_operations: ['goto_cell', 'goto_coords'],
      ros: null,
      ros_connected: false,
      manual_control_enabled: true,
      cmd_vel_publisher: undefined,
      arm_operation_publisher: undefined,
      navigate_to_pose_operation_publisher: undefined,
      navigate_to_pose_operation_listener: undefined,
      lin_vel: 0.0,
      ang_vel: 0.0,
      vel_coeff: 0.8,
      is_arm_operation_in_process: false,
      last_arm_operation_result: false,
      queue_operation_in_process: false,
      queue_execution: true,
      cur_operation_status: '',
      cur_operation: undefined,
      cur_operation_id: undefined,
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

      if (this.cur_operation) {
        if (message.data) {
          this.cur_operation.status = 'Success';
          setTimeout(() => {
            this.delOperationFromQueue(this.cur_operation_id);
            this.cur_operation_id = undefined;
            this.cur_operation = undefined;
          }, 1000)
        } else {
          this.cur_operation.status = 'Failed';
          this.queue_execution = false;
        }
      }
    });

    // ----------------------- COMMAND_QUEUE-------------------

    this.navigate_to_pose_operation_publisher = new ROSLIB.Topic({
      ros : ros,
      name : '/call_navigate_to_pose',
      messageType : 'geometry_msgs/PoseStamped'
    });

    var navigate_to_pose_operation_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/navigate_to_pose_result',
      messageType : 'std_msgs/Bool'
    });

    navigate_to_pose_operation_listener.subscribe((message) => {
      console.log('Nav to pose operation result: ', message);
      if (this.cur_operation) {
        if (message.data) {
          this.cur_operation.status = 'Success';
          setTimeout(() => {
            this.delOperationFromQueue(this.cur_operation_id);
            this.cur_operation_id = undefined;
            this.cur_operation = undefined;
          }, 1000)
        } else {
          this.cur_operation.status = 'Failed';
          this.queue_execution = false;
        }
      }
    });

    setInterval(this.process_operation_queue, 1000);
  },
  methods: {
    process_operation_queue() {
      if (Object.keys(this.operation_queue).length && this.queue_execution){
        let first_op_in_queue = this.operation_queue[Object.keys(this.operation_queue)[0]];
        // console.log('First op in queue: ', first_op_in_queue);

        if (first_op_in_queue.status != 'Executing'
            && first_op_in_queue.status != 'Success'
            && first_op_in_queue.status != 'Failed') {
          if (this.arm_operations.includes(first_op_in_queue.operation)) {
            this.publish_arm_operation(first_op_in_queue.operation);
          } else {
            this.publish_nav_to_pose_operation(this.inputed_coords)
          }
          this.cur_operation_id = Object.keys(this.operation_queue)[0];
          this.cur_operation = first_op_in_queue;
          this.cur_operation.status = 'Executing';
        }
      }
      // if (this.adding_operation == 'goto_coords') {
      //   this.publish_nav_to_pose_operation(this.inputed_coords)
      // }
    },
    publish_nav_to_pose_operation(pose) {
      if (this.ros_connected) {
        var pose = new ROSLIB.Message({
          header: {stamp: {sec: 0}, frame_id: 'map'},
          pose: {
            position : {
              x : pose.x,
              y : pose.y,
              z : 0.0
            },
            orientation : {
              x : 0.0,
              y : 0.0,
              z : pose.yaw,
              w : 1.0
            }
        }
        });
        this.navigate_to_pose_operation_publisher.publish(pose);
      }
    },
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
        this.publish_arm_operation('load_left')
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