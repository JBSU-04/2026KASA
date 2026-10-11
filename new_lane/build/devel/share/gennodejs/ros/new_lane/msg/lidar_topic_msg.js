// Auto-generated. Do not edit!

// (in-package new_lane.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class lidar_topic_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lane_num = null;
      this.ACC_speed = null;
      this.ACC_flag = null;
      this.ACC_index_gap = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lane_num')) {
        this.lane_num = initObj.lane_num
      }
      else {
        this.lane_num = 0;
      }
      if (initObj.hasOwnProperty('ACC_speed')) {
        this.ACC_speed = initObj.ACC_speed
      }
      else {
        this.ACC_speed = 0.0;
      }
      if (initObj.hasOwnProperty('ACC_flag')) {
        this.ACC_flag = initObj.ACC_flag
      }
      else {
        this.ACC_flag = false;
      }
      if (initObj.hasOwnProperty('ACC_index_gap')) {
        this.ACC_index_gap = initObj.ACC_index_gap
      }
      else {
        this.ACC_index_gap = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lidar_topic_msg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lane_num]
    bufferOffset = _serializer.int32(obj.lane_num, buffer, bufferOffset);
    // Serialize message field [ACC_speed]
    bufferOffset = _serializer.float64(obj.ACC_speed, buffer, bufferOffset);
    // Serialize message field [ACC_flag]
    bufferOffset = _serializer.bool(obj.ACC_flag, buffer, bufferOffset);
    // Serialize message field [ACC_index_gap]
    bufferOffset = _serializer.int32(obj.ACC_index_gap, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lidar_topic_msg
    let len;
    let data = new lidar_topic_msg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lane_num]
    data.lane_num = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ACC_speed]
    data.ACC_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ACC_flag]
    data.ACC_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ACC_index_gap]
    data.ACC_index_gap = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'new_lane/lidar_topic_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f3a2e64a65093ac6a9f5ca405937d5aa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    int32 lane_num
    float64 ACC_speed
    
    bool ACC_flag
    int32 ACC_index_gap
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lidar_topic_msg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lane_num !== undefined) {
      resolved.lane_num = msg.lane_num;
    }
    else {
      resolved.lane_num = 0
    }

    if (msg.ACC_speed !== undefined) {
      resolved.ACC_speed = msg.ACC_speed;
    }
    else {
      resolved.ACC_speed = 0.0
    }

    if (msg.ACC_flag !== undefined) {
      resolved.ACC_flag = msg.ACC_flag;
    }
    else {
      resolved.ACC_flag = false
    }

    if (msg.ACC_index_gap !== undefined) {
      resolved.ACC_index_gap = msg.ACC_index_gap;
    }
    else {
      resolved.ACC_index_gap = 0
    }

    return resolved;
    }
};

module.exports = lidar_topic_msg;
