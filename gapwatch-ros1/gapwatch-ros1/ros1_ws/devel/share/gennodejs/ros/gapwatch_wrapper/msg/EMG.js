// Auto-generated. Do not edit!

// (in-package gapwatch_wrapper.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class EMG {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.emg = null;
      this.battery = null;
      this.counter = null;
      this.ts = null;
    }
    else {
      if (initObj.hasOwnProperty('emg')) {
        this.emg = initObj.emg
      }
      else {
        this.emg = new Array(80).fill(0);
      }
      if (initObj.hasOwnProperty('battery')) {
        this.battery = initObj.battery
      }
      else {
        this.battery = new Array(1).fill(0);
      }
      if (initObj.hasOwnProperty('counter')) {
        this.counter = initObj.counter
      }
      else {
        this.counter = new Array(1).fill(0);
      }
      if (initObj.hasOwnProperty('ts')) {
        this.ts = initObj.ts
      }
      else {
        this.ts = new Array(1).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EMG
    // Check that the constant length array field [emg] has the right length
    if (obj.emg.length !== 80) {
      throw new Error('Unable to serialize array field emg - length must be 80')
    }
    // Serialize message field [emg]
    bufferOffset = _arraySerializer.float32(obj.emg, buffer, bufferOffset, 80);
    // Check that the constant length array field [battery] has the right length
    if (obj.battery.length !== 1) {
      throw new Error('Unable to serialize array field battery - length must be 1')
    }
    // Serialize message field [battery]
    bufferOffset = _arraySerializer.uint8(obj.battery, buffer, bufferOffset, 1);
    // Check that the constant length array field [counter] has the right length
    if (obj.counter.length !== 1) {
      throw new Error('Unable to serialize array field counter - length must be 1')
    }
    // Serialize message field [counter]
    bufferOffset = _arraySerializer.uint8(obj.counter, buffer, bufferOffset, 1);
    // Check that the constant length array field [ts] has the right length
    if (obj.ts.length !== 1) {
      throw new Error('Unable to serialize array field ts - length must be 1')
    }
    // Serialize message field [ts]
    bufferOffset = _arraySerializer.uint64(obj.ts, buffer, bufferOffset, 1);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EMG
    let len;
    let data = new EMG(null);
    // Deserialize message field [emg]
    data.emg = _arrayDeserializer.float32(buffer, bufferOffset, 80)
    // Deserialize message field [battery]
    data.battery = _arrayDeserializer.uint8(buffer, bufferOffset, 1)
    // Deserialize message field [counter]
    data.counter = _arrayDeserializer.uint8(buffer, bufferOffset, 1)
    // Deserialize message field [ts]
    data.ts = _arrayDeserializer.uint64(buffer, bufferOffset, 1)
    return data;
  }

  static getMessageSize(object) {
    return 330;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gapwatch_wrapper/EMG';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a78d4218423202e0323f407914156a1d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[80] emg
    uint8[1] battery
    uint8[1] counter
    uint64[1] ts
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EMG(null);
    if (msg.emg !== undefined) {
      resolved.emg = msg.emg;
    }
    else {
      resolved.emg = new Array(80).fill(0)
    }

    if (msg.battery !== undefined) {
      resolved.battery = msg.battery;
    }
    else {
      resolved.battery = new Array(1).fill(0)
    }

    if (msg.counter !== undefined) {
      resolved.counter = msg.counter;
    }
    else {
      resolved.counter = new Array(1).fill(0)
    }

    if (msg.ts !== undefined) {
      resolved.ts = msg.ts;
    }
    else {
      resolved.ts = new Array(1).fill(0)
    }

    return resolved;
    }
};

module.exports = EMG;
