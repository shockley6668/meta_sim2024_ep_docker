// Auto-generated. Do not edit!

// (in-package rmus_solution.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class switchRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type switchRequest
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type switchRequest
    let len;
    let data = new switchRequest(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rmus_solution/switchRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '89b81386720be1cd0ce7a3953fcd1b19';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 mode
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new switchRequest(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    return resolved;
    }
};

class switchResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type switchResponse
    // Serialize message field [success]
    bufferOffset = _serializer.uint8(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type switchResponse
    let len;
    let data = new switchResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rmus_solution/switchResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6cca7c80398b8b31af04b80534923f16';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new switchResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: switchRequest,
  Response: switchResponse,
  md5sum() { return '9ae8834e0ab84ed7c29d5f2c87a2b979'; },
  datatype() { return 'rmus_solution/switch'; }
};
