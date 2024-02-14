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

class setgoalRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.point = null;
      this.call = null;
    }
    else {
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = 0;
      }
      if (initObj.hasOwnProperty('call')) {
        this.call = initObj.call
      }
      else {
        this.call = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type setgoalRequest
    // Serialize message field [point]
    bufferOffset = _serializer.int32(obj.point, buffer, bufferOffset);
    // Serialize message field [call]
    bufferOffset = _serializer.string(obj.call, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type setgoalRequest
    let len;
    let data = new setgoalRequest(null);
    // Deserialize message field [point]
    data.point = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [call]
    data.call = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.call);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rmus_solution/setgoalRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a358ca69337c562fa080694379731528';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 point
    string call
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new setgoalRequest(null);
    if (msg.point !== undefined) {
      resolved.point = msg.point;
    }
    else {
      resolved.point = 0
    }

    if (msg.call !== undefined) {
      resolved.call = msg.call;
    }
    else {
      resolved.call = ''
    }

    return resolved;
    }
};

class setgoalResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.res = null;
      this.response = null;
    }
    else {
      if (initObj.hasOwnProperty('res')) {
        this.res = initObj.res
      }
      else {
        this.res = false;
      }
      if (initObj.hasOwnProperty('response')) {
        this.response = initObj.response
      }
      else {
        this.response = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type setgoalResponse
    // Serialize message field [res]
    bufferOffset = _serializer.bool(obj.res, buffer, bufferOffset);
    // Serialize message field [response]
    bufferOffset = _serializer.string(obj.response, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type setgoalResponse
    let len;
    let data = new setgoalResponse(null);
    // Deserialize message field [res]
    data.res = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [response]
    data.response = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.response);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rmus_solution/setgoalResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7c684d66cac50a24a80edabfe629efad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool res
    string response
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new setgoalResponse(null);
    if (msg.res !== undefined) {
      resolved.res = msg.res;
    }
    else {
      resolved.res = false
    }

    if (msg.response !== undefined) {
      resolved.response = msg.response;
    }
    else {
      resolved.response = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: setgoalRequest,
  Response: setgoalResponse,
  md5sum() { return '5dd6c60142d9c86b4dfc9ca1994ff38a'; },
  datatype() { return 'rmus_solution/setgoal'; }
};
