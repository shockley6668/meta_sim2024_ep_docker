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

class graspsignalRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.call = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
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
    // Serializes a message object of type graspsignalRequest
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [call]
    bufferOffset = _serializer.string(obj.call, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type graspsignalRequest
    let len;
    let data = new graspsignalRequest(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [call]
    data.call = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.call);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rmus_solution/graspsignalRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8137b29b76a1895dc6bb6316e834de88';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 mode
    string call
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new graspsignalRequest(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
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

class graspsignalResponse {
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
    // Serializes a message object of type graspsignalResponse
    // Serialize message field [res]
    bufferOffset = _serializer.bool(obj.res, buffer, bufferOffset);
    // Serialize message field [response]
    bufferOffset = _serializer.string(obj.response, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type graspsignalResponse
    let len;
    let data = new graspsignalResponse(null);
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
    return 'rmus_solution/graspsignalResponse';
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
    const resolved = new graspsignalResponse(null);
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
  Request: graspsignalRequest,
  Response: graspsignalResponse,
  md5sum() { return 'ba5c545fc7cb2bb25e9f962a2a65350e'; },
  datatype() { return 'rmus_solution/graspsignal'; }
};
