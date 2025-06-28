// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from md_interfaces:msg/MdRobotUserParam.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "md_interfaces/msg/detail/md_robot_user_param__struct.h"
#include "md_interfaces/msg/detail/md_robot_user_param__functions.h"

bool md_interfaces__msg__md_robot_slow_start_stop__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * md_interfaces__msg__md_robot_slow_start_stop__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool md_interfaces__msg__md_robot_user_param__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("md_interfaces.msg._md_robot_user_param.MdRobotUserParam", full_classname_dest, 55) == 0);
  }
  md_interfaces__msg__MdRobotUserParam * ros_message = _ros_message;
  {  // param_bit_select
    PyObject * field = PyObject_GetAttrString(_pymsg, "param_bit_select");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->param_bit_select = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // slow_sd
    PyObject * field = PyObject_GetAttrString(_pymsg, "slow_sd");
    if (!field) {
      return false;
    }
    if (!md_interfaces__msg__md_robot_slow_start_stop__convert_from_py(field, &ros_message->slow_sd)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // max_linear_speed_usr
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_linear_speed_usr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_linear_speed_usr = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_angular_speed_usr
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_angular_speed_usr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_angular_speed_usr = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // brake_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "brake_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->brake_type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * md_interfaces__msg__md_robot_user_param__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MdRobotUserParam */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("md_interfaces.msg._md_robot_user_param");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MdRobotUserParam");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  md_interfaces__msg__MdRobotUserParam * ros_message = (md_interfaces__msg__MdRobotUserParam *)raw_ros_message;
  {  // param_bit_select
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->param_bit_select);
    {
      int rc = PyObject_SetAttrString(_pymessage, "param_bit_select", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // slow_sd
    PyObject * field = NULL;
    field = md_interfaces__msg__md_robot_slow_start_stop__convert_to_py(&ros_message->slow_sd);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "slow_sd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_linear_speed_usr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_linear_speed_usr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_linear_speed_usr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_angular_speed_usr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_angular_speed_usr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_angular_speed_usr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // brake_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->brake_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "brake_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
