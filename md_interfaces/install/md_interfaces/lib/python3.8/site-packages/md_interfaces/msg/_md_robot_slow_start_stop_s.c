// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from md_interfaces:msg/MdRobotSlowStartStop.idl
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
#include "md_interfaces/msg/detail/md_robot_slow_start_stop__struct.h"
#include "md_interfaces/msg/detail/md_robot_slow_start_stop__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool md_interfaces__msg__md_robot_slow_start_stop__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[65];
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
    assert(strncmp("md_interfaces.msg._md_robot_slow_start_stop.MdRobotSlowStartStop", full_classname_dest, 64) == 0);
  }
  md_interfaces__msg__MdRobotSlowStartStop * ros_message = _ros_message;
  {  // slowstart
    PyObject * field = PyObject_GetAttrString(_pymsg, "slowstart");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->slowstart = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // slowstop
    PyObject * field = PyObject_GetAttrString(_pymsg, "slowstop");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->slowstop = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * md_interfaces__msg__md_robot_slow_start_stop__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MdRobotSlowStartStop */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("md_interfaces.msg._md_robot_slow_start_stop");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MdRobotSlowStartStop");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  md_interfaces__msg__MdRobotSlowStartStop * ros_message = (md_interfaces__msg__MdRobotSlowStartStop *)raw_ros_message;
  {  // slowstart
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->slowstart);
    {
      int rc = PyObject_SetAttrString(_pymessage, "slowstart", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // slowstop
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->slowstop);
    {
      int rc = PyObject_SetAttrString(_pymessage, "slowstop", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
