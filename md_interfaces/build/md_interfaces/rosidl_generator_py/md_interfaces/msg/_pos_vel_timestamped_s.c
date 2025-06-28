// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from md_interfaces:msg/PosVelTimestamped.idl
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
#include "md_interfaces/msg/detail/pos_vel_timestamped__struct.h"
#include "md_interfaces/msg/detail/pos_vel_timestamped__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool md_interfaces__msg__pos_vel_timestamped__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[57];
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
    assert(strncmp("md_interfaces.msg._pos_vel_timestamped.PosVelTimestamped", full_classname_dest, 56) == 0);
  }
  md_interfaces__msg__PosVelTimestamped * ros_message = _ros_message;
  {  // motor_pos1
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_pos1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motor_pos1 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // motor_vel1
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_vel1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motor_vel1 = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // motor_pos2
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_pos2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motor_pos2 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // motor_vel2
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_vel2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motor_vel2 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // stamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "stamp");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->stamp)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * md_interfaces__msg__pos_vel_timestamped__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PosVelTimestamped */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("md_interfaces.msg._pos_vel_timestamped");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PosVelTimestamped");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  md_interfaces__msg__PosVelTimestamped * ros_message = (md_interfaces__msg__PosVelTimestamped *)raw_ros_message;
  {  // motor_pos1
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motor_pos1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_pos1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_vel1
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motor_vel1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_vel1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_pos2
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motor_pos2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_pos2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_vel2
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motor_vel2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_vel2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stamp
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->stamp);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "stamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
