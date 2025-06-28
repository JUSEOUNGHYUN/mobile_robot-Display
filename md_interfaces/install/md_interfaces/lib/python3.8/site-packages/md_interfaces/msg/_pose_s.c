// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from md_interfaces:msg/Pose.idl
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
#include "md_interfaces/msg/detail/pose__struct.h"
#include "md_interfaces/msg/detail/pose__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool md_interfaces__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[29];
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
    assert(strncmp("md_interfaces.msg._pose.Pose", full_classname_dest, 28) == 0);
  }
  md_interfaces__msg__Pose * ros_message = _ros_message;
  {  // x
    PyObject * field = PyObject_GetAttrString(_pymsg, "x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y
    PyObject * field = PyObject_GetAttrString(_pymsg, "y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // theta
    PyObject * field = PyObject_GetAttrString(_pymsg, "theta");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->theta = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // linear_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "linear_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->linear_velocity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angular_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "angular_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angular_velocity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // us_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "us_1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->us_1 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // us_2
    PyObject * field = PyObject_GetAttrString(_pymsg, "us_2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->us_2 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // us_3
    PyObject * field = PyObject_GetAttrString(_pymsg, "us_3");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->us_3 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // us_4
    PyObject * field = PyObject_GetAttrString(_pymsg, "us_4");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->us_4 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // platform_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "platform_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->platform_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // left_motor_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_motor_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->left_motor_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // right_motor_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_motor_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->right_motor_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * md_interfaces__msg__pose__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Pose */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("md_interfaces.msg._pose");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Pose");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  md_interfaces__msg__Pose * ros_message = (md_interfaces__msg__Pose *)raw_ros_message;
  {  // x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // theta
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->theta);
    {
      int rc = PyObject_SetAttrString(_pymessage, "theta", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // linear_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->linear_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "linear_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angular_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angular_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angular_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // us_1
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->us_1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "us_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // us_2
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->us_2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "us_2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // us_3
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->us_3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "us_3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // us_4
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->us_4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "us_4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // platform_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->platform_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "platform_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_motor_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->left_motor_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_motor_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_motor_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->right_motor_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_motor_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
