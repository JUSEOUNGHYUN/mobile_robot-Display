// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from md_interfaces:msg/MdRobotMsg2.idl
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
#include "md_interfaces/msg/detail/md_robot_msg2__struct.h"
#include "md_interfaces/msg/detail/md_robot_msg2__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool md_interfaces__msg__md_robot_msg2__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[45];
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
    assert(strncmp("md_interfaces.msg._md_robot_msg2.MdRobotMsg2", full_classname_dest, 44) == 0);
  }
  md_interfaces__msg__MdRobotMsg2 * ros_message = _ros_message;
  {  // interval_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "interval_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->interval_time = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // x_pos
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_pos");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->x_pos = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // y_pos
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_pos");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->y_pos = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // angule
    PyObject * field = PyObject_GetAttrString(_pymsg, "angule");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->angule = (int16_t)PyLong_AsLong(field);
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
  {  // linear_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "linear_velocity");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->linear_velocity = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // angular_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "angular_velocity");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->angular_velocity = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // input_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "input_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->input_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * md_interfaces__msg__md_robot_msg2__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MdRobotMsg2 */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("md_interfaces.msg._md_robot_msg2");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MdRobotMsg2");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  md_interfaces__msg__MdRobotMsg2 * ros_message = (md_interfaces__msg__MdRobotMsg2 *)raw_ros_message;
  {  // interval_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->interval_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "interval_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_pos
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->x_pos);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_pos", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_pos
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->y_pos);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_pos", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angule
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->angule);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angule", field);
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
  {  // linear_velocity
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->linear_velocity);
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
    field = PyLong_FromLong(ros_message->angular_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angular_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // input_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->input_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "input_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
