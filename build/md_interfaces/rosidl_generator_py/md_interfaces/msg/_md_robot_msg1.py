# generated from rosidl_generator_py/resource/_idl.py.em
# with input from md_interfaces:msg/MdRobotMsg1.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MdRobotMsg1(type):
    """Metaclass of message 'MdRobotMsg1'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('md_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'md_interfaces.msg.MdRobotMsg1')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__md_robot_msg1
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__md_robot_msg1
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__md_robot_msg1
            cls._TYPE_SUPPORT = module.type_support_msg__msg__md_robot_msg1
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__md_robot_msg1

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MdRobotMsg1(metaclass=Metaclass_MdRobotMsg1):
    """Message class 'MdRobotMsg1'."""

    __slots__ = [
        '_interval_time',
        '_motor1_rpm',
        '_motor1_pos',
        '_motor2_rpm',
        '_motor2_pos',
        '_motor1_state',
        '_motor2_state',
        '_input_voltage',
    ]

    _fields_and_field_types = {
        'interval_time': 'double',
        'motor1_rpm': 'int16',
        'motor1_pos': 'int32',
        'motor2_rpm': 'int16',
        'motor2_pos': 'int32',
        'motor1_state': 'uint8',
        'motor2_state': 'uint8',
        'input_voltage': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.interval_time = kwargs.get('interval_time', float())
        self.motor1_rpm = kwargs.get('motor1_rpm', int())
        self.motor1_pos = kwargs.get('motor1_pos', int())
        self.motor2_rpm = kwargs.get('motor2_rpm', int())
        self.motor2_pos = kwargs.get('motor2_pos', int())
        self.motor1_state = kwargs.get('motor1_state', int())
        self.motor2_state = kwargs.get('motor2_state', int())
        self.input_voltage = kwargs.get('input_voltage', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.interval_time != other.interval_time:
            return False
        if self.motor1_rpm != other.motor1_rpm:
            return False
        if self.motor1_pos != other.motor1_pos:
            return False
        if self.motor2_rpm != other.motor2_rpm:
            return False
        if self.motor2_pos != other.motor2_pos:
            return False
        if self.motor1_state != other.motor1_state:
            return False
        if self.motor2_state != other.motor2_state:
            return False
        if self.input_voltage != other.input_voltage:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def interval_time(self):
        """Message field 'interval_time'."""
        return self._interval_time

    @interval_time.setter
    def interval_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'interval_time' field must be of type 'float'"
        self._interval_time = value

    @property
    def motor1_rpm(self):
        """Message field 'motor1_rpm'."""
        return self._motor1_rpm

    @motor1_rpm.setter
    def motor1_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor1_rpm' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'motor1_rpm' field must be an integer in [-32768, 32767]"
        self._motor1_rpm = value

    @property
    def motor1_pos(self):
        """Message field 'motor1_pos'."""
        return self._motor1_pos

    @motor1_pos.setter
    def motor1_pos(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor1_pos' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'motor1_pos' field must be an integer in [-2147483648, 2147483647]"
        self._motor1_pos = value

    @property
    def motor2_rpm(self):
        """Message field 'motor2_rpm'."""
        return self._motor2_rpm

    @motor2_rpm.setter
    def motor2_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor2_rpm' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'motor2_rpm' field must be an integer in [-32768, 32767]"
        self._motor2_rpm = value

    @property
    def motor2_pos(self):
        """Message field 'motor2_pos'."""
        return self._motor2_pos

    @motor2_pos.setter
    def motor2_pos(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor2_pos' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'motor2_pos' field must be an integer in [-2147483648, 2147483647]"
        self._motor2_pos = value

    @property
    def motor1_state(self):
        """Message field 'motor1_state'."""
        return self._motor1_state

    @motor1_state.setter
    def motor1_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor1_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'motor1_state' field must be an unsigned integer in [0, 255]"
        self._motor1_state = value

    @property
    def motor2_state(self):
        """Message field 'motor2_state'."""
        return self._motor2_state

    @motor2_state.setter
    def motor2_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor2_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'motor2_state' field must be an unsigned integer in [0, 255]"
        self._motor2_state = value

    @property
    def input_voltage(self):
        """Message field 'input_voltage'."""
        return self._input_voltage

    @input_voltage.setter
    def input_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'input_voltage' field must be of type 'float'"
        self._input_voltage = value
