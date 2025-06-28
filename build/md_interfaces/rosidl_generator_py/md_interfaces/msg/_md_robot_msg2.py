# generated from rosidl_generator_py/resource/_idl.py.em
# with input from md_interfaces:msg/MdRobotMsg2.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MdRobotMsg2(type):
    """Metaclass of message 'MdRobotMsg2'."""

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
                'md_interfaces.msg.MdRobotMsg2')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__md_robot_msg2
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__md_robot_msg2
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__md_robot_msg2
            cls._TYPE_SUPPORT = module.type_support_msg__msg__md_robot_msg2
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__md_robot_msg2

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MdRobotMsg2(metaclass=Metaclass_MdRobotMsg2):
    """Message class 'MdRobotMsg2'."""

    __slots__ = [
        '_interval_time',
        '_x_pos',
        '_y_pos',
        '_angule',
        '_us_1',
        '_us_2',
        '_us_3',
        '_us_4',
        '_platform_state',
        '_linear_velocity',
        '_angular_velocity',
        '_input_voltage',
    ]

    _fields_and_field_types = {
        'interval_time': 'double',
        'x_pos': 'int32',
        'y_pos': 'int32',
        'angule': 'int16',
        'us_1': 'uint8',
        'us_2': 'uint8',
        'us_3': 'uint8',
        'us_4': 'uint8',
        'platform_state': 'uint8',
        'linear_velocity': 'int16',
        'angular_velocity': 'int16',
        'input_voltage': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.interval_time = kwargs.get('interval_time', float())
        self.x_pos = kwargs.get('x_pos', int())
        self.y_pos = kwargs.get('y_pos', int())
        self.angule = kwargs.get('angule', int())
        self.us_1 = kwargs.get('us_1', int())
        self.us_2 = kwargs.get('us_2', int())
        self.us_3 = kwargs.get('us_3', int())
        self.us_4 = kwargs.get('us_4', int())
        self.platform_state = kwargs.get('platform_state', int())
        self.linear_velocity = kwargs.get('linear_velocity', int())
        self.angular_velocity = kwargs.get('angular_velocity', int())
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
        if self.x_pos != other.x_pos:
            return False
        if self.y_pos != other.y_pos:
            return False
        if self.angule != other.angule:
            return False
        if self.us_1 != other.us_1:
            return False
        if self.us_2 != other.us_2:
            return False
        if self.us_3 != other.us_3:
            return False
        if self.us_4 != other.us_4:
            return False
        if self.platform_state != other.platform_state:
            return False
        if self.linear_velocity != other.linear_velocity:
            return False
        if self.angular_velocity != other.angular_velocity:
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
    def x_pos(self):
        """Message field 'x_pos'."""
        return self._x_pos

    @x_pos.setter
    def x_pos(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'x_pos' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'x_pos' field must be an integer in [-2147483648, 2147483647]"
        self._x_pos = value

    @property
    def y_pos(self):
        """Message field 'y_pos'."""
        return self._y_pos

    @y_pos.setter
    def y_pos(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'y_pos' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'y_pos' field must be an integer in [-2147483648, 2147483647]"
        self._y_pos = value

    @property
    def angule(self):
        """Message field 'angule'."""
        return self._angule

    @angule.setter
    def angule(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'angule' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'angule' field must be an integer in [-32768, 32767]"
        self._angule = value

    @property
    def us_1(self):
        """Message field 'us_1'."""
        return self._us_1

    @us_1.setter
    def us_1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'us_1' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'us_1' field must be an unsigned integer in [0, 255]"
        self._us_1 = value

    @property
    def us_2(self):
        """Message field 'us_2'."""
        return self._us_2

    @us_2.setter
    def us_2(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'us_2' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'us_2' field must be an unsigned integer in [0, 255]"
        self._us_2 = value

    @property
    def us_3(self):
        """Message field 'us_3'."""
        return self._us_3

    @us_3.setter
    def us_3(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'us_3' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'us_3' field must be an unsigned integer in [0, 255]"
        self._us_3 = value

    @property
    def us_4(self):
        """Message field 'us_4'."""
        return self._us_4

    @us_4.setter
    def us_4(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'us_4' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'us_4' field must be an unsigned integer in [0, 255]"
        self._us_4 = value

    @property
    def platform_state(self):
        """Message field 'platform_state'."""
        return self._platform_state

    @platform_state.setter
    def platform_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'platform_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'platform_state' field must be an unsigned integer in [0, 255]"
        self._platform_state = value

    @property
    def linear_velocity(self):
        """Message field 'linear_velocity'."""
        return self._linear_velocity

    @linear_velocity.setter
    def linear_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'linear_velocity' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'linear_velocity' field must be an integer in [-32768, 32767]"
        self._linear_velocity = value

    @property
    def angular_velocity(self):
        """Message field 'angular_velocity'."""
        return self._angular_velocity

    @angular_velocity.setter
    def angular_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'angular_velocity' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'angular_velocity' field must be an integer in [-32768, 32767]"
        self._angular_velocity = value

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
