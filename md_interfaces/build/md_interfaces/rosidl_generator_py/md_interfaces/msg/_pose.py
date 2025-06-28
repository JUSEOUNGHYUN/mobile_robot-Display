# generated from rosidl_generator_py/resource/_idl.py.em
# with input from md_interfaces:msg/Pose.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Pose(type):
    """Metaclass of message 'Pose'."""

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
                'md_interfaces.msg.Pose')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__pose
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__pose
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__pose
            cls._TYPE_SUPPORT = module.type_support_msg__msg__pose
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__pose

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Pose(metaclass=Metaclass_Pose):
    """Message class 'Pose'."""

    __slots__ = [
        '_x',
        '_y',
        '_theta',
        '_linear_velocity',
        '_angular_velocity',
        '_us_1',
        '_us_2',
        '_us_3',
        '_us_4',
        '_platform_state',
        '_left_motor_state',
        '_right_motor_state',
    ]

    _fields_and_field_types = {
        'x': 'float',
        'y': 'float',
        'theta': 'float',
        'linear_velocity': 'float',
        'angular_velocity': 'float',
        'us_1': 'uint8',
        'us_2': 'uint8',
        'us_3': 'uint8',
        'us_4': 'uint8',
        'platform_state': 'uint8',
        'left_motor_state': 'uint8',
        'right_motor_state': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.theta = kwargs.get('theta', float())
        self.linear_velocity = kwargs.get('linear_velocity', float())
        self.angular_velocity = kwargs.get('angular_velocity', float())
        self.us_1 = kwargs.get('us_1', int())
        self.us_2 = kwargs.get('us_2', int())
        self.us_3 = kwargs.get('us_3', int())
        self.us_4 = kwargs.get('us_4', int())
        self.platform_state = kwargs.get('platform_state', int())
        self.left_motor_state = kwargs.get('left_motor_state', int())
        self.right_motor_state = kwargs.get('right_motor_state', int())

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
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.theta != other.theta:
            return False
        if self.linear_velocity != other.linear_velocity:
            return False
        if self.angular_velocity != other.angular_velocity:
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
        if self.left_motor_state != other.left_motor_state:
            return False
        if self.right_motor_state != other.right_motor_state:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
        self._x = value

    @property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
        self._y = value

    @property
    def theta(self):
        """Message field 'theta'."""
        return self._theta

    @theta.setter
    def theta(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'theta' field must be of type 'float'"
        self._theta = value

    @property
    def linear_velocity(self):
        """Message field 'linear_velocity'."""
        return self._linear_velocity

    @linear_velocity.setter
    def linear_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'linear_velocity' field must be of type 'float'"
        self._linear_velocity = value

    @property
    def angular_velocity(self):
        """Message field 'angular_velocity'."""
        return self._angular_velocity

    @angular_velocity.setter
    def angular_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angular_velocity' field must be of type 'float'"
        self._angular_velocity = value

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
    def left_motor_state(self):
        """Message field 'left_motor_state'."""
        return self._left_motor_state

    @left_motor_state.setter
    def left_motor_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_motor_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'left_motor_state' field must be an unsigned integer in [0, 255]"
        self._left_motor_state = value

    @property
    def right_motor_state(self):
        """Message field 'right_motor_state'."""
        return self._right_motor_state

    @right_motor_state.setter
    def right_motor_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_motor_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'right_motor_state' field must be an unsigned integer in [0, 255]"
        self._right_motor_state = value
