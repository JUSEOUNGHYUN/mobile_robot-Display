# generated from rosidl_generator_py/resource/_idl.py.em
# with input from md_interfaces:msg/MdRobotUserParam.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MdRobotUserParam(type):
    """Metaclass of message 'MdRobotUserParam'."""

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
                'md_interfaces.msg.MdRobotUserParam')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__md_robot_user_param
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__md_robot_user_param
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__md_robot_user_param
            cls._TYPE_SUPPORT = module.type_support_msg__msg__md_robot_user_param
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__md_robot_user_param

            from md_interfaces.msg import MdRobotSlowStartStop
            if MdRobotSlowStartStop.__class__._TYPE_SUPPORT is None:
                MdRobotSlowStartStop.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MdRobotUserParam(metaclass=Metaclass_MdRobotUserParam):
    """Message class 'MdRobotUserParam'."""

    __slots__ = [
        '_param_bit_select',
        '_slow_sd',
        '_max_linear_speed_usr',
        '_max_angular_speed_usr',
        '_brake_type',
    ]

    _fields_and_field_types = {
        'param_bit_select': 'uint16',
        'slow_sd': 'md_interfaces/MdRobotSlowStartStop',
        'max_linear_speed_usr': 'double',
        'max_angular_speed_usr': 'double',
        'brake_type': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['md_interfaces', 'msg'], 'MdRobotSlowStartStop'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.param_bit_select = kwargs.get('param_bit_select', int())
        from md_interfaces.msg import MdRobotSlowStartStop
        self.slow_sd = kwargs.get('slow_sd', MdRobotSlowStartStop())
        self.max_linear_speed_usr = kwargs.get('max_linear_speed_usr', float())
        self.max_angular_speed_usr = kwargs.get('max_angular_speed_usr', float())
        self.brake_type = kwargs.get('brake_type', int())

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
        if self.param_bit_select != other.param_bit_select:
            return False
        if self.slow_sd != other.slow_sd:
            return False
        if self.max_linear_speed_usr != other.max_linear_speed_usr:
            return False
        if self.max_angular_speed_usr != other.max_angular_speed_usr:
            return False
        if self.brake_type != other.brake_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def param_bit_select(self):
        """Message field 'param_bit_select'."""
        return self._param_bit_select

    @param_bit_select.setter
    def param_bit_select(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'param_bit_select' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'param_bit_select' field must be an unsigned integer in [0, 65535]"
        self._param_bit_select = value

    @property
    def slow_sd(self):
        """Message field 'slow_sd'."""
        return self._slow_sd

    @slow_sd.setter
    def slow_sd(self, value):
        if __debug__:
            from md_interfaces.msg import MdRobotSlowStartStop
            assert \
                isinstance(value, MdRobotSlowStartStop), \
                "The 'slow_sd' field must be a sub message of type 'MdRobotSlowStartStop'"
        self._slow_sd = value

    @property
    def max_linear_speed_usr(self):
        """Message field 'max_linear_speed_usr'."""
        return self._max_linear_speed_usr

    @max_linear_speed_usr.setter
    def max_linear_speed_usr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_linear_speed_usr' field must be of type 'float'"
        self._max_linear_speed_usr = value

    @property
    def max_angular_speed_usr(self):
        """Message field 'max_angular_speed_usr'."""
        return self._max_angular_speed_usr

    @max_angular_speed_usr.setter
    def max_angular_speed_usr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_angular_speed_usr' field must be of type 'float'"
        self._max_angular_speed_usr = value

    @property
    def brake_type(self):
        """Message field 'brake_type'."""
        return self._brake_type

    @brake_type.setter
    def brake_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'brake_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'brake_type' field must be an unsigned integer in [0, 255]"
        self._brake_type = value
