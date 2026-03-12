# generated from rosidl_generator_py/resource/_idl.py.em
# with input from topology_msgs:msg/RouteInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RouteInfo(type):
    """Metaclass of message 'RouteInfo'."""

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
            module = import_type_support('topology_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'topology_msgs.msg.RouteInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__route_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__route_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__route_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__route_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__route_info

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RouteInfo(metaclass=Metaclass_RouteInfo):
    """Message class 'RouteInfo'."""

    __slots__ = [
        '_controller',
        '_goal_checker',
        '_speed_limit',
    ]

    _fields_and_field_types = {
        'controller': 'string',
        'goal_checker': 'string',
        'speed_limit': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.controller = kwargs.get('controller', str())
        self.goal_checker = kwargs.get('goal_checker', str())
        self.speed_limit = kwargs.get('speed_limit', float())

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
        if self.controller != other.controller:
            return False
        if self.goal_checker != other.goal_checker:
            return False
        if self.speed_limit != other.speed_limit:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def controller(self):
        """Message field 'controller'."""
        return self._controller

    @controller.setter
    def controller(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'controller' field must be of type 'str'"
        self._controller = value

    @builtins.property
    def goal_checker(self):
        """Message field 'goal_checker'."""
        return self._goal_checker

    @goal_checker.setter
    def goal_checker(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'goal_checker' field must be of type 'str'"
        self._goal_checker = value

    @builtins.property
    def speed_limit(self):
        """Message field 'speed_limit'."""
        return self._speed_limit

    @speed_limit.setter
    def speed_limit(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed_limit' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'speed_limit' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._speed_limit = value
