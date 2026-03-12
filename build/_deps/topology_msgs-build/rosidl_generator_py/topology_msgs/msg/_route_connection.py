# generated from rosidl_generator_py/resource/_idl.py.em
# with input from topology_msgs:msg/RouteConnection.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RouteConnection(type):
    """Metaclass of message 'RouteConnection'."""

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
                'topology_msgs.msg.RouteConnection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__route_connection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__route_connection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__route_connection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__route_connection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__route_connection

            from topology_msgs.msg import RouteInfo
            if RouteInfo.__class__._TYPE_SUPPORT is None:
                RouteInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RouteConnection(metaclass=Metaclass_RouteConnection):
    """Message class 'RouteConnection'."""

    __slots__ = [
        '_from_point',
        '_to_point',
        '_route_info',
    ]

    _fields_and_field_types = {
        'from_point': 'string',
        'to_point': 'string',
        'route_info': 'topology_msgs/RouteInfo',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['topology_msgs', 'msg'], 'RouteInfo'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.from_point = kwargs.get('from_point', str())
        self.to_point = kwargs.get('to_point', str())
        from topology_msgs.msg import RouteInfo
        self.route_info = kwargs.get('route_info', RouteInfo())

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
        if self.from_point != other.from_point:
            return False
        if self.to_point != other.to_point:
            return False
        if self.route_info != other.route_info:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def from_point(self):
        """Message field 'from_point'."""
        return self._from_point

    @from_point.setter
    def from_point(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'from_point' field must be of type 'str'"
        self._from_point = value

    @builtins.property
    def to_point(self):
        """Message field 'to_point'."""
        return self._to_point

    @to_point.setter
    def to_point(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'to_point' field must be of type 'str'"
        self._to_point = value

    @builtins.property
    def route_info(self):
        """Message field 'route_info'."""
        return self._route_info

    @route_info.setter
    def route_info(self, value):
        if __debug__:
            from topology_msgs.msg import RouteInfo
            assert \
                isinstance(value, RouteInfo), \
                "The 'route_info' field must be a sub message of type 'RouteInfo'"
        self._route_info = value
