# generated from rosidl_generator_py/resource/_idl.py.em
# with input from topology_msgs:msg/TopologyMap.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TopologyMap(type):
    """Metaclass of message 'TopologyMap'."""

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
                'topology_msgs.msg.TopologyMap')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__topology_map
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__topology_map
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__topology_map
            cls._TYPE_SUPPORT = module.type_support_msg__msg__topology_map
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__topology_map

            from topology_msgs.msg import MapProperty
            if MapProperty.__class__._TYPE_SUPPORT is None:
                MapProperty.__class__.__import_type_support__()

            from topology_msgs.msg import RouteConnection
            if RouteConnection.__class__._TYPE_SUPPORT is None:
                RouteConnection.__class__.__import_type_support__()

            from topology_msgs.msg import TopologyMapPointInfo
            if TopologyMapPointInfo.__class__._TYPE_SUPPORT is None:
                TopologyMapPointInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TopologyMap(metaclass=Metaclass_TopologyMap):
    """Message class 'TopologyMap'."""

    __slots__ = [
        '_map_name',
        '_map_property',
        '_points',
        '_routes',
    ]

    _fields_and_field_types = {
        'map_name': 'string',
        'map_property': 'topology_msgs/MapProperty',
        'points': 'sequence<topology_msgs/TopologyMapPointInfo>',
        'routes': 'sequence<topology_msgs/RouteConnection>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['topology_msgs', 'msg'], 'MapProperty'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['topology_msgs', 'msg'], 'TopologyMapPointInfo')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['topology_msgs', 'msg'], 'RouteConnection')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.map_name = kwargs.get('map_name', str())
        from topology_msgs.msg import MapProperty
        self.map_property = kwargs.get('map_property', MapProperty())
        self.points = kwargs.get('points', [])
        self.routes = kwargs.get('routes', [])

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
        if self.map_name != other.map_name:
            return False
        if self.map_property != other.map_property:
            return False
        if self.points != other.points:
            return False
        if self.routes != other.routes:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def map_name(self):
        """Message field 'map_name'."""
        return self._map_name

    @map_name.setter
    def map_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'map_name' field must be of type 'str'"
        self._map_name = value

    @builtins.property
    def map_property(self):
        """Message field 'map_property'."""
        return self._map_property

    @map_property.setter
    def map_property(self, value):
        if __debug__:
            from topology_msgs.msg import MapProperty
            assert \
                isinstance(value, MapProperty), \
                "The 'map_property' field must be a sub message of type 'MapProperty'"
        self._map_property = value

    @builtins.property
    def points(self):
        """Message field 'points'."""
        return self._points

    @points.setter
    def points(self, value):
        if __debug__:
            from topology_msgs.msg import TopologyMapPointInfo
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, TopologyMapPointInfo) for v in value) and
                 True), \
                "The 'points' field must be a set or sequence and each value of type 'TopologyMapPointInfo'"
        self._points = value

    @builtins.property
    def routes(self):
        """Message field 'routes'."""
        return self._routes

    @routes.setter
    def routes(self, value):
        if __debug__:
            from topology_msgs.msg import RouteConnection
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, RouteConnection) for v in value) and
                 True), \
                "The 'routes' field must be a set or sequence and each value of type 'RouteConnection'"
        self._routes = value
