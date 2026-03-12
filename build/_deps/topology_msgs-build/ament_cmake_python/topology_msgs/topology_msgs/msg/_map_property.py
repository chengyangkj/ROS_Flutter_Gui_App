# generated from rosidl_generator_py/resource/_idl.py.em
# with input from topology_msgs:msg/MapProperty.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MapProperty(type):
    """Metaclass of message 'MapProperty'."""

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
                'topology_msgs.msg.MapProperty')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__map_property
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__map_property
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__map_property
            cls._TYPE_SUPPORT = module.type_support_msg__msg__map_property
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__map_property

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MapProperty(metaclass=Metaclass_MapProperty):
    """Message class 'MapProperty'."""

    __slots__ = [
        '_support_controllers',
        '_support_goal_checkers',
    ]

    _fields_and_field_types = {
        'support_controllers': 'sequence<string>',
        'support_goal_checkers': 'sequence<string>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.support_controllers = kwargs.get('support_controllers', [])
        self.support_goal_checkers = kwargs.get('support_goal_checkers', [])

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
        if self.support_controllers != other.support_controllers:
            return False
        if self.support_goal_checkers != other.support_goal_checkers:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def support_controllers(self):
        """Message field 'support_controllers'."""
        return self._support_controllers

    @support_controllers.setter
    def support_controllers(self, value):
        if __debug__:
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'support_controllers' field must be a set or sequence and each value of type 'str'"
        self._support_controllers = value

    @builtins.property
    def support_goal_checkers(self):
        """Message field 'support_goal_checkers'."""
        return self._support_goal_checkers

    @support_goal_checkers.setter
    def support_goal_checkers(self, value):
        if __debug__:
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'support_goal_checkers' field must be a set or sequence and each value of type 'str'"
        self._support_goal_checkers = value
