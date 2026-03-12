# generated from rosidl_generator_py/resource/_idl.py.em
# with input from topology_msgs:srv/NavToTopologyPoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_NavToTopologyPoint_Request(type):
    """Metaclass of message 'NavToTopologyPoint_Request'."""

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
                'topology_msgs.srv.NavToTopologyPoint_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__nav_to_topology_point__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__nav_to_topology_point__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__nav_to_topology_point__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__nav_to_topology_point__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__nav_to_topology_point__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavToTopologyPoint_Request(metaclass=Metaclass_NavToTopologyPoint_Request):
    """Message class 'NavToTopologyPoint_Request'."""

    __slots__ = [
        '_point_name',
    ]

    _fields_and_field_types = {
        'point_name': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.point_name = kwargs.get('point_name', str())

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
        if self.point_name != other.point_name:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def point_name(self):
        """Message field 'point_name'."""
        return self._point_name

    @point_name.setter
    def point_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'point_name' field must be of type 'str'"
        self._point_name = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NavToTopologyPoint_Response(type):
    """Metaclass of message 'NavToTopologyPoint_Response'."""

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
                'topology_msgs.srv.NavToTopologyPoint_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__nav_to_topology_point__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__nav_to_topology_point__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__nav_to_topology_point__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__nav_to_topology_point__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__nav_to_topology_point__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavToTopologyPoint_Response(metaclass=Metaclass_NavToTopologyPoint_Response):
    """Message class 'NavToTopologyPoint_Response'."""

    __slots__ = [
        '_is_success',
        '_task_id',
        '_msg',
    ]

    _fields_and_field_types = {
        'is_success': 'boolean',
        'task_id': 'string',
        'msg': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.is_success = kwargs.get('is_success', bool())
        self.task_id = kwargs.get('task_id', str())
        self.msg = kwargs.get('msg', str())

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
        if self.is_success != other.is_success:
            return False
        if self.task_id != other.task_id:
            return False
        if self.msg != other.msg:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def is_success(self):
        """Message field 'is_success'."""
        return self._is_success

    @is_success.setter
    def is_success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_success' field must be of type 'bool'"
        self._is_success = value

    @builtins.property
    def task_id(self):
        """Message field 'task_id'."""
        return self._task_id

    @task_id.setter
    def task_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'task_id' field must be of type 'str'"
        self._task_id = value

    @builtins.property
    def msg(self):
        """Message field 'msg'."""
        return self._msg

    @msg.setter
    def msg(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'msg' field must be of type 'str'"
        self._msg = value


class Metaclass_NavToTopologyPoint(type):
    """Metaclass of service 'NavToTopologyPoint'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('topology_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'topology_msgs.srv.NavToTopologyPoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__nav_to_topology_point

            from topology_msgs.srv import _nav_to_topology_point
            if _nav_to_topology_point.Metaclass_NavToTopologyPoint_Request._TYPE_SUPPORT is None:
                _nav_to_topology_point.Metaclass_NavToTopologyPoint_Request.__import_type_support__()
            if _nav_to_topology_point.Metaclass_NavToTopologyPoint_Response._TYPE_SUPPORT is None:
                _nav_to_topology_point.Metaclass_NavToTopologyPoint_Response.__import_type_support__()


class NavToTopologyPoint(metaclass=Metaclass_NavToTopologyPoint):
    from topology_msgs.srv._nav_to_topology_point import NavToTopologyPoint_Request as Request
    from topology_msgs.srv._nav_to_topology_point import NavToTopologyPoint_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
