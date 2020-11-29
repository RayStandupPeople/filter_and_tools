# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: types.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='types.proto',
  package='pb_types',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=b'\n\x0btypes.proto\x12\x08pb_types\"<\n\x07LogFile\x12\x11\n\tframe_num\x18\x01 \x01(\x05\x12\x1e\n\x05\x66rame\x18\x02 \x03(\x0b\x32\x0f.pb_types.Frame\"\x90\x02\n\x05\x46rame\x12\n\n\x02id\x18\x01 \x01(\x05\x12$\n\x08obstacle\x18\x02 \x03(\x0b\x32\x12.pb_types.Obstacle\x12-\n\x0clocalization\x18\x03 \x01(\x0b\x32\x17.pb_types.Locallization\x12&\n\thdmapinfo\x18\x04 \x01(\x0b\x32\x13.pb_types.HDMapInfo\x12(\n\nvehiclests\x18\x05 \x01(\x0b\x32\x14.pb_types.VehicleSts\x12.\n\rstatusmachine\x18\x06 \x01(\x0b\x32\x17.pb_types.StatusMachine\x12$\n\x08userinfo\x18\x07 \x01(\x0b\x32\x12.pb_types.UserInfo\"O\n\rLocallization\x12\x0b\n\x03lat\x18\x01 \x01(\x02\x12\x0b\n\x03lon\x18\x02 \x01(\x02\x12\x11\n\tlatRest_X\x18\x03 \x01(\x02\x12\x11\n\tlonRest_Y\x18\x04 \x01(\x02\"1\n\tHDMapInfo\x12\t\n\x01x\x18\x01 \x01(\x05\x12\t\n\x01y\x18\x02 \x01(\x05\x12\x0e\n\x06onpath\x18\x03 \x01(\x08\"=\n\nVehicleSts\x12\x15\n\rvehicle_speed\x18\x01 \x01(\x02\x12\x18\n\x10steerwheel_angle\x18\x02 \x01(\x02\"8\n\rStatusMachine\x12\x0f\n\x07\x61vp_req\x18\x01 \x01(\x05\x12\x16\n\x0eTBOX_AVPModKey\x18\x02 \x01(\x05\"h\n\x08Obstacle\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x0c\n\x04type\x18\x02 \x01(\x05\x12\r\n\x05pos_x\x18\x03 \x01(\x02\x12\r\n\x05pos_y\x18\x04 \x01(\x02\x12\x11\n\trel_spd_x\x18\x05 \x01(\x02\x12\x11\n\trel_spd_y\x18\x06 \x01(\x02\"\x91\x01\n\x08UserInfo\x12\x15\n\robj_left_flag\x18\x01 \x01(\x08\x12\x14\n\x0cobj_mid_flag\x18\x02 \x01(\x08\x12\x16\n\x0eobj_right_flag\x18\x03 \x01(\x08\x12\x14\n\x0cobj_left_s_l\x18\x04 \x01(\x02\x12\x13\n\x0bobj_mid_s_m\x18\x05 \x01(\x02\x12\x15\n\robj_right_s_r\x18\x06 \x01(\x02\x62\x06proto3'
)




_LOGFILE = _descriptor.Descriptor(
  name='LogFile',
  full_name='pb_types.LogFile',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='frame_num', full_name='pb_types.LogFile.frame_num', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='frame', full_name='pb_types.LogFile.frame', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=25,
  serialized_end=85,
)


_FRAME = _descriptor.Descriptor(
  name='Frame',
  full_name='pb_types.Frame',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='pb_types.Frame.id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='obstacle', full_name='pb_types.Frame.obstacle', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='localization', full_name='pb_types.Frame.localization', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='hdmapinfo', full_name='pb_types.Frame.hdmapinfo', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='vehiclests', full_name='pb_types.Frame.vehiclests', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='statusmachine', full_name='pb_types.Frame.statusmachine', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='userinfo', full_name='pb_types.Frame.userinfo', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=88,
  serialized_end=360,
)


_LOCALLIZATION = _descriptor.Descriptor(
  name='Locallization',
  full_name='pb_types.Locallization',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='lat', full_name='pb_types.Locallization.lat', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='lon', full_name='pb_types.Locallization.lon', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='latRest_X', full_name='pb_types.Locallization.latRest_X', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='lonRest_Y', full_name='pb_types.Locallization.lonRest_Y', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=362,
  serialized_end=441,
)


_HDMAPINFO = _descriptor.Descriptor(
  name='HDMapInfo',
  full_name='pb_types.HDMapInfo',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='pb_types.HDMapInfo.x', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='y', full_name='pb_types.HDMapInfo.y', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='onpath', full_name='pb_types.HDMapInfo.onpath', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=443,
  serialized_end=492,
)


_VEHICLESTS = _descriptor.Descriptor(
  name='VehicleSts',
  full_name='pb_types.VehicleSts',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='vehicle_speed', full_name='pb_types.VehicleSts.vehicle_speed', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='steerwheel_angle', full_name='pb_types.VehicleSts.steerwheel_angle', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=494,
  serialized_end=555,
)


_STATUSMACHINE = _descriptor.Descriptor(
  name='StatusMachine',
  full_name='pb_types.StatusMachine',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='avp_req', full_name='pb_types.StatusMachine.avp_req', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='TBOX_AVPModKey', full_name='pb_types.StatusMachine.TBOX_AVPModKey', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=557,
  serialized_end=613,
)


_OBSTACLE = _descriptor.Descriptor(
  name='Obstacle',
  full_name='pb_types.Obstacle',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='pb_types.Obstacle.id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='type', full_name='pb_types.Obstacle.type', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pos_x', full_name='pb_types.Obstacle.pos_x', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pos_y', full_name='pb_types.Obstacle.pos_y', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rel_spd_x', full_name='pb_types.Obstacle.rel_spd_x', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rel_spd_y', full_name='pb_types.Obstacle.rel_spd_y', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=615,
  serialized_end=719,
)


_USERINFO = _descriptor.Descriptor(
  name='UserInfo',
  full_name='pb_types.UserInfo',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='obj_left_flag', full_name='pb_types.UserInfo.obj_left_flag', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='obj_mid_flag', full_name='pb_types.UserInfo.obj_mid_flag', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='obj_right_flag', full_name='pb_types.UserInfo.obj_right_flag', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='obj_left_s_l', full_name='pb_types.UserInfo.obj_left_s_l', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='obj_mid_s_m', full_name='pb_types.UserInfo.obj_mid_s_m', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='obj_right_s_r', full_name='pb_types.UserInfo.obj_right_s_r', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=722,
  serialized_end=867,
)

_LOGFILE.fields_by_name['frame'].message_type = _FRAME
_FRAME.fields_by_name['obstacle'].message_type = _OBSTACLE
_FRAME.fields_by_name['localization'].message_type = _LOCALLIZATION
_FRAME.fields_by_name['hdmapinfo'].message_type = _HDMAPINFO
_FRAME.fields_by_name['vehiclests'].message_type = _VEHICLESTS
_FRAME.fields_by_name['statusmachine'].message_type = _STATUSMACHINE
_FRAME.fields_by_name['userinfo'].message_type = _USERINFO
DESCRIPTOR.message_types_by_name['LogFile'] = _LOGFILE
DESCRIPTOR.message_types_by_name['Frame'] = _FRAME
DESCRIPTOR.message_types_by_name['Locallization'] = _LOCALLIZATION
DESCRIPTOR.message_types_by_name['HDMapInfo'] = _HDMAPINFO
DESCRIPTOR.message_types_by_name['VehicleSts'] = _VEHICLESTS
DESCRIPTOR.message_types_by_name['StatusMachine'] = _STATUSMACHINE
DESCRIPTOR.message_types_by_name['Obstacle'] = _OBSTACLE
DESCRIPTOR.message_types_by_name['UserInfo'] = _USERINFO
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LogFile = _reflection.GeneratedProtocolMessageType('LogFile', (_message.Message,), {
  'DESCRIPTOR' : _LOGFILE,
  '__module__' : 'types_pb2'
  # @@protoc_insertion_point(class_scope:pb_types.LogFile)
  })
_sym_db.RegisterMessage(LogFile)

Frame = _reflection.GeneratedProtocolMessageType('Frame', (_message.Message,), {
  'DESCRIPTOR' : _FRAME,
  '__module__' : 'types_pb2'
  # @@protoc_insertion_point(class_scope:pb_types.Frame)
  })
_sym_db.RegisterMessage(Frame)

Locallization = _reflection.GeneratedProtocolMessageType('Locallization', (_message.Message,), {
  'DESCRIPTOR' : _LOCALLIZATION,
  '__module__' : 'types_pb2'
  # @@protoc_insertion_point(class_scope:pb_types.Locallization)
  })
_sym_db.RegisterMessage(Locallization)

HDMapInfo = _reflection.GeneratedProtocolMessageType('HDMapInfo', (_message.Message,), {
  'DESCRIPTOR' : _HDMAPINFO,
  '__module__' : 'types_pb2'
  # @@protoc_insertion_point(class_scope:pb_types.HDMapInfo)
  })
_sym_db.RegisterMessage(HDMapInfo)

VehicleSts = _reflection.GeneratedProtocolMessageType('VehicleSts', (_message.Message,), {
  'DESCRIPTOR' : _VEHICLESTS,
  '__module__' : 'types_pb2'
  # @@protoc_insertion_point(class_scope:pb_types.VehicleSts)
  })
_sym_db.RegisterMessage(VehicleSts)

StatusMachine = _reflection.GeneratedProtocolMessageType('StatusMachine', (_message.Message,), {
  'DESCRIPTOR' : _STATUSMACHINE,
  '__module__' : 'types_pb2'
  # @@protoc_insertion_point(class_scope:pb_types.StatusMachine)
  })
_sym_db.RegisterMessage(StatusMachine)

Obstacle = _reflection.GeneratedProtocolMessageType('Obstacle', (_message.Message,), {
  'DESCRIPTOR' : _OBSTACLE,
  '__module__' : 'types_pb2'
  # @@protoc_insertion_point(class_scope:pb_types.Obstacle)
  })
_sym_db.RegisterMessage(Obstacle)

UserInfo = _reflection.GeneratedProtocolMessageType('UserInfo', (_message.Message,), {
  'DESCRIPTOR' : _USERINFO,
  '__module__' : 'types_pb2'
  # @@protoc_insertion_point(class_scope:pb_types.UserInfo)
  })
_sym_db.RegisterMessage(UserInfo)


# @@protoc_insertion_point(module_scope)
