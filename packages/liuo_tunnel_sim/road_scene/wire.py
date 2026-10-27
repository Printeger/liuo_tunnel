"""Semantic identity excludes unspecified CDR alignment padding, preserves every field."""
import json
from rosidl_runtime_py.convert import message_to_ordereddict


def semantic_bytes(msg):
  if hasattr(msg, 'point_step'):
    metadata = dict(header=message_to_ordereddict(msg.header),
                    height=msg.height,
                    width=msg.width,
                    fields=[message_to_ordereddict(f) for f in msg.fields],
                    is_bigendian=msg.is_bigendian,
                    point_step=msg.point_step,
                    row_step=msg.row_step,
                    is_dense=msg.is_dense)
    return json.dumps(metadata, sort_keys=True, separators=(',', ':')).encode() + bytes(msg.data)
  return json.dumps(message_to_ordereddict(msg), sort_keys=True, separators=(',', ':'), allow_nan=True).encode()
