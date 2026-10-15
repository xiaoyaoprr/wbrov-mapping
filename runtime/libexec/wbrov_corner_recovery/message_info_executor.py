"""Humble-only private executor adapter preserving actual rmw receive metadata.

The installed Python wrapper discards publisher_gid. This uses the same live
rcl subscription pointer while its Destroyable handle is held, takes once via
the public serialized rcl API, and deserializes using that subscription's type.
No installed ROS file or package is modified.
"""
from pathlib import Path
from types import SimpleNamespace
import ctypes
import inspect

from rclpy.executors import SingleThreadedExecutor, await_or_execute
from rclpy.serialization import deserialize_message


class MessageInfoExecutor(SingleThreadedExecutor):
    def __init__(self, *args, **kwargs):
        # This private adaptation targets the inspected Humble executor shape.
        # Refuse a different installed implementation pending a fresh review.
        if 'return msg_info[0]' not in inspect.getsource(SingleThreadedExecutor._take_subscription):
            raise RuntimeError('unreviewed rclpy executor metadata interface')
        super().__init__(*args, **kwargs)
        self.native = ctypes.CDLL(str(Path(__file__).with_name('libwbrov_message_info.so')))
        self.native.wbrov_gid_size.restype = ctypes.c_size_t
        self.gid_size = self.native.wbrov_gid_size()
        if not 16 <= self.gid_size <= 32:
            raise RuntimeError('unsupported actual RMW GID storage')
        self.native.wbrov_message_free.argtypes = [ctypes.c_void_p]
        self.native.wbrov_message_free.restype = None
        self.native.wbrov_take_with_info.argtypes = [
            ctypes.c_size_t, ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_size_t),
            ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_int64), ctypes.POINTER(ctypes.c_int64),
            ctypes.POINTER(ctypes.c_char), ctypes.c_size_t]
        self.native.wbrov_take_with_info.restype = ctypes.c_int

    def _take_subscription(self, sub):
        if not getattr(sub, '_wbrov_message_info_required', False):
            return super()._take_subscription(sub)
        output=ctypes.c_void_p();length=ctypes.c_size_t()
        gid=(ctypes.c_uint8*self.gid_size)();source=ctypes.c_int64();received=ctypes.c_int64()
        error=ctypes.create_string_buffer(1024)
        with sub.handle:
            pointer=sub.handle.pointer
            if type(pointer) is not int or pointer <= 0:
                raise RuntimeError('invalid live rcl subscription pointer')
            ret=self.native.wbrov_take_with_info(pointer,ctypes.byref(output),ctypes.byref(length),
                gid,self.gid_size,ctypes.byref(source),ctypes.byref(received),error,len(error))
        if ret == 0:
            return None
        if ret != 1:
            raise RuntimeError(error.value.decode('utf-8',errors='replace') or 'native receive failed')
        try:
            data=ctypes.string_at(output,length.value)
        finally:
            self.native.wbrov_message_free(output)
        if not any(gid):
            raise RuntimeError('received message has no actual publisher GID')
        message=data if sub.raw else deserialize_message(data,sub.msg_type)
        return message,SimpleNamespace(publisher_gid=bytes(gid),source_timestamp=source.value,
                                       received_timestamp=received.value)

    async def _execute_subscription(self, sub, taken):
        if not getattr(sub, '_wbrov_message_info_required', False):
            return await super()._execute_subscription(sub,taken)
        if taken is not None:
            await await_or_execute(sub.callback,*taken)
