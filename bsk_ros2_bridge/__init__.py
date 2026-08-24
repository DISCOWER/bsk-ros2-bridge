# bsk_ros2_bridge/__init__.py
__all__ = ['RosBridgeHandler']


def __getattr__(name):
    if name == 'RosBridgeHandler':
        from .rosBridgeHandler import RosBridgeHandler
        return RosBridgeHandler
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
