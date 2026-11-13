"""Explicit interface mapping shared by runtime, clients and evidence collection."""


def topic_map(config):
    if not config.get('vehicle'):
        return {}
    if config['vehicle'] != 'drone0':
        raise ValueError('This single-aircraft entry owns drone0 only')
    return {
        '/ego/goal': '/drone0/ego/goal',
        '/ego/cancel': '/drone0/ego/cancel',
        '/ego/status': '/drone0/ego/status',
        '/ego/drone0/odom': '/drone0/ego/native/odom',
        '/ego/drone0/points': '/drone0/ego/native/points',
        '/planner/trajectory': '/drone0/planner/trajectory',
        '/planner/failure': '/drone0/planner/failure',
    }
